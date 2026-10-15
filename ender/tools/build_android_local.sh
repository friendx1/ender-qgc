#!/usr/bin/env bash
# Baut die Ender-QGC-APK (Android, arm64-v8a) lokal auf dem Linux-Rechner.
# Nachbau von .github/workflows/android.yml, Versionen kommen aus .github/build-config.json.
#
# Aufruf (im Repo-Wurzelverzeichnis, Submodule muessen geklont sein):
#   ender/tools/build_android_local.sh            # alles einrichten (nur beim ersten Mal lang) und bauen
#   ender/tools/build_android_local.sh --install  # danach per adb auf die Funke installieren
#
# Alles Schwere liegt ausserhalb des Repos in ~/ender-android (SDK, NDK, Qt, Keystore).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

TOOLS="${ENDER_ANDROID_HOME:-$HOME/ender-android}"
BUILD_DIR="${ENDER_ANDROID_BUILD:-$TOOLS/build}"
OUT_DIR="${ENDER_ANDROID_OUT:-$HOME/ender-apk}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
ABI="arm64-v8a"          # die Funke ist arm64, armeabi-v7a spart nur Zeit
DO_INSTALL=0
[[ "${1:-}" == "--install" ]] && DO_INSTALL=1

mkdir -p "$TOOLS" "$OUT_DIR"

cfg() { python3 - "$1" <<'PY'
import json, sys
d = json.load(open(".github/build-config.json"))
cur = d
for k in sys.argv[1].split("."):
    cur = cur[k]
print(cur)
PY
}

QT_VERSION="$(cfg qt.version)"
QT_MODULES="$(cfg qt.modules)"
PLATFORM="$(cfg android.platform)"
BUILD_TOOLS="$(cfg android.build_tools)"
CMDLINE="$(cfg android.cmdline_tools)"
NDK_FULL="$(cfg android.ndk_full_version)"
JAVA_VER="$(cfg android.java_version)"

echo "== Versionen: Qt $QT_VERSION, NDK $NDK_FULL, Java $JAVA_VER, Plattform $PLATFORM, build-tools $BUILD_TOOLS"

# 1. Systempakete und Java ------------------------------------------------------------
if [[ ! -f "$TOOLS/.deps-done" ]]; then
  echo "== Systempakete (sudo)"
  sudo apt-get update
  sudo apt-get install -y "openjdk-${JAVA_VER}-jdk-headless" unzip curl git python3-venv ninja-build
  python3 tools/setup/install_dependencies --platform debian --skip-gstreamer || \
    echo "WARN: install_dependencies fehlgeschlagen, Build kann trotzdem gehen"
  touch "$TOOLS/.deps-done"
fi
export JAVA_HOME="${JAVA_HOME:-/usr/lib/jvm/java-${JAVA_VER}-openjdk-amd64}"
[[ -d "$JAVA_HOME" ]] || { echo "JAVA_HOME $JAVA_HOME fehlt (Java $JAVA_VER nicht installiert?)"; exit 1; }
export PATH="$JAVA_HOME/bin:$PATH"

# 2. Python-venv (Generatoren, aqtinstall) und CMake < 4.1 ------------------------------
# CMake 4.1+ hat uns auf Windows beim Bau von zstd Probleme gemacht, deshalb 3.31.x im venv.
if [[ ! -x "$REPO/.venv/bin/python" ]]; then
  python3 -m venv "$REPO/.venv"
fi
"$REPO/.venv/bin/pip" install -q --upgrade pip
"$REPO/.venv/bin/pip" install -q jinja2 pyyaml httpx defusedxml aqtinstall "cmake==3.31.6" ninja
export PATH="$REPO/.venv/bin:$PATH"
cmake --version | head -1

# 3. Android SDK + NDK -----------------------------------------------------------------
export ANDROID_SDK_ROOT="$TOOLS/sdk"
export ANDROID_HOME="$ANDROID_SDK_ROOT"
SDKMANAGER="$ANDROID_SDK_ROOT/cmdline-tools/latest/bin/sdkmanager"
if [[ ! -x "$SDKMANAGER" ]]; then
  echo "== Android cmdline-tools $CMDLINE"
  mkdir -p "$ANDROID_SDK_ROOT/cmdline-tools"
  curl -fL -o "$TOOLS/cmdline-tools.zip" \
    "https://dl.google.com/android/repository/commandlinetools-linux-${CMDLINE}_latest.zip"
  unzip -q -o "$TOOLS/cmdline-tools.zip" -d "$TOOLS/cmdtmp"
  rm -rf "$ANDROID_SDK_ROOT/cmdline-tools/latest"
  mv "$TOOLS/cmdtmp/cmdline-tools" "$ANDROID_SDK_ROOT/cmdline-tools/latest"
  rm -rf "$TOOLS/cmdtmp" "$TOOLS/cmdline-tools.zip"
fi
if [[ ! -d "$ANDROID_SDK_ROOT/ndk/$NDK_FULL" || ! -d "$ANDROID_SDK_ROOT/platforms/android-$PLATFORM" ]]; then
  echo "== SDK-Pakete"
  yes | "$SDKMANAGER" --sdk_root="$ANDROID_SDK_ROOT" --licenses >/dev/null || true
  "$SDKMANAGER" --sdk_root="$ANDROID_SDK_ROOT" \
    "platform-tools" "platforms;android-$PLATFORM" "build-tools;$BUILD_TOOLS" "ndk;$NDK_FULL"
fi
export ANDROID_NDK_ROOT="$ANDROID_SDK_ROOT/ndk/$NDK_FULL"
export ANDROID_NDK_HOME="$ANDROID_NDK_ROOT"
export ANDROID_NDK="$ANDROID_NDK_ROOT"
export PATH="$ANDROID_SDK_ROOT/platform-tools:$PATH"

# 4. Qt (Host-Qt fuer moc & Co. plus Qt fuer Android) --------------------------------------
QT_DIR="$TOOLS/qt"
QT_HOST="$QT_DIR/$QT_VERSION/gcc_64"
QT_ANDROID="$QT_DIR/$QT_VERSION/android_arm64_v8a"
PY="$REPO/.venv/bin/python"
if [[ ! -d "$QT_HOST" ]]; then
  echo "== Qt Host (gcc_64)"
  "$PY" tools/setup/install_qt.py install --version "$QT_VERSION" --host linux --target desktop \
    --arch linux_gcc_64 --outdir "$QT_DIR" --modules "$QT_MODULES"
fi
if [[ ! -d "$QT_ANDROID" ]]; then
  echo "== Qt Android (arm64_v8a)"
  "$PY" tools/setup/install_qt.py install --version "$QT_VERSION" --host linux --target android \
    --arch android_arm64_v8a --outdir "$QT_DIR" --modules "$QT_MODULES"
fi
[[ -d "$QT_HOST" && -d "$QT_ANDROID" ]] || { echo "Qt-Verzeichnisse nicht gefunden unter $QT_DIR"; find "$QT_DIR" -maxdepth 2; exit 1; }

# 5. Eigener Debug-Keystore (bleibt gleich, damit adb install -r spaeter ueber die alte App geht)
KEYSTORE="$TOOLS/ender-debug.keystore"
if [[ ! -f "$KEYSTORE" ]]; then
  echo "== Keystore anlegen"
  keytool -genkey -v -keystore "$KEYSTORE" -storepass android -alias androiddebugkey \
    -keypass android -keyalg RSA -keysize 2048 -validity 10000 \
    -dname "CN=Ender Debug,O=Ender Robotics,C=DE"
fi
export QT_ANDROID_KEYSTORE_PATH="$KEYSTORE"
export QT_ANDROID_KEYSTORE_ALIAS="androiddebugkey"
export QT_ANDROID_KEYSTORE_STORE_PASS="android"
export QT_ANDROID_KEYSTORE_KEY_PASS="android"

# 6. Konfigurieren und bauen --------------------------------------------------------------
echo "== Configure ($BUILD_TYPE, $ABI)"
"$PY" tools/configure.py -S "$REPO" -B "$BUILD_DIR" -G Ninja -t "$BUILD_TYPE" --no-qt-cmake -- \
  -DCMAKE_WARN_DEPRECATED=FALSE \
  -DCMAKE_TOOLCHAIN_FILE="$QT_ANDROID/lib/cmake/Qt6/qt.toolchain.cmake" \
  -DCMAKE_PREFIX_PATH="$QT_ANDROID" \
  -DQT_ANDROID_ABIS="$ABI" \
  -DQT_HOST_PATH="$QT_HOST" \
  -DQT_ANDROID_SIGN_APK=ON \
  -DPython3_EXECUTABLE="$PY"

echo "== Build"
time cmake --build "$BUILD_DIR" --parallel 2>&1 | tee "$TOOLS/last-build.log"

APK="$(ls "$BUILD_DIR"/android-build/*.apk 2>/dev/null | head -1 || true)"
[[ -n "$APK" ]] || { echo "Keine APK gefunden in $BUILD_DIR/android-build (Log: $TOOLS/last-build.log)"; exit 1; }
cp "$APK" "$OUT_DIR/QGroundControl.apk"
echo "== Fertig: $OUT_DIR/QGroundControl.apk"

if [[ $DO_INSTALL -eq 1 ]]; then
  adb devices
  adb install -r "$OUT_DIR/QGroundControl.apk"
fi
