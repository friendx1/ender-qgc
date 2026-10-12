# Ender Robotics Fork

Fork von QGroundControl (Basis: `Stable_V5.1`) für die Siyi UniRC 7 Pro.

Änderung: ein zusätzliches Fenster in der Fly View, das die aktuelle LiDAR-Punktwolke der Drohne
zeigt (Livox Mid-360, FAST-LIO2, Topic `/cloud_registered_body`). Die Punktwolke kommt per UDP
(Port 19900, Format EPCL v1) vom ROS2-Node `ender_lidar_bridge` auf dem Onboard-PC.
Das Format steht in `ros2/ender_lidar_bridge/ender_lidar_bridge/protocol.py` im Repo `friendx1/ender-gcs`.

Aufbau (neu, alles Weitere bleibt Upstream):

- `src/PointCloud/EpclFrameAssembler` setzt die UDP-Pakete zu kompletten Scans zusammen.
- `src/PointCloud/PointCloudReceiver` lauscht auf UDP 19900 und hält den letzten Scan.
- `src/PointCloud/PointCloudInstancing` zeichnet jeden Punkt als kleinen Würfel, nach Höhe eingefärbt.
- `src/PointCloud/PointCloudQml/PointCloudView.qml` ist die 3D-Ansicht (Ziehen dreht, Mausrad/Pinch zoomt).
- `src/FlyView/FlyViewPipLayout.qml` ordnet Video, Karte und Punktwolke im Fly View an: ein Fenster groß, die anderen beiden klein unten links und unten rechts. Klick auf ein kleines Fenster tauscht es mit dem großen, die Größe wird an der Ecke gezogen. Es ersetzt `PipView` im Fly View. `PipSlotState.qml` ersetzt dort `PipState`. Kompass und Telemetrie liegen unten in der Mitte (`FlyViewWidgetLayer.qml`).
- Tests: `test/PointCloud/` (Label `PointCloud`).

Verbindung zur Drohne: UniGCS auf der Funke auf UDP stellen, in QGC eine UDP-Verbindung mit Port 0
und Server `192.168.144.20:19856` anlegen. Kein USB-C-Kabel an der Funke, es stört den Datenpfad.

Android-Build: GitHub Actions, Workflow "Android", Branch wählen, Artifact `QGroundControl-linux`.

## Entwicklung auf einem zweiten Rechner

Der Code liegt auf GitHub, die APKs baut GitHub Actions. Auf jedem Rechner reichen deshalb:

- Git für Windows (git-scm.com) mit Anmeldung bei GitHub im Browser
- Android platform-tools (`adb`), entweder mit Android Studio oder einzeln von developer.android.com
- ein Texteditor (z. B. VS Code)

Einmalig:

```
git config --global user.name "Finn Kerwien"
git config --global user.email "finn.kerwien8476@gmail.com"
git clone --branch ender/pointcloud https://github.com/friendx1/ender-qgc.git
```

Zum lokalen Bauen von QGC (Qt 6.11, Visual Studio, Android-NDK) sind die Submodule nötig
(`git submodule update --init --recursive`). Für die tägliche Arbeit ist das nicht nötig.

Arbeitsablauf:

```
git pull                     # vor dem Arbeiten
git add -A && git commit -m "feat(...): ..."   # Conventional Commits, siehe AGENTS.md
git push                     # nach dem Arbeiten
```

APK bauen und aufspielen:

1. GitHub, Repo `ender-qgc`, Reiter **Actions**, Workflow **Android**, **Run workflow**, Branch `ender/pointcloud`, Build type **Release**. Dauer etwa eine Stunde.
2. Auf der Seite des Laufs unter **Artifacts** `QGroundControl-linux` herunterladen und entpacken.
3. Funke per USB anschließen und installieren: `adb install -r pfad\zur\QGroundControl.apk`.
4. USB-Kabel abziehen, erst dann in QGC verbinden (UDP, Port 0, Server `192.168.144.20:19856`).

## Desktop-Build auf Windows (schnelles UI-Entwickeln am PC)

Ziel: Änderungen am Fly View am PC bauen und sofort ansehen. Die Android-APK über GitHub ist nur noch der Endtest auf der Funke.
Es gilt Qt **6.11.1** (nur diese Version, siehe `.github/build-config.json`).

Einmalig einrichten:
1. Git und Python 3 installieren (falls nicht vorhanden).
2. Visual Studio 2022 Build Tools mit der Workload "C++ build tools" installieren. Das geht auch mit `python tools/setup/install_dependencies --platform windows --msvc`.
3. Qt 6.11.1 mit dem Qt Online Installer (Qt-Konto nötig): "Custom Installation", Haken bei "Archive", dann Qt 6.11.1 -> "MSVC 2022 64-bit" und alle "Additional Libraries" (darin Qt Quick 3D, Qt Graphs, Qt Location, Qt Multimedia, Qt Speech u. a.). Qt Design Studio abwählen.
4. GStreamer (Video): `python tools/setup/install_dependencies --platform windows` aus dem Repo-Ordner. CMake lädt es sonst beim Konfigurieren selbst.
5. CMake und Ninja installieren (liegen auch im Qt-Installer unter "Tools").
6. Fork mit Unterordnern klonen: `git clone --recurse-submodules https://github.com/friendx1/ender-qgc.git`, dann `git checkout ender/pointcloud`.

Bauen und starten (in der "x64 Native Tools Command Prompt for VS 2022", im Repo-Ordner):
```
C:\Qt\6.11.1\msvc2022_64\bin\qt-cmake.bat -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug
build\Debug\QGroundControl.exe
```
Nach einer Änderung reicht `cmake --build build --config Debug` (baut nur das Geänderte neu).

Punktwolke ohne Drohne testen: `python ros2/ender_lidar_bridge/tools/fake_sender.py` (Repo ender-gcs) sendet Testdaten an 127.0.0.1:19900. QGC hört darauf, sobald das Punktwolken-Fenster sichtbar ist.
