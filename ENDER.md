# Ender Robotics Fork

Fork von QGroundControl (Basis: `Stable_V5.1`) für die Siyi UniRC 7 Pro.

Änderung: ein zusätzliches Fenster in der Fly View, das die aktuelle LiDAR-Punktwolke der Drohne
zeigt (Livox Mid-360, FAST-LIO2, Topic `/cloud_registered_body`). Die Punktwolke kommt per UDP
(Port 19900, Format EPCL v1) vom ROS2-Node `ender_lidar_bridge` auf dem Onboard-PC.
Das Format steht in `ender/ros2/ender_lidar_bridge/ender_lidar_bridge/protocol.py` (dort liegt auch die Node selbst).

Aufbau (neu, alles Weitere bleibt Upstream):

- `src/PointCloud/EpclFrameAssembler` setzt die UDP-Pakete zu kompletten Scans zusammen.
- `src/PointCloud/PointCloudReceiver` lauscht auf UDP 19900 und hält den letzten Scan.
- `src/PointCloud/PointCloudInstancing` zeichnet jeden Punkt als kleinen Würfel, nach Höhe eingefärbt.
- `src/PointCloud/PointCloudQml/PointCloudView.qml` ist die 3D-Ansicht (Ziehen dreht, Mausrad/Pinch zoomt).
- `src/FlyView/FlyViewPipLayout.qml` ordnet Video, Karte und Punktwolke im Fly View an: ein Fenster groß, die anderen beiden klein unten links und unten rechts. Klick auf ein kleines Fenster tauscht es mit dem großen, die Größe wird an der Ecke gezogen. Es ersetzt `PipView` im Fly View. `PipSlotState.qml` ersetzt dort `PipState`. Kompass und Telemetrie liegen unten in der Mitte (`FlyViewWidgetLayer.qml`).
- `src/PointCloud/GoalProtocol` und `GoalSender` schicken ein angeklicktes Flugziel per UDP (Port 19901, Format EGOL v1) an den Onboard-PC und warten auf die Bestätigung. Die Node `goal_node` dort rechnet das Ziel ins Weltsystem um und gibt es an SUPER (`/goal_pose`), siehe `ender/ros2/ender_lidar_bridge/README.md`.
- Zielmodus in `PointCloudView.qml`: Knopf "Ziel wählen" friert den Scan ein und zeigt ihn von oben, Antippen setzt den Zielpunkt, "Senden" schickt ihn. Die Adresse des Onboard-PCs ist `192.168.144.50`, für Tests am PC kann sie mit der Umgebungsvariable `ENDER_GOAL_HOST` überschrieben werden.
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

Einmalig einrichten (so auf zwei Rechnern getestet, Windows 11):
1. **Defender-Ausnahme** für `C:\dev` anlegen (Windows-Sicherheit -> Viren- & Bedrohungsschutz -> Ausschlüsse). Ohne sie hing der moc-Schritt auf dem Laptop sehr lange.
2. Git und Python von python.org installieren (3.13.0 und 3.14.6 liefen beide; "Add to PATH" anhaken, nicht den Store-Python verwenden).
3. **Visual Studio Build Tools 2022** (17.14, nicht 2026) mit der Workload "Desktopentwicklung mit C++". Sie bringen CMake 3.31 und Ninja mit.
4. **Qt 6.11.1** (nur diese Version) mit dem Qt Online Installer: "Benutzerdefinierte Installation", Haken bei "Archive", dann nur MSVC 2022 64-bit und die Zusatzbibliotheken Graphs, Location, Positioning, Speech, Multimedia, Serial Port, Image Formats, Shader Tools, Connectivity, Quick 3D, Sensors, State Machines (= SCXML), WebSockets, HTTP Server.
5. Repo klonen: `git clone --recurse-submodules https://github.com/friendx1/ender-qgc.git C:\dev\ender-qgc`, dann `git checkout ender/pointcloud`.
6. Eigenes CMake nicht installieren: CMake ab 4.1 scheitert an zstd. Die CMake aus den Build Tools (3.31.6) verwenden. CMake lädt und installiert GStreamer beim Konfigurieren selbst.

Bauen (im **"x64 Native Tools Command Prompt for VS 2022"**, nicht im x86-Prompt und nicht in PowerShell). Zur Kontrolle: `echo %VSCMD_ARG_TGT_ARCH%` muss `x64` zeigen.
```
cd /d C:\dev\ender-qgc
set CM="C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
%CM% -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=C:\Qt\6.11.1\msvc2022_64 -DCMAKE_TOOLCHAIN_FILE=C:\Qt\6.11.1\msvc2022_64\lib\cmake\Qt6\qt.toolchain.cmake
%CM% --build build
```
Der Configure legt die Python-Umgebung `.venv` an und installiert GStreamer still (ca. 6 Minuten, nicht abbrechen). Meldet er `.venv has no pip`: `.venv\Scripts\python.exe -m ensurepip --upgrade` und den Configure wiederholen. Nach einem Fehler im Configure vorher `rmdir /s /q build`. `qt-cmake.bat` ging auf dem Laptop nicht (Abbruch ohne Meldung), deshalb der direkte Aufruf.

Der erste Build hat 760 Schritte. Der Schritt "Automatic MOC and UIC for target QGroundControl" gibt lange keine Ausgabe, das ist normal.

Starten: Die App findet Qt und GStreamer nicht von selbst. Lege `C:\dev\run-qgc.bat` an:
```
@echo off
set PATH=C:\dev\ender-qgc\.cache\CPM\gstreamer-win-x86_64-1.28.4\sdk\bin;C:\Qt\6.11.1\msvc2022_64\bin;%PATH%
C:\dev\ender-qgc\build\Debug\QGroundControl.exe
```
Nach einer Änderung im Code reicht `%CM% --build build` (baut nur das Geänderte neu), dann `C:\dev\run-qgc.bat`.

Punktwolke ohne Drohne testen (einmal `python -m pip install numpy`):
```
python C:\dev\ender-qgc\ender\ros2\ender_lidar_bridge\tools\fake_sender.py --ip 127.0.0.1 --port 19900 --hz 5.0 --points 3000
```
QGC hört auf UDP 19900, sobald das Punktwolken-Fenster sichtbar ist. Beim ersten Start fragt die Windows-Firewall, dort private Netzwerke erlauben.

Flugziel ohne Drohne testen: `python ender\ros2\ender_lidar_bridge\tools\fake_goal_receiver.py --port 19901` nimmt Ziele an und bestätigt sie. In `run-qgc.bat` vor dem Start der App `set ENDER_GOAL_HOST=127.0.0.1` einfügen.

## Simulation auf dem Linux-Rechner (Ziel antippen testen)

Die Drohnen-Software liegt im Repo `BirkS04/drone-project` (privat, nur lesend klonen: `gh auth login`, dann
`git clone` nach `~/drone-project` mit Submodulen). Die Sim läuft im Docker-Image des devcontainers, wir starten es
mit NVIDIA-Grafik, ohne dort etwas zu ändern:

```bash
~/ender-qgc/ender/sim/start_sim_container.sh      # auf dem Host; ein zweiter Aufruf öffnet ein weiteres Terminal im Container
```

Im Container (tmux, Fenster mit `Strg+b` und `c`):

```bash
sl                  # Gazebo, ArduPilot SITL, ROS-System
launch_super        # SUPER (eigenes Terminal)
start_rviz          # optional
gat                 # Guided, Arm, Takeoff

cd /workspaces/ender-qgc/ender/ros2
colcon build --packages-select ender_lidar_bridge --symlink-install
source install/setup.zsh
ros2 run ender_lidar_bridge goal_node
ros2 run ender_lidar_bridge bridge_node --ros-args -p dest_ip:=<IP des Laptops>
```

Auf dem Laptop (PowerShell), Laptop und Linux im selben Netz:

```powershell
$env:ENDER_GOAL_HOST = "<IP des Linux-Rechners>"
run-qgc
```

Hinweise: In `drone-project` muss `ego.parm` die Werte aus `egoparms.parm` enthalten, sonst fliegt die Sim schlecht
(lokale Änderung, vor `git pull` `git stash`). Die Windows-Firewall muss QGC im privaten Netz zulassen.
`Strg+c` beendet eine Node, `clu` räumt die ganze Sim auf.
