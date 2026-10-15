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

APK lokal auf dem Linux-Rechner bauen (ohne GitHub Actions):

```bash
cd ~/ender-qgc && git pull
git submodule update --init --recursive
ender/tools/build_android_local.sh --install     # --install: danach per adb auf die Funke
```

Das Skript baut den Workflow `android.yml` nach (Versionen aus `.github/build-config.json`: Qt, NDK, Java, SDK),
nur für `arm64-v8a`. SDK, NDK, Qt und der Debug-Keystore liegen in `~/ender-android`, die fertige APK in
`~/ender-apk/QGroundControl.apk`, das Build-Log in `~/ender-android/last-build.log`. Der erste Lauf lädt mehrere GB.
Die APK ist mit dem lokalen Keystore signiert, nicht mit dem des Workflows: vor der ersten Installation einmal
`adb uninstall org.mavlink.qgroundcontrol` (die gespeicherten Einstellungen der App gehen dabei verloren), danach
geht `adb install -r` wieder. Stand: noch nicht auf dem Linux-Rechner ausgeführt.

APK über GitHub Actions bauen und aufspielen (Alternative):

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

Alle Nodes mit einem Befehl (statt der letzten drei Zeilen oben und der Befehle im Abschnitt Hindernis-Karte):

```bash
/workspaces/ender-qgc/ender/sim/start_ender_nodes.sh <IP des Laptops>          # Sim, 3000 Punkte je Scan
/workspaces/ender-qgc/ender/sim/start_ender_nodes.sh <IP der Funke> 6000       # echt, 6000 Punkte
/workspaces/ender-qgc/ender/sim/start_ender_nodes.sh --stop                    # alles beenden
```

Das Skript startet `bridge_node`, `map_relay_node`, `goal_node` und Birks `pointcloud_compressor_node` (mit den
Werten per `-p`, siehe unten) in vier tmux-Fenstern `ender-*`. Mit `ENDER_COMPRESSOR=0` entfällt der Kompressor (wenn Birks
Launch-File ihn schon startet), mit `ENDER_ROS_WS=<Pfad>` zeigt man auf einen anderen Workspace (Onboard-PC) und lässt sie offen, falls eine Node mit einem Fehler
endet. Es braucht den gebauten Workspace und läuft SUPER (`sl`, `launch_super`). Im Container wechselst du mit
`Strg+b` und der Fensternummer (`Strg+b w` zeigt die Liste).

Hinweise: In `drone-project` muss `ego.parm` die Werte aus `egoparms.parm` enthalten, sonst fliegt die Sim schlecht
(lokale Änderung, vor `git pull` `git stash`). Die Windows-Firewall muss QGC im privaten Netz zulassen.
`Strg+c` beendet eine Node, `clu` räumt die ganze Sim auf.

### Kameras in der 3D-Ansicht (Drohnen-Ansicht und freie Kamera)

Die Punktwolken-Ansicht hat zwei Kameras, umgeschaltet mit dem Button rechts am Rand:

| | Drohnen-Ansicht (Standard) | Freie Kamera (wie RViz) |
|---|---|---|
| Kamera | kreist um die Drohne, Drohne bleibt in der Mitte und zeigt nach oben | kreist um einen Fokuspunkt in der Welt, bleibt stehen, wenn die Drohne fliegt |
| 1 Finger / linke Maustaste | drehen | drehen |
| 2 Finger | Zoom (ein längeres Ziehen mit zwei Fingern schaltet in die freie Kamera) | Zoom und Verschieben (Pan) |
| Maus | Rad = Zoom | Rad = Zoom, mittlere oder rechte Taste = Pan |
| Button | **Freie Kamera** | **Drohne zentrieren** (zurück in die Drohnen-Ansicht) |

Die freie Kamera braucht die Pose der Drohne in der Welt (`/Odometry`, FAST-LIO). QGC verbindet sich dafür mit der
`map_relay_node` (TCP 19902), auch wenn die Hindernisse aus sind; die Karte wird dann nicht angefordert. Der Scan
(Drohnen-Koordinaten) wird mit der Pose zum Scan-Zeitpunkt in die Welt gestellt, die Pose dazwischen interpoliert.
Ohne Pose bleibt der Scan im Ursprung, unten links steht dann ein Hinweis.

**Nachleuchten:** Scan-Punkte bleiben 5 s sichtbar (`PointCloudView.scanPersistenceMs`, 0 = nur der letzte Scan). Jeder
Scan wird mit der Pose zu seinem Zeitstempel in die Welt gestellt und in ein 10-cm-Raster eingetragen; eine Fläche, die
wieder gesehen wird, frischt nur ihre Zelle auf, die Punktzahl wächst also nicht, solange die Drohne schwebt. Die Punkte
bleiben beim Fliegen dort stehen, wo sie gesehen wurden. Dafür verbindet sich QGC immer mit der `map_relay_node`
(nur Pose, die Karte nur bei "Hindernisse an"). Ohne Pose (Relay-Node läuft nicht) zeigt die Ansicht wie vorher nur den
letzten Scan, die Statuszeile zählt die gezeichneten Punkte. "Ziel wählen" schaltet immer in die
Drohnen-Ansicht (Blick von oben) und danach zurück in die freie Kamera, falls sie an war.

### Hindernis-Karte (SUPER inflate map) in der 3D-Ansicht

Der Button **Hindernisse an/aus** in der Punktwolken-Ansicht zeigt den Sicherheitsabstand, den SUPER um Hindernisse
legt, als orange Ebene. Der Weg der Daten (Format EMAP v1, siehe `src/PointCloud/MapProtocol.h` und
`ender/ros2/ender_lidar_bridge/ender_lidar_bridge/map_protocol.py`):

```text
SUPER /rog_map/inf_occ -> pointcloud_compressor_node (Birk) -> /rog_map/inf_occ_compressed
  -> map_relay_node (TCP-Server, Port 19902, schickt Pose und, auf Wunsch, die Karte) -> QGC
```

Die Relay-Node abonniert erst, wenn QGC verbunden ist, und dann nur die Pose. QGC schickt beim Verbinden eine
Steuernachricht (Typ 3): "Karte an" erst bei **Hindernisse an**; nur dann abonniert die Node die Karte und packt
Birks Kompressor. Beim Ausschalten kommt "Karte aus", ist auch die freie Kamera aus, trennt QGC ganz.
Die Karte bleibt im Weltsystem, in der Szene steht sie ohne Umrechnung; Drohne und Scan werden mit der Pose (10 Hz)
platziert. In QGC werden nur Punkte im Umkreis von 20 m (`MapReceiver.radius`) behalten.

Im Container, zusätzlich zu `sl` und `launch_super` (jeweils eigenes tmux-Fenster):

```bash
source /workspaces/ender-qgc/ender/ros2/install/setup.zsh     # nach colcon build wie oben
ros2 run drone_control_cpp pointcloud_compressor_node --ros-args -p compression_level:=3 -p 'topic_keys:=[rog_map_inf_occ]' -p rog_map_inf_occ.input_topic:=/rog_map/inf_occ -p rog_map_inf_occ.downsample_leaf_size:=0.4 -p rog_map_inf_occ.max_frequency_hz:=1.0
ros2 run ender_lidar_bridge map_relay_node
```

Die Werte kommen per `-p`, denn `--params-file` mit `config/inf_map_compressor.yaml` wurde von Birks Node mit
"unknown ROS arguments" abgelehnt (in der Sim am 2026-10-15; die Ursache ist nicht geklärt, wahrscheinlich die
Liste `topic_keys`). Bei der Liste keine Anführungszeichen um die Einträge setzen: `[rog_map_inf_occ]` geht,
`['rog_map_inf_occ']` nicht. Gemessen mit diesen Werten: etwa 1300 Punkte, 5 KB je Karte.

Die Parameterdatei stellt den Kompressor auf Raster 0,4 m und 1 Hz (Birks Standard: 0,2 m und 2 Hz, etwa 117 KB je
Bild). Läuft der Kompressor schon über Birks Launch-File, darf es ihn nicht doppelt geben; dann muss dort der Wert
angepasst werden, oder die Relay-Node begrenzt nur die Rate (`map_rate_hz`, Standard 1).

Ohne Drohne: `python3 tools/fake_map_server.py` (im Ordner `ender/ros2/ender_lidar_bridge`, braucht
`pip install zstandard numpy`) spielt die Relay-Node nach. Der Laptop zeigt dann einen Raum aus Hüllen, die Drohne
fliegt im Kreis.

### Geplante Bahn (SUPER exp_traj und backup_traj) in der 3D-Ansicht

Der Button **Bahn an/aus** in der Punktwolken-Ansicht zeigt, was SUPER geplant hat, wie in RViz: die Bahn, der die
Drohne folgt (grün, `/visualization/exp_traj`), und die Ausweichbahn (cyan, `/visualization/backup_traj`). Beide sind
Linien im Weltsystem, wie die Karte. Die Bahn kommt über dieselbe TCP-Verbindung wie die Pose (Typ 4, siehe
`MapProtocol.h`), die Relay-Node braucht dafür nichts Zusätzliches, nur die Steuernachricht (Bit 1 "Bahn gewünscht").
Ohne den Button abonniert die Relay-Node die Marker nicht.

SUPER veröffentlicht die Bahnen als `visualization_msgs/MarkerArray` (Best Effort), jede Nachricht mit `DELETEALL` und
dann hunderten kurzen Pfeilen (Typ ARROW, zwei Punkte) im Frame `world`. Die Relay-Node führt denselben Stand wie
RViz, verkettet die Pfeile zu Linien (`trajectory.py`), dünnt auf höchstens `traj_max_points` (3000) aus und rechnet
die Punkte per TF von `world` nach `camera_init` um (in der Sim liegt `camera_init` 19,5 cm über `world`). Fehlt
die TF-Verbindung, bleibt die Bahn unverschoben und das Log warnt. Höchstens `traj_rate_hz` (4) Stände pro Sekunde.

Parameter der Relay-Node: `exp_traj_topic`, `backup_traj_topic`, `traj_rate_hz`, `traj_max_points`, `world_frame`.
`/visualization/committed_traj` ist bewusst nicht dabei (RViz zeigt es in unserer Sim nicht). Die Anzeige in QGC
zeichnet je 4 cm einen kleinen Würfel entlang der Linie (`TrajectoryInstancing`).

### Adresse der Drohne auf der Funke einstellen

Auf der Funke lässt sich keine Umgebungsvariable setzen. Oben in der Mitte der Punktwolken-Ansicht, unter der
QGC-Leiste, steht deshalb "Drohne: <Adresse>" (grün, wenn die Verbindung steht). Antippen öffnet ein Eingabefeld, "Übernehmen" speichert die
Adresse (QSettings, Datei `Ender/EnderGCS.ini`), sie gilt für Ziele, Pose, Karte und Bahn und bleibt nach einem
Neustart erhalten. Leer lassen und übernehmen setzt auf `192.168.144.50` zurück. Beim Start gilt die
Umgebungsvariable `ENDER_GOAL_HOST` (Desktop) vor der gespeicherten Adresse.

Test der Funke gegen die Sim (Funke im WLAN des Linux-Rechners):
1. APK bauen (GitHub, Workflow "Android", Branch `ender/pointcloud`, Run workflow, etwa 55 Minuten), herunterladen,
   `adb install -r <apk>`.
2. IP der Funke im WLAN: `adb shell ip -f inet addr show wlan0`.
3. Im Container: `ender/sim/start_ender_nodes.sh <IP der Funke> 3000` (der Scan geht an die Funke).
4. In QGC auf der Funke "Drohne: ..." (oben in der Mitte) antippen und die IP des Linux-Rechners eintragen.
