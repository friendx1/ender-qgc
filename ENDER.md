# Ender Robotics Fork

Fork von QGroundControl (Basis: `Stable_V5.1`) für die Siyi UniRC 7 Pro.

Geplante Änderung: ein zusätzliches Fenster in der Fly View, das die aktuelle LiDAR-Punktwolke
der Drohne zeigt (Livox Mid-360, FAST-LIO2, Topic `/cloud_registered_body`). Die Punktwolke kommt
per UDP (Port 19900, Format EPCL v1) vom ROS2-Node `ender_lidar_bridge` auf dem Onboard-PC.
Das Format steht in `ros2/ender_lidar_bridge/ender_lidar_bridge/protocol.py` im Repo `friendx1/ender-gcs`.

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
