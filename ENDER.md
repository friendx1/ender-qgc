# Ender Robotics Fork

Fork von QGroundControl (Basis: `Stable_V5.1`) für die Siyi UniRC 7 Pro.

Geplante Änderung: ein zusätzliches Fenster in der Fly View, das die aktuelle LiDAR-Punktwolke
der Drohne zeigt (Livox Mid-360, FAST-LIO2, Topic `/cloud_registered_body`). Die Punktwolke kommt
per UDP (Port 19900, Format EPCL v1) vom ROS2-Node `ender_lidar_bridge` auf dem Onboard-PC.
Das Format steht in `ros2/ender_lidar_bridge/ender_lidar_bridge/protocol.py` im Repo `friendx1/ender-gcs`.

Verbindung zur Drohne: UniGCS auf der Funke auf UDP stellen, in QGC eine UDP-Verbindung mit Port 0
und Server `192.168.144.20:19856` anlegen. Kein USB-C-Kabel an der Funke, es stört den Datenpfad.

Android-Build: GitHub Actions, Workflow "Android", Branch wählen, Artifact `QGroundControl-linux`.
