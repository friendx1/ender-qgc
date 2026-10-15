# ender_lidar_bridge

ROS2-Node (Python), der die Punktwolke von FAST-LIO2 ausdünnt und per UDP zur Bodenstation
(Funke, 192.168.144.20) schickt. Format: siehe `ender_lidar_bridge/protocol.py` (EPCL v1).

## Auf dem Onboard-PC bauen und starten

```
mkdir -p ~/ender_ws/src && cd ~/ender_ws/src
git clone https://github.com/friendx1/ender-qgc.git   # oder bereits vorhanden
ln -s ender-qgc/ender/ros2/ender_lidar_bridge .
cd ~/ender_ws && colcon build --packages-select ender_lidar_bridge
source install/setup.bash
ros2 run ender_lidar_bridge bridge_node --ros-args -p dest_ip:=192.168.144.20
```

Voraussetzung: ROS2 Humble oder neuer (`sensor_msgs_py.read_points` liefert ein numpy-Array).
Der Onboard-PC braucht eine eigene feste IP im Netz 192.168.144.x (z. B. 192.168.144.50/24).

## Parameter

| Parameter | Standard | Bedeutung |
|---|---|---|
| `topic` | `/cloud_registered_body` | Eingang (Scan im Koordinatensystem der Drohne) |
| `dest_ip` | `192.168.144.20` | Empfänger (Funke) |
| `dest_port` | `19900` | Empfangsport (nicht 19856, der gehört dem MAVLink-Server der Funke) |
| `voxel_size` | `0.10` | ein Punkt je Würfel von 10 cm |
| `max_points` | `6000` | Obergrenze je Scan |
| `max_rate_hz` | `5.0` | höchstens so viele Scans pro Sekunde |
| `max_range` | `40.0` | weiter entfernte Punkte fallen weg (m) |
| `send_intensity` | `false` | Intensität mitsenden (+1 Byte je Punkt) |

Alle 5 Sekunden steht im Log, wie viele Scans, Punkte und Mbit/s tatsächlich gesendet werden.

## Flugziele von der Bodenstation (goal_node)

Die Bodenstation schickt ein angeklicktes Ziel per UDP (Port 19901, Format EGOL v1, siehe
`ender_lidar_bridge/goal_protocol.py`). Die Node `goal_node` rechnet es mit der Pose zum
Scan-Zeitpunkt (aus `/Odometry`) ins Weltsystem um, prüft es und publiziert eine `PoseStamped`
auf `/goal_pose`, das Topic, auf das SUPER hört (`fsm/click_goal_topic`). Danach bestätigt
sie dem Absender per UDP (EGOA). SUPER setzt die Flughöhe selbst (`fsm/click_height`).

```
ros2 run ender_lidar_bridge goal_node
```

| Parameter | Standard | Bedeutung |
|---|---|---|
| `listen_port` | `19901` | UDP-Port für Ziele |
| `odom_topic` | `/Odometry` | Pose der Drohne (FAST-LIO2) |
| `goal_topic` | `/goal_pose` | Ausgang, Eingang von SUPER |
| `goal_frame` | `camera_init` | Frame im Header (SUPER liest nur die Zahlen) |
| `goal_z` | `1.5` | z im Ziel (SUPER überschreibt es) |
| `max_distance` | `40.0` | weiter entfernte Ziele werden abgelehnt (m) |
| `max_age_s` | `20.0` | so alt darf der Scan beim Senden höchstens sein (die Bodenstation friert ihn beim Wählen ein) |
| `max_pose_gap_s` | `0.3` | größter Abstand zwischen Scan und nächster Pose |

Ohne Drohne testen: `python3 tools/fake_goal_receiver.py --port 19901` nimmt Ziele an und bestätigt sie.

## Hindernis-Karte zur Bodenstation (map_relay_node)

Die Node reicht die Hindernis-Karte, die Birks `pointcloud_compressor_node` schon gepackt auf
`/rog_map/inf_occ_compressed` veröffentlicht, per TCP an die Bodenstation weiter und schickt dazu die Pose der
Drohne (aus `/Odometry`). Sie ist der Server; erst wenn sich die Bodenstation verbindet, abonniert sie, und
sie hört auf, wenn die Bodenstation trennt. Format: `ender_lidar_bridge/map_protocol.py` (EMAP v1).

```
ros2 run ender_lidar_bridge map_relay_node
```

| Parameter | Standard | Bedeutung |
|---|---|---|
| `listen_port` | `19902` | TCP-Port |
| `map_topic` | `/rog_map/inf_occ_compressed` | Eingang (Ausgang des Kompressors) |
| `odom_topic` | `/Odometry` | Pose der Drohne |
| `map_rate_hz` | `1.0` | höchstens so viele Karten je Sekunde weitergeben |
| `pose_rate_hz` | `10.0` | Posen je Sekunde |

Ist die Verbindung langsamer als die Karten kommen, wird nur die neueste Karte gesendet, ältere fallen weg.
`config/inf_map_compressor.yaml` stellt den Kompressor auf Raster 0,4 m und 1 Hz.
Ohne Drohne testen: `python3 tools/fake_map_server.py` (braucht `pip install zstandard numpy`).

## Tests (ohne ROS)

```
python3 -m unittest discover -s . -p "test_*.py" -t . -v
```

`tools/fake_sender.py` schickt eine künstliche Szene, damit sich der Empfänger ohne Drohne testen lässt.
