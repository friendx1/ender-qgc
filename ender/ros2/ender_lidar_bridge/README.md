# ender_lidar_bridge

ROS2-Node (Python), der die Punktwolke von FAST-LIO2 ausdünnt und per UDP zur Bodenstation
(Funke, 192.168.144.20) schickt. Format: siehe `ender_lidar_bridge/protocol.py` (EPCL v1).

## Auf dem Onboard-PC bauen und starten

```
mkdir -p ~/ender_ws/src && cd ~/ender_ws/src
git clone https://github.com/friendx1/ender-gcs.git   # oder bereits vorhanden
ln -s ender-gcs/ros2/ender_lidar_bridge .
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

## Tests (ohne ROS)

```
python3 -m unittest discover -s . -p "test_*.py" -t . -v
```

`tools/fake_sender.py` schickt eine künstliche Szene, damit sich der Empfänger ohne Drohne testen lässt.
