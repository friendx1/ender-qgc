import QtQuick
import QtQuick3D

import QGroundControl.PointCloud

/// Live LiDAR point cloud seen from outside the vehicle. The vehicle sits at the center
/// and always points up on the screen. One finger or the mouse rotates, wheel or two fingers zoom.
Rectangle {
    id: root

    property alias receiver: cloudReceiver
    property bool showStatus: true

    readonly property real _minDistance: 150
    readonly property real _maxDistance: 4000

    property real _yaw: 0
    property real _pitch: -35
    property real _distance: 900

    color: "#10151a"
    clip: true

    PointCloudReceiver {
        id: cloudReceiver
        active: root.visible
    }

    View3D {
        id: view3d
        anchors.fill: parent

        environment: SceneEnvironment {
            clearColor: root.color
            backgroundMode: SceneEnvironment.Color
        }

        Node {
            id: cameraPivot
            eulerRotation.y: root._yaw
            eulerRotation.x: root._pitch

            PerspectiveCamera {
                id: camera
                z: root._distance
                clipNear: 10
                clipFar: 20000
            }
        }

        Model {
            id: cloudModel
            source: "#Cube"
            instancing: PointCloudInstancing {
                id: cloudInstancing
                receiver: cloudReceiver
            }
            materials: DefaultMaterial {
                diffuseColor: "white"
                lighting: DefaultMaterial.NoLighting
            }
        }

        // Vehicle marker: a cone at the center whose tip points in flight direction (-z)
        Model {
            source: "#Cone"
            scale: Qt.vector3d(0.25, 0.5, 0.25)
            eulerRotation.x: -90
            materials: DefaultMaterial {
                diffuseColor: "white"
                lighting: DefaultMaterial.NoLighting
            }
        }
    }

    WheelHandler {
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
        onWheel: (event) => root._zoomBy(event.angleDelta.y > 0 ? 0.9 : 1.1)
    }

    // One finger (or the mouse) rotates, two fingers zoom. A MultiPointTouchArea is used because
    // it is also what the on-screen joystick uses, and it receives single touches on the radio.
    MultiPointTouchArea {
        id: touchArea
        anchors.fill: parent
        minimumTouchPoints: 1
        maximumTouchPoints: 2
        mouseEnabled: true

        property real _lastPinchDistance: 0

        touchPoints: [
            TouchPoint { id: touchA },
            TouchPoint { id: touchB }
        ]

        onUpdated: {
            if (touchA.pressed && touchB.pressed) {
                var distance = Math.hypot(touchA.x - touchB.x, touchA.y - touchB.y)
                if (_lastPinchDistance > 0 && distance > 0) {
                    root._zoomBy(_lastPinchDistance / distance)
                }
                _lastPinchDistance = distance
            } else if (touchA.pressed || touchB.pressed) {
                _lastPinchDistance = 0
                var finger = touchA.pressed ? touchA : touchB
                root._yaw -= (finger.x - finger.previousX) * 0.4
                root._pitch = Math.max(-89, Math.min(-5, root._pitch + (finger.y - finger.previousY) * -0.4))
            }
        }

        onReleased: _lastPinchDistance = 0
        onCanceled: _lastPinchDistance = 0
    }

    function _zoomBy(factor) {
        root._distance = Math.max(_minDistance, Math.min(_maxDistance, root._distance * factor))
    }

    Text {
        anchors.centerIn: parent
        visible: !cloudReceiver.receiving
        color: "#aab4be"
        horizontalAlignment: Text.AlignHCenter
        text: cloudReceiver.listening
              ? qsTr("Keine Punktwolke\nUDP-Port %1").arg(cloudReceiver.port)
              : qsTr("Empfänger nicht aktiv\n%1").arg(cloudReceiver.errorString)
    }

    Text {
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.margins: 6
        visible: root.showStatus && cloudReceiver.receiving
        color: "#aab4be"
        font.pixelSize: 12
        text: qsTr("%1 Punkte · %2 Hz").arg(cloudReceiver.pointCount).arg(cloudReceiver.frameRate)
    }
}
