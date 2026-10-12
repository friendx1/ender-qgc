import QtQuick
import QtQuick3D

import QGroundControl.PointCloud

/// Live LiDAR point cloud seen from outside the vehicle. The vehicle sits at the center
/// and always points up on the screen. Drag to rotate, wheel or pinch to zoom.
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

    DragHandler {
        target: null
        onCentroidChanged: {
            if (active) {
                root._yaw -= (centroid.position.x - centroid.lastPosition.x) * 0.4
                root._pitch = Math.max(-89, Math.min(-5, root._pitch + (centroid.position.y - centroid.lastPosition.y) * -0.4))
            }
        }
    }

    WheelHandler {
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
        onWheel: (event) => root._zoomBy(event.angleDelta.y > 0 ? 0.9 : 1.1)
    }

    PinchHandler {
        target: null
        property real _lastScale: 1
        onActiveChanged: _lastScale = 1
        onScaleChanged: {
            if (active && scale > 0) {
                root._zoomBy(_lastScale / scale)
                _lastScale = scale
            }
        }
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
