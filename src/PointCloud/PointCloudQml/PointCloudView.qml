import QtQuick
import QtQuick3D

import QGroundControl.PointCloud

/// Live LiDAR point cloud seen from outside the vehicle. The vehicle sits at the center
/// and always points up on the screen. One finger or the mouse rotates, wheel or two fingers zoom.
///
/// Goal mode: freezes the scan, looks straight down and lets the user tap a point. "Senden" sends
/// that point to the onboard computer as a flight goal.
Rectangle {
    id: root

    property alias receiver: cloudReceiver
    property alias goalSender: sender
    property bool showStatus: true

    /// False while the view sits in a small slot, where taps belong to the slot (swap, resize)
    property bool interactive: true

    property bool goalMode: false

    readonly property real _minDistance: 150
    readonly property real _maxDistance: 4000
    readonly property real _maxGoalDistance: 40     // meters, the onboard computer rejects farther goals
    readonly property real _topCameraHeight: 30000  // centimeters, far away so the view is almost orthographic
    readonly property real _minTopFov: 1.5
    readonly property real _maxTopFov: 14
    readonly property real _tapMaxTravel: 14
    readonly property int _tapMaxDurationMs: 700

    property real _yaw: 0
    property real _pitch: -35
    property real _distance: 900
    property real _topFov: 4.6

    // Picked goal in scene centimeters (x right, z backwards), see PointCloudInstancing.toScenePosition
    property bool _hasGoal: false
    property real _goalSceneX: 0
    property real _goalSceneZ: 0
    readonly property real _goalForward: -_goalSceneZ / 100  // meters, vehicle body frame
    readonly property real _goalLeft: -_goalSceneX / 100
    readonly property real _goalDistance: Math.hypot(_goalForward, _goalLeft)
    readonly property bool _goalTooFar: _goalDistance > _maxGoalDistance

    readonly property bool _roomy: width >= 360 && height >= 240
    readonly property bool _controlsVisible: interactive && _roomy
    readonly property real _buttonHeight: Math.max(44, Math.min(60, height * 0.12))

    color: "#10151a"
    clip: true

    onInteractiveChanged: {
        if (!interactive && goalMode) {
            _leaveGoalMode()
        }
    }

    PointCloudReceiver {
        id: cloudReceiver
        active: root.visible
    }

    GoalSender {
        id: sender

        onGoalResult: (accepted, text) => {
            if (accepted) {
                root._leaveGoalMode()
            }
            resultToast.restart()
        }
    }

    View3D {
        id: view3d
        anchors.fill: parent
        camera: root.goalMode ? topCamera : orbitCamera

        environment: SceneEnvironment {
            clearColor: root.color
            backgroundMode: SceneEnvironment.Color
        }

        Node {
            id: cameraPivot
            eulerRotation.y: root._yaw
            eulerRotation.x: root._pitch

            PerspectiveCamera {
                id: orbitCamera
                z: root._distance
                clipNear: 10
                clipFar: 20000
            }
        }

        // Goal mode: straight down, vehicle front at the top of the screen
        PerspectiveCamera {
            id: topCamera
            position: Qt.vector3d(0, root._topCameraHeight, 0)
            eulerRotation.x: -90
            fieldOfView: root._topFov
            clipNear: 100
            clipFar: root._topCameraHeight * 2
        }

        Model {
            id: cloudModel
            source: "#Cube"
            instancing: PointCloudInstancing {
                id: cloudInstancing
                receiver: cloudReceiver
                // Seen from far above a 4 cm cube is smaller than a pixel
                pointSize: root.goalMode ? Math.max(4, root._topFov * 3) : 4
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

        // Picked goal
        Model {
            visible: root._hasGoal
            source: "#Sphere"
            position: Qt.vector3d(root._goalSceneX, 0, root._goalSceneZ)
            scale: Qt.vector3d(0.6, 0.6, 0.6)
            materials: DefaultMaterial {
                diffuseColor: root._goalTooFar ? "#ff4040" : "#ff2bd6"
                lighting: DefaultMaterial.NoLighting
            }
        }
    }

    WheelHandler {
        enabled: root.interactive
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
        onWheel: (event) => root._zoomBy(event.angleDelta.y > 0 ? 0.9 : 1.1)
    }

    // One finger (or the mouse) rotates, two fingers zoom. A MultiPointTouchArea is used because
    // it is also what the on-screen joystick uses, and it receives single touches on the radio.
    // A short touch without movement is a tap: it presses a button or picks the goal.
    MultiPointTouchArea {
        id: touchArea
        anchors.fill: parent
        enabled: root.interactive
        minimumTouchPoints: 1
        maximumTouchPoints: 2
        mouseEnabled: true

        property real _lastPinchDistance: 0
        property real _travel: 0
        property bool _multi: false
        property double _pressTime: 0

        touchPoints: [
            TouchPoint { id: touchA },
            TouchPoint { id: touchB }
        ]

        onPressed: (points) => {
            if (touchA.pressed && touchB.pressed) {
                _multi = true
            } else {
                _multi = false
                _travel = 0
                _pressTime = Date.now()
            }
        }

        onUpdated: (points) => {
            if (touchA.pressed && touchB.pressed) {
                var distance = Math.hypot(touchA.x - touchB.x, touchA.y - touchB.y)
                if (_lastPinchDistance > 0 && distance > 0) {
                    root._zoomBy(_lastPinchDistance / distance)
                }
                _lastPinchDistance = distance
            } else if (touchA.pressed || touchB.pressed) {
                _lastPinchDistance = 0
                var finger = touchA.pressed ? touchA : touchB
                var dx = finger.x - finger.previousX
                var dy = finger.y - finger.previousY
                _travel += Math.abs(dx) + Math.abs(dy)
                if (!root.goalMode) {
                    root._yaw -= dx * 0.4
                    root._pitch = Math.max(-89, Math.min(-5, root._pitch + dy * -0.4))
                }
            }
        }

        onReleased: (points) => {
            _lastPinchDistance = 0
            if (!touchA.pressed && !touchB.pressed) {
                var isTap = !_multi && _travel < root._tapMaxTravel && (Date.now() - _pressTime) < root._tapMaxDurationMs
                _multi = false
                if (isTap && points.length > 0) {
                    root._handleTap(points[0].x, points[0].y)
                }
            }
        }

        onCanceled: (points) => {
            _lastPinchDistance = 0
            _multi = false
        }
    }

    function _zoomBy(factor) {
        if (goalMode) {
            root._topFov = Math.max(_minTopFov, Math.min(_maxTopFov, root._topFov * factor))
        } else {
            root._distance = Math.max(_minDistance, Math.min(_maxDistance, root._distance * factor))
        }
    }

    function _enterGoalMode() {
        if (!cloudReceiver.receiving) {
            return
        }
        _hasGoal = false
        cloudInstancing.frozen = true
        goalMode = true
    }

    function _leaveGoalMode() {
        goalMode = false
        _hasGoal = false
        cloudInstancing.frozen = false
    }

    function _sendGoal() {
        if (!_hasGoal || _goalTooFar || sender.pending) {
            return
        }
        sender.sendGoal(_goalForward, _goalLeft, cloudInstancing.stampMs)
    }

    function _hit(item, x, y) {
        if (!item.visible) {
            return false
        }
        var p = item.mapFromItem(touchArea, x, y)
        return p.x >= 0 && p.y >= 0 && p.x <= item.width && p.y <= item.height
    }

    // The buttons are plain rectangles; taps are routed here because the touch area sits below them
    function _handleTap(x, y) {
        if (_hit(enterButton, x, y)) {
            _enterGoalMode()
        } else if (_hit(cancelButton, x, y)) {
            _leaveGoalMode()
        } else if (_hit(sendButton, x, y)) {
            _sendGoal()
        } else if (goalMode) {
            _pickGoal(x, y)
        }
    }

    // Converts a tap on the screen into a point on the plane of the vehicle, using the real camera
    function _pickGoal(x, y) {
        var origin = view3d.mapFrom3DScene(Qt.vector3d(0, 0, 0))
        var alongX = view3d.mapFrom3DScene(Qt.vector3d(100, 0, 0))
        var alongZ = view3d.mapFrom3DScene(Qt.vector3d(0, 0, 100))
        var pixelsPerCmX = (alongX.x - origin.x) / 100
        var pixelsPerCmZ = (alongZ.y - origin.y) / 100
        if (Math.abs(pixelsPerCmX) < 1e-6 || Math.abs(pixelsPerCmZ) < 1e-6) {
            return
        }
        _goalSceneX = (x - origin.x) / pixelsPerCmX
        _goalSceneZ = (y - origin.y) / pixelsPerCmZ
        _hasGoal = true
    }

    component GoalButton: Rectangle {
        property string label
        property bool primary: false
        property bool dimmed: false

        // Inline components cannot see the ids of this file, so the height is set where the button is used
        width: Math.max(height * 2.4, labelText.implicitWidth + 32)
        radius: 6
        color: primary ? "#c01fa8" : "#2b3640"
        opacity: dimmed ? 0.45 : 1
        border.color: "#aab4be"
        border.width: 1

        Text {
            id: labelText
            anchors.centerIn: parent
            text: parent.label
            color: "white"
            font.pixelSize: 18
        }
    }

    // At the right edge, centered: the small slots of the Fly View cover the bottom corners
    Column {
        anchors.right: parent.right
        anchors.verticalCenter: parent.verticalCenter
        anchors.rightMargin: 8
        spacing: 8
        horizontalItemAlignment: Grid.AlignRight
        visible: root._controlsVisible

        GoalButton {
            id: enterButton
            height: root._buttonHeight
            visible: !root.goalMode
            dimmed: !cloudReceiver.receiving
            label: qsTr("Ziel wählen")
        }
        GoalButton {
            id: sendButton
            height: root._buttonHeight
            visible: root.goalMode
            primary: true
            dimmed: !root._hasGoal || root._goalTooFar || sender.pending
            label: sender.pending ? qsTr("Sende…") : qsTr("Senden")
        }

        GoalButton {
            id: cancelButton
            height: root._buttonHeight
            visible: root.goalMode
            label: qsTr("Abbrechen")
        }
    }

    Text {
        id: goalInfo
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: 8
        visible: root.goalMode && root._controlsVisible
        horizontalAlignment: Text.AlignHCenter
        color: root._goalTooFar ? "#ff6060" : "white"
        font.pixelSize: 18
        style: Text.Outline
        styleColor: "#10151a"
        text: {
            var lines = []
            if (!sender.pending && sender.resultText.length > 0) {
                lines.push(sender.resultText)
            }
            if (sender.pending) {
                lines.push(qsTr("Sende Ziel…"))
            } else if (!root._hasGoal) {
                lines.push(qsTr("Punkt antippen"))
            } else if (root._goalTooFar) {
                lines.push(qsTr("Zu weit weg (höchstens %1 m)").arg(root._maxGoalDistance))
            } else {
                var forward = root._goalForward
                var left = root._goalLeft
                lines.push(qsTr("Ziel: %1 m %2, %3 m %4")
                    .arg(Math.abs(forward).toFixed(1)).arg(forward >= 0 ? qsTr("vor") : qsTr("hinter"))
                    .arg(Math.abs(left).toFixed(1)).arg(left >= 0 ? qsTr("links") : qsTr("rechts")))
            }
            return lines.join("\n")
        }
    }

    // Result of the last goal, shown for a few seconds after leaving goal mode
    Text {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: 8
        visible: resultToast.running && !root.goalMode && root._controlsVisible
        color: "white"
        font.pixelSize: 18
        style: Text.Outline
        styleColor: "#10151a"
        text: sender.resultText
    }

    Timer {
        id: resultToast
        interval: 4000
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
        text: root.goalMode ? qsTr("Scan eingefroren") : qsTr("%1 Punkte · %2 Hz").arg(cloudReceiver.pointCount).arg(cloudReceiver.frameRate)
    }
}
