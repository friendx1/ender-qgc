import QtQuick
import QtQuick3D

import QGroundControl.PointCloud

/// Live LiDAR point cloud seen from outside the vehicle. Two views:
///
/// Vehicle view (default): the vehicle sits at the center and always points up on the screen.
/// One finger or the mouse rotates, wheel or two fingers zoom.
///
/// Free view (like RViz): the camera orbits a focus point in the world and stays there while the vehicle flies.
/// One finger or the left mouse button rotates, two fingers pan and zoom, middle or right mouse button pans,
/// wheel zooms. The button "Drohne zentrieren" goes back to the vehicle view. The world pose of the vehicle
/// comes from the onboard computer (MapReceiver), the scan is placed with the pose it was taken at.
///
/// Goal mode: freezes the scan, looks straight down and lets the user tap a point. "Senden" sends
/// that point to the onboard computer as a flight goal.
///
/// Obstacle map: the button "Hindernisse" connects to the onboard computer and draws the safety margin
/// that the planner keeps around obstacles (SUPER inflate map) as a second, orange layer.
///
/// Trajectory: the button "Bahn" draws what the planner has planned, as in RViz: the trajectory it follows (green,
/// exp_traj) and the backup trajectory (cyan, backup_traj). Both are lines in the world, like the map.
Rectangle {
    id: root

    property alias receiver: cloudReceiver
    property alias goalSender: sender
    property alias mapReceiver: mapRx
    property bool showStatus: true

    /// False while the view sits in a small slot, where taps belong to the slot (swap, resize)
    property bool interactive: true

    property bool goalMode: false

    /// Free camera in the world instead of the camera that follows the vehicle
    property bool freeView: false

    /// How long scan points stay visible, in milliseconds. They stay where they were seen while the vehicle flies,
    /// which needs its pose from the onboard computer. 0 shows only the latest scan.
    property int scanPersistenceMs: 5000

    /// Obstacle map on or off. Off by default, so nothing is requested from the vehicle until it is wanted.
    property bool showObstacles: false

    /// Planned trajectories on or off. Off by default, so nothing is requested from the vehicle until it is wanted.
    property bool showTrajectory: false

    readonly property real _minDistance: 150
    readonly property real _maxDistance: 4000
    readonly property real _maxFreeDistance: 8000
    readonly property real _panStartTravel: 40      // pixels of two finger travel that switch to the free view
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

    // Free view: orbit around _focus (scene centimeters in the world)
    property vector3d _focus: Qt.vector3d(0, 0, 0)
    property real _freeYaw: 0
    property real _freePitch: -35
    property real _freeDistance: 900
    property bool _focusOnPose: false   // the focus moves to the vehicle as soon as its pose is known
    property bool _freeBeforeGoal: false

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
    readonly property real _buttonWidth: Math.max(140, _buttonHeight * 2.8)

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

    // Connected for the obstacle map, the trajectories, the free view and the persistence of the scan: all need the
    // pose of the vehicle. The map and the trajectories are only requested while they are shown.
    MapReceiver {
        id: mapRx
        active: (root.showObstacles || root.showTrajectory || root.freeView || root.scanPersistenceMs > 0) && root.visible
        wantMap: root.showObstacles
        wantTrajectory: root.showTrajectory
        frozen: root.goalMode
        scanStamp: cloudInstancing.stampMs
    }

    Connections {
        target: mapRx
        function onPoseChanged() {
            if (root._focusOnPose && mapRx.poseValid) {
                root._focus = mapRx.vehiclePosition
                root._focusOnPose = false
            }
        }
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

    // The scene is the world. The map sits in it as it is, the vehicle (with the vehicle camera, the marker and the
    // goal) and the scan are placed at the poses from the onboard computer. Without a pose all of them sit at the
    // origin, which is the plain vehicle view.
    View3D {
        id: view3d
        anchors.fill: parent
        camera: root.goalMode ? topCamera : (root.freeView ? freeCamera : orbitCamera)

        environment: SceneEnvironment {
            clearColor: root.color
            backgroundMode: SceneEnvironment.Color
        }

        // Free view camera
        Node {
            id: freePivot
            position: root._focus
            eulerRotation.y: root._freeYaw
            eulerRotation.x: root._freePitch

            PerspectiveCamera {
                id: freeCamera
                z: root._freeDistance
                clipNear: 10
                clipFar: 60000
            }
        }

        // The vehicle. In the free view it is at its latest pose. In the vehicle view it is at the pose of the scan,
        // so that the scan, which is given in the vehicle frame, stays exactly where it was seen.
        Node {
            id: vehicleNode
            position: root.freeView ? mapRx.vehiclePosition : mapRx.scanPosition
            rotation: root.freeView ? mapRx.vehicleRotation : mapRx.scanRotation

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

        // The scan. While points are kept over time they are already in the world; otherwise the scan is in the vehicle
        // frame and is placed with the pose of the vehicle at the moment of the scan.
        Node {
            position: cloudInstancing.worldMode ? Qt.vector3d(0, 0, 0) : mapRx.scanPosition
            rotation: cloudInstancing.worldMode ? Qt.quaternion(1, 0, 0, 0) : mapRx.scanRotation

            Model {
                id: cloudModel
                source: "#Cube"
                instancing: PointCloudInstancing {
                    id: cloudInstancing
                    receiver: cloudReceiver
                    poseSource: mapRx
                    persistenceMs: root.scanPersistenceMs
                    // Seen from far above a 4 cm cube is smaller than a pixel
                    pointSize: root.goalMode ? Math.max(4, root._topFov * 3) : 4
                }
                materials: DefaultMaterial {
                    diffuseColor: "white"
                    lighting: DefaultMaterial.NoLighting
                }
            }
        }

        // Obstacle map. The points are in the world frame, so the model needs no transform and the buffer
        // is not rebuilt when the vehicle flies.
        Model {
            id: mapModel
            visible: root.showObstacles && mapRx.active && mapRx.poseValid && mapRx.pointCount > 0
            source: "#Cube"
            opacity: 0.6
            instancing: MapInstancing {
                receiver: mapRx
                pointSize: root.goalMode ? Math.max(8, root._topFov * 3) : 10
            }
            materials: DefaultMaterial {
                diffuseColor: "white"
                lighting: DefaultMaterial.NoLighting
            }
        }

        // Planned trajectories, in the world frame like the map
        Model {
            visible: root.showTrajectory && mapRx.active && expInstancing.count > 0
            source: "#Cube"
            instancing: TrajectoryInstancing {
                id: expInstancing
                receiver: mapRx
                kind: 0
                color: "#39ff14"
                pointSize: root.goalMode ? Math.max(6, root._topFov * 2) : 6
            }
            materials: DefaultMaterial {
                diffuseColor: "white"
                lighting: DefaultMaterial.NoLighting
            }
        }

        Model {
            visible: root.showTrajectory && mapRx.active && backupInstancing.count > 0
            source: "#Cube"
            instancing: TrajectoryInstancing {
                id: backupInstancing
                receiver: mapRx
                kind: 1
                color: "#00e5ff"
                pointSize: root.goalMode ? Math.max(5, root._topFov * 1.6) : 5
            }
            materials: DefaultMaterial {
                diffuseColor: "white"
                lighting: DefaultMaterial.NoLighting
            }
        }
    }

    WheelHandler {
        enabled: root.interactive
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
        onWheel: (event) => root._zoomBy(event.angleDelta.y > 0 ? 0.9 : 1.1)
    }

    // One finger (or the left mouse button) rotates, two fingers zoom and, in the free view, pan. In the vehicle
    // view a two finger drag of some length switches to the free view. A MultiPointTouchArea is used because
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
        property real _lastMidX: 0
        property real _lastMidY: 0
        property bool _hasMid: false
        property real _panTravel: 0
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
                _hasMid = false
                _panTravel = 0
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

                var midX = (touchA.x + touchB.x) / 2
                var midY = (touchA.y + touchB.y) / 2
                if (_hasMid && !root.goalMode) {
                    var panX = midX - _lastMidX
                    var panY = midY - _lastMidY
                    if (root.freeView) {
                        root._panBy(panX, panY)
                    } else {
                        _panTravel += Math.abs(panX) + Math.abs(panY)
                        if (_panTravel > root._panStartTravel) {
                            root._enterFreeView()
                        }
                    }
                }
                _lastMidX = midX
                _lastMidY = midY
                _hasMid = true
            } else if (touchA.pressed || touchB.pressed) {
                _lastPinchDistance = 0
                _hasMid = false
                var finger = touchA.pressed ? touchA : touchB
                var dx = finger.x - finger.previousX
                var dy = finger.y - finger.previousY
                _travel += Math.abs(dx) + Math.abs(dy)
                if (!root.goalMode) {
                    root._rotateBy(dx, dy)
                }
            }
        }

        onReleased: (points) => {
            _lastPinchDistance = 0
            _hasMid = false
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
            _hasMid = false
            _multi = false
        }
    }

    // Middle or right mouse button pans, like in RViz. Touch does not come through here.
    DragHandler {
        id: mousePan
        enabled: root.interactive && !root.goalMode
        target: null
        acceptedDevices: PointerDevice.Mouse
        acceptedButtons: Qt.MiddleButton | Qt.RightButton

        property real _lastX: 0
        property real _lastY: 0

        onActiveChanged: {
            if (active) {
                root._enterFreeView()
                _lastX = 0
                _lastY = 0
            }
        }
        onTranslationChanged: {
            if (active) {
                root._panBy(translation.x - _lastX, translation.y - _lastY)
                _lastX = translation.x
                _lastY = translation.y
            }
        }
    }

    function _zoomBy(factor) {
        if (goalMode) {
            root._topFov = Math.max(_minTopFov, Math.min(_maxTopFov, root._topFov * factor))
        } else if (freeView) {
            root._freeDistance = Math.max(_minDistance, Math.min(_maxFreeDistance, root._freeDistance * factor))
        } else {
            root._distance = Math.max(_minDistance, Math.min(_maxDistance, root._distance * factor))
        }
    }

    function _rotateBy(dx, dy) {
        if (freeView) {
            root._freeYaw -= dx * 0.4
            root._freePitch = Math.max(-89, Math.min(89, root._freePitch + dy * -0.4))
        } else {
            root._yaw -= dx * 0.4
            root._pitch = Math.max(-89, Math.min(-5, root._pitch + dy * -0.4))
        }
    }

    // Moves the focus so that the scene follows the fingers (dx, dy in pixels)
    function _panBy(dx, dy) {
        if (!freeView || view3d.height <= 0) {
            return
        }
        var perPixel = 2 * _freeDistance * Math.tan(freeCamera.fieldOfView * Math.PI / 360) / view3d.height
        var right = freePivot.sceneRotation.times(Qt.vector3d(1, 0, 0))
        var up = freePivot.sceneRotation.times(Qt.vector3d(0, 1, 0))
        _focus = _focus.minus(right.times(dx * perPixel)).plus(up.times(dy * perPixel))
    }

    // Switches to the free view and keeps the picture: same distance and tilt, same direction of view, looking
    // at the vehicle. The camera of the vehicle view turns with the vehicle, the free camera is fixed to the world.
    function _enterFreeView() {
        if (goalMode || freeView) {
            return
        }
        var forward = vehicleNode.sceneRotation.times(Qt.vector3d(0, 0, -1))
        _freeYaw = _yaw + Math.atan2(-forward.x, -forward.z) * 180 / Math.PI
        _freePitch = _pitch
        _freeDistance = _distance
        _focus = vehicleNode.scenePosition
        _focusOnPose = !mapRx.poseValid
        freeView = true
    }

    function _enterVehicleView() {
        freeView = false
        _focusOnPose = false
    }

    function _toggleView() {
        if (freeView) {
            _enterVehicleView()
        } else {
            _enterFreeView()
        }
    }

    function _enterGoalMode() {
        if (!cloudReceiver.receiving) {
            return
        }
        _hasGoal = false
        // The goal is picked from above in the vehicle frame
        _freeBeforeGoal = freeView
        freeView = false
        cloudInstancing.frozen = true
        goalMode = true
    }

    function _leaveGoalMode() {
        goalMode = false
        _hasGoal = false
        cloudInstancing.frozen = false
        if (_freeBeforeGoal) {
            _freeBeforeGoal = false
            freeView = true
        }
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
        } else if (_hit(obstacleButton, x, y)) {
            showObstacles = !showObstacles
        } else if (_hit(trajectoryButton, x, y)) {
            showTrajectory = !showTrajectory
        } else if (_hit(viewButton, x, y)) {
            _toggleView()
        } else if (_hit(cancelButton, x, y)) {
            _leaveGoalMode()
        } else if (_hit(sendButton, x, y)) {
            _sendGoal()
        } else if (goalMode) {
            _pickGoal(x, y)
        }
    }

    // Converts a tap on the screen into a point on the plane of the vehicle (its own frame), using the real camera
    function _pickGoal(x, y) {
        var origin = view3d.mapFrom3DScene(vehicleNode.mapPositionToScene(Qt.vector3d(0, 0, 0)))
        var alongX = view3d.mapFrom3DScene(vehicleNode.mapPositionToScene(Qt.vector3d(100, 0, 0)))
        var alongZ = view3d.mapFrom3DScene(vehicleNode.mapPositionToScene(Qt.vector3d(0, 0, 100)))
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
        visible: root._controlsVisible

        GoalButton {
            id: enterButton
            width: root._buttonWidth
            height: root._buttonHeight
            visible: !root.goalMode
            dimmed: !cloudReceiver.receiving
            label: qsTr("Ziel wählen")
        }
        GoalButton {
            id: obstacleButton
            width: root._buttonWidth
            height: root._buttonHeight
            visible: !root.goalMode
            primary: root.showObstacles
            label: root.showObstacles ? qsTr("Hindernisse an") : qsTr("Hindernisse aus")
        }
        GoalButton {
            id: trajectoryButton
            width: root._buttonWidth
            height: root._buttonHeight
            visible: !root.goalMode
            primary: root.showTrajectory
            label: root.showTrajectory ? qsTr("Bahn an") : qsTr("Bahn aus")
        }
        GoalButton {
            id: viewButton
            width: root._buttonWidth
            height: root._buttonHeight
            visible: !root.goalMode
            primary: root.freeView
            label: root.freeView ? qsTr("Drohne zentrieren") : qsTr("Freie Kamera")
        }
        GoalButton {
            id: sendButton
            width: root._buttonWidth
            height: root._buttonHeight
            visible: root.goalMode
            primary: true
            dimmed: !root._hasGoal || root._goalTooFar || sender.pending
            label: sender.pending ? qsTr("Sende…") : qsTr("Senden")
        }

        GoalButton {
            id: cancelButton
            width: root._buttonWidth
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
        anchors.bottomMargin: 42
        visible: root.showStatus && root.freeView && !mapRx.poseValid
        color: "#e8a34a"
        font.pixelSize: 12
        text: mapRx.connected ? qsTr("Freie Kamera: warte auf die Pose der Drohne")
                              : qsTr("Freie Kamera: keine Verbindung zur Drohne")
    }

    Text {
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.margins: 6
        anchors.bottomMargin: 60
        visible: root.showStatus && root.showTrajectory
        color: "#7ee06a"
        font.pixelSize: 12
        text: {
            if (!mapRx.connected) {
                return qsTr("Bahn: keine Verbindung")
            }
            var points = expInstancing.count + backupInstancing.count
            return points > 0 ? qsTr("Bahn: geplant") : qsTr("Bahn: keine geplant")
        }
    }

    Text {
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.margins: 6
        anchors.bottomMargin: 24
        visible: root.showStatus && root.showObstacles
        color: "#e8a34a"
        font.pixelSize: 12
        text: {
            if (!mapRx.connected) {
                return mapRx.errorString.length > 0 ? qsTr("Hindernisse: keine Verbindung (%1)").arg(mapRx.errorString)
                                                     : qsTr("Hindernisse: verbinde…")
            }
            if (!mapRx.receiving) {
                return qsTr("Hindernisse: verbunden, warte auf Karte")
            }
            return qsTr("Hindernisse: %1 Punkte").arg(mapRx.pointCount)
        }
    }

    Text {
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.margins: 6
        visible: root.showStatus && cloudReceiver.receiving
        color: "#aab4be"
        font.pixelSize: 12
        text: root.goalMode ? qsTr("Scan eingefroren") : qsTr("%1 Punkte · %2 Hz").arg(cloudInstancing.count).arg(cloudReceiver.frameRate)
    }
}
