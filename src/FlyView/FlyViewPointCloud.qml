import QtQuick

import QGroundControl
import QGroundControl.Controls
import QGroundControl.PointCloud

/// Floating window in the Fly View showing what the vehicle's LiDAR currently sees.
/// A small button hides and shows it.
Item {
    id: _root

    property bool expanded: QGroundControl.loadBoolGlobalSetting(_expandedSettingsKey, true)

    readonly property string _expandedSettingsKey: "IsPointCloudVisible"
    readonly property real _buttonSize: ScreenTools.defaultFontPixelHeight * 2

    width: expanded ? parent.width * 0.22 : _buttonSize * 2
    height: expanded ? width * 0.75 : _buttonSize

    onExpandedChanged: QGroundControl.saveBoolGlobalSetting(_expandedSettingsKey, expanded)

    PointCloudView {
        anchors.fill: parent
        visible: _root.expanded
        radius: ScreenTools.defaultFontPixelHeight / 3
    }

    Rectangle {
        id: _toggleButton
        width: _root.expanded ? _root._buttonSize : _root.width
        height: _root._buttonSize
        anchors.top: parent.top
        anchors.right: parent.right
        radius: ScreenTools.defaultFontPixelHeight / 3
        color: Qt.rgba(0, 0, 0, 0.6)

        QGCLabel {
            anchors.centerIn: parent
            text: _root.expanded ? "✕" : "LiDAR"
            font.pointSize: _root.expanded ? ScreenTools.defaultFontPointSize : ScreenTools.smallFontPointSize
        }

        MouseArea {
            anchors.fill: parent
            onClicked: _root.expanded = !_root.expanded
        }
    }
}
