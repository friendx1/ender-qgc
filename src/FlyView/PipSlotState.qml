import QtQuick

// Tells a Fly View item whether it currently fills the screen ("full") or sits in a small slot ("pip").
// Same property names as QmlControls/PipState.qml so existing checks keep working, but without
// reparenting: FlyViewPipLayout only moves and resizes the items.
QtObject {
    readonly property string initState:     "init"
    readonly property string pipState:      "pip"
    readonly property string fullState:     "full"
    readonly property string windowState:   "window"

    property string state:  initState
    property bool   isDark: true       // true: use dark overlay visuals
}
