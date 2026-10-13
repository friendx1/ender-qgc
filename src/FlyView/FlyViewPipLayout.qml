import QtQuick

import QGroundControl
import QGroundControl.Controls

/// Arranges the three big Fly View items: video, map and LiDAR point cloud.
///
/// One item fills the screen, the other two sit in small slots at the bottom (left and right).
/// Clicking a small slot swaps it with the full screen item. Each slot can be resized by dragging
/// its corner icon. Without a video stream the video slot stays empty.
///
/// The items are children of the same parent as this control and are only moved and resized here.
Item {
    id: _root

    property Item   mapItem                     // Has a PipSlotState named pipState
    property Item   videoItem                   // Has a PipSlotState named pipState
    property Item   cloudItem
    property bool   videoAvailable:     false
    property bool   showSlots:          true    // false: hide the small slots (e.g. video full screen mode)
    property real   margin:             ScreenTools.defaultFontPixelWidth * 0.75

    // Names of the items in slot order: [full screen, left, right]
    property var    order:              ["video", "map", "cloud"]
    property real   leftFraction:       0.22    // Slot width as fraction of this control's width
    property real   rightFraction:      0.22

    readonly property real _minFraction:    0.10
    readonly property real _maxFraction:    0.45
    readonly property real _slotAspect:     9 / 16
    readonly property real _maxSlotHeight:  0.6     // Fraction of this control's height

    readonly property var   _slotNames: _effectiveOrder(order, videoAvailable)
    readonly property bool  leftSlotVisible:   showSlots && _slotNames[1] !== ""
    readonly property bool  rightSlotVisible:  showSlots && _slotNames[2] !== ""
    readonly property rect  _leftRect:  _rectFor(1)
    readonly property rect  _rightRect: _rectFor(2)

    // Space taken by the slots, for the tool insets of the other Fly View layers
    readonly property real  leftEdgeBottomInset:    leftSlotVisible ? _leftRect.width + margin : 0
    readonly property real  bottomEdgeLeftInset:    leftSlotVisible ? _leftRect.height + margin : 0
    readonly property real  rightEdgeBottomInset:   rightSlotVisible ? _rightRect.width + margin : 0
    readonly property real  bottomEdgeRightInset:   rightSlotVisible ? _rightRect.height + margin : 0

    readonly property string _stateFull: "full"
    readonly property string _statePip:  "pip"

    // Without video the video entry is empty and the item next to it moves up to full screen
    function _effectiveOrder(order, videoAvailable) {
        var names = order.slice()
        if (!videoAvailable) {
            var videoIndex = names.indexOf("video")
            if (videoIndex === 0) {
                names[0] = names[1]
                names[1] = ""
            } else if (videoIndex > 0) {
                names[videoIndex] = ""
            }
        }
        return names
    }

    function _rectFor(slot) {
        if (slot === 0) {
            return Qt.rect(0, 0, width, height)
        }
        if (slot === 1 || slot === 2) {
            var w = width * (slot === 1 ? leftFraction : rightFraction)
            var h = w * _slotAspect
            var x = slot === 1 ? margin : width - margin - w
            return Qt.rect(x, height - margin - h, w, h)
        }
        return Qt.rect(0, 0, 0, 0)
    }

    function _setFraction(slot, slotWidth) {
        if (width <= 0) {
            return
        }
        var maxByHeight = height * _maxSlotHeight / _slotAspect / width
        var fraction = Math.max(_minFraction, Math.min(_maxFraction, maxByHeight, slotWidth / width))
        if (slot === 1) {
            leftFraction = fraction
        } else {
            rightFraction = fraction
        }
    }

    // Swaps the item in a small slot with the full screen item
    function _swapWithFull(slot) {
        var names = _slotNames.slice()
        if (names[0] === "") {
            return
        }
        // Keep the unavailable video where it is so the other slot does not change
        for (var i = 0; i < names.length; i++) {
            if (names[i] === "") {
                names[i] = "video"
            }
        }
        var tmp = names[0]
        names[0] = names[slot]
        names[slot] = tmp
        order = names
    }

    // Moves and resizes one item to the rectangle of its slot
    component SlotPlacer: Item {
        id: placer

        property Item   target
        property int    slot:               -1
        property rect   slotRect
        property bool   slotsShown:         true
        property QtObject slotState                 // PipSlotState of the target, if it has one
        property real   pipZ:               0

        readonly property bool  _inFull:    slot === 0
        readonly property bool  _active:    target !== null

        Binding { when: placer._active; target: placer.target; property: "x"; value: placer.slotRect.x; restoreMode: Binding.RestoreNone }
        Binding { when: placer._active; target: placer.target; property: "y"; value: placer.slotRect.y; restoreMode: Binding.RestoreNone }
        Binding { when: placer._active; target: placer.target; property: "width"; value: placer.slotRect.width; restoreMode: Binding.RestoreNone }
        Binding { when: placer._active; target: placer.target; property: "height"; value: placer.slotRect.height; restoreMode: Binding.RestoreNone }
        Binding { when: placer._active; target: placer.target; property: "z"; value: placer._inFull ? 0 : placer.pipZ; restoreMode: Binding.RestoreNone }
        Binding { when: placer._active; target: placer.target; property: "clip"; value: !placer._inFull; restoreMode: Binding.RestoreNone }
        // Opacity instead of visible: the map hides itself in 3D mode and must keep doing that
        Binding { when: placer._active; target: placer.target; property: "opacity"; value: (placer.slot >= 0 && (placer._inFull || placer.slotsShown)) ? 1 : 0; restoreMode: Binding.RestoreNone }
        Binding { when: placer.slotState !== null; target: placer.slotState; property: "state"; value: placer._inFull ? "full" : "pip"; restoreMode: Binding.RestoreNone }
    }

    // Click area, frame and resize corner drawn on top of a small slot
    component SlotOverlay: Item {
        id: overlay

        property Item   layout
        property bool   rightSide:  false

        signal swapRequested
        signal widthRequested(real slotWidth)

        readonly property real _iconSize: ScreenTools.defaultFontPixelHeight * 2.5

        MouseArea {
            anchors.fill:       parent
            preventStealing:    true
            onClicked:          overlay.swapRequested()
        }

        Rectangle {
            anchors.fill:   parent
            color:          "transparent"
            border.color:   Qt.rgba(1, 1, 1, 0.6)
            border.width:   1
        }

        // Drag to resize
        Image {
            id:                 resizeIcon
            source:             "/qmlimages/pipResize.svg"
            fillMode:           Image.PreserveAspectFit
            mipmap:             true
            mirror:             overlay.rightSide
            anchors.top:        parent.top
            anchors.left:       overlay.rightSide ? parent.left : undefined
            anchors.right:      overlay.rightSide ? undefined : parent.right
            width:              overlay._iconSize
            height:             overlay._iconSize
            sourceSize.height:  height

            MouseArea {
                anchors.fill:       parent
                preventStealing:    true

                property real startPointerX:    0
                property real startWidth:       0

                onPressed: (mouse) => {
                    startPointerX = mapToItem(overlay.layout, mouse.x, mouse.y).x
                    startWidth = overlay.width
                }

                onPositionChanged: (mouse) => {
                    if (!pressed) {
                        return
                    }
                    var delta = mapToItem(overlay.layout, mouse.x, mouse.y).x - startPointerX
                    overlay.widthRequested(overlay.rightSide ? startWidth - delta : startWidth + delta)
                }
            }
        }
    }

    // While the point cloud sits in a small slot its taps belong to the slot (swap, resize),
    // so the view must not react to them itself
    Binding {
        when:           _root.cloudItem !== null
        target:         _root.cloudItem
        property:       "interactive"
        value:          _root._slotNames.indexOf("cloud") === 0
        restoreMode:    Binding.RestoreNone
    }

    SlotPlacer {
        target:     _root.mapItem
        slot:       _root._slotNames.indexOf("map")
        slotRect:   _root._rectFor(slot)
        slotsShown: _root.showSlots
        slotState:  _root.mapItem ? _root.mapItem.pipState : null
        pipZ:       QGroundControl.zOrderWidgets
    }

    SlotPlacer {
        target:     _root.videoItem
        slot:       _root._slotNames.indexOf("video")
        slotRect:   _root._rectFor(slot)
        slotsShown: _root.showSlots
        slotState:  _root.videoItem ? _root.videoItem.pipState : null
        pipZ:       QGroundControl.zOrderWidgets
    }

    SlotPlacer {
        target:     _root.cloudItem
        slot:       _root._slotNames.indexOf("cloud")
        slotRect:   _root._rectFor(slot)
        slotsShown: _root.showSlots
        pipZ:       QGroundControl.zOrderWidgets
    }

    SlotOverlay {
        layout:     _root
        x:          _root._leftRect.x
        y:          _root._leftRect.y
        width:      _root._leftRect.width
        height:     _root._leftRect.height
        visible:    _root.leftSlotVisible

        onSwapRequested:        _root._swapWithFull(1)
        onWidthRequested: (slotWidth) => _root._setFraction(1, slotWidth)
    }

    SlotOverlay {
        layout:     _root
        rightSide:  true
        x:          _root._rightRect.x
        y:          _root._rightRect.y
        width:      _root._rightRect.width
        height:     _root._rightRect.height
        visible:    _root.rightSlotVisible

        onSwapRequested:        _root._swapWithFull(2)
        onWidthRequested: (slotWidth) => _root._setFraction(2, slotWidth)
    }
}
