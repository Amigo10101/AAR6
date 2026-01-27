import QtQuick
import QtQuick.Controls

// Reusable styled button with hover effects and multiple variants
Rectangle {
    id: root

    // Public properties
    property string text: "Button"
    property string variant: "primary"  // primary, secondary, danger, success
    property bool enabled: true
    property alias icon: iconText.text
    property real fontSize: 12

    // Signals
    signal clicked
    signal pressed
    signal released

    // Size defaults
    width: 100
    height: 32
    radius: 6

    // Color schemes based on variant
    property color baseColor: {
        switch (variant) {
        case "primary":
            return "#2196F3";
        case "secondary":
            return "#303030";
        case "danger":
            return "#f44336";
        case "success":
            return "#4CAF50";
        default:
            return "#303030";
        }
    }

    property color hoverColor: {
        switch (variant) {
        case "primary":
            return "#42A5F5";
        case "secondary":
            return "#404040";
        case "danger":
            return "#EF5350";
        case "success":
            return "#66BB6A";
        default:
            return "#404040";
        }
    }

    property color pressColor: {
        switch (variant) {
        case "primary":
            return "#1E88E5";
        case "secondary":
            return "#505050";
        case "danger":
            return "#E53935";
        case "success":
            return "#43A047";
        default:
            return "#505050";
        }
    }

    color: !enabled ? "#1a1a1a" : mouseArea.pressed ? pressColor : mouseArea.containsMouse ? hoverColor : baseColor

    border.color: variant === "secondary" ? "#404040" : "transparent"
    border.width: variant === "secondary" ? 1 : 0

    opacity: enabled ? 1.0 : 0.5

    Behavior on color {
        ColorAnimation {
            duration: 100
        }
    }

    Row {
        anchors.centerIn: parent
        spacing: 6

        Text {
            id: iconText
            text: ""
            color: enabled ? "white" : "#666666"
            font.pixelSize: root.fontSize + 2
            font.bold: true
            visible: text !== ""
            anchors.verticalCenter: parent.verticalCenter
        }

        Text {
            text: root.text
            color: enabled ? "white" : "#666666"
            font.pixelSize: root.fontSize
            font.bold: true
            anchors.verticalCenter: parent.verticalCenter
        }
    }

    MouseArea {
        id: mouseArea
        anchors.fill: parent
        enabled: root.enabled
        hoverEnabled: true
        cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor

        onClicked: root.clicked()
        onPressed: root.pressed()
        onReleased: root.released()
    }
}
