import QtQuick

// Glass-morphism styled panel container
Rectangle {
    id: root

    // Public properties
    property string title: ""
    property bool showBorder: true
    property real glassOpacity: 0.85

    // Default appearance
    color: Qt.rgba(0.12, 0.12, 0.12, glassOpacity)  // #1e1e1e with alpha
    radius: 10
    border.color: showBorder ? "#30ffffff" : "transparent"
    border.width: showBorder ? 1 : 0

    // Optional title header
    Text {
        id: titleText
        text: root.title
        visible: root.title !== ""
        color: "#ffffff"
        font.pixelSize: 14
        font.bold: true
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.margins: 15
    }

    // Content area offset if title is present
    property real contentTopMargin: title !== "" ? 40 : 15
}
