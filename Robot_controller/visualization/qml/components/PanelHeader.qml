import QtQuick

// Section header with optional divider line
Rectangle {
    id: root

    property string title: "Section"
    property bool showDivider: true
    property color textColor: "#ffffff"
    property real fontSize: 14

    height: 36
    color: "transparent"

    Text {
        text: root.title
        color: root.textColor
        font.pixelSize: root.fontSize
        font.bold: true
        anchors.left: parent.left
        anchors.verticalCenter: parent.verticalCenter
    }

    Rectangle {
        visible: root.showDivider
        height: 1
        color: "#30ffffff"
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
    }
}
