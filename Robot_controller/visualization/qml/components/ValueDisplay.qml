import QtQuick

// Formatted value display with label
Item {
    id: root

    property string label: "Value"
    property string value: "0.00"
    property string unit: ""
    property color labelColor: "#888888"
    property color valueColor: "#ffffff"
    property real labelSize: 10
    property real valueSize: 12
    property bool monospace: true

    implicitWidth: row.implicitWidth
    implicitHeight: row.implicitHeight

    Row {
        id: row
        spacing: 4

        Text {
            text: root.label + ":"
            color: root.labelColor
            font.pixelSize: root.labelSize
            anchors.baseline: valueText.baseline
        }

        Text {
            id: valueText
            text: root.value + (root.unit !== "" ? " " + root.unit : "")
            color: root.valueColor
            font.pixelSize: root.valueSize
            font.family: root.monospace ? "Monospace" : undefined
            font.bold: true
        }
    }
}
