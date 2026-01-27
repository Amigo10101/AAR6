import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Settings Tab - Connection configuration
Rectangle {
    id: root
    color: "#2a2a2a"

    // Required properties from parent
    required property var robotManager
    required property bool isSimulationMode
    required property string selectedDevice
    required property int selectedBaudrate
    required property var availableBaudrates

    // Signals to update parent state
    signal deviceChanged(string device)
    signal baudrateChanged(int baudrate)
    signal saveConfiguration

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 40
        spacing: 25

        Text {
            text: "Connection Settings"
            color: "white"
            font.pixelSize: 24
            font.bold: true
            font.family: "Segoe UI"
            Layout.alignment: Qt.AlignHCenter
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredWidth: 500
            Layout.alignment: Qt.AlignHCenter
            height: 1
            color: "#40ffffff"
        }

        // Device Selection
        ColumnLayout {
            Layout.fillWidth: true
            Layout.preferredWidth: 500
            Layout.alignment: Qt.AlignHCenter
            spacing: 10

            Text {
                text: "Serial Device"
                color: "#cccccc"
                font.pixelSize: 16
                font.bold: true
            }

            Rectangle {
                Layout.fillWidth: true
                height: 45
                radius: 25
                color: "#1e1e1e"
                border.color: "#30ffffff"
                border.width: 1

                Row {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 10

                    Text {
                        text: root.selectedDevice
                        color: "#ffffff"
                        font.pixelSize: 14
                        font.family: "Monospace"
                        anchors.verticalCenter: parent.verticalCenter
                    }

                    Item {
                        Layout.fillWidth: true
                        width: 1
                    }

                    Text {
                        text: "▼"
                        color: "#888888"
                        font.pixelSize: 12
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }

                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        var ports = root.robotManager.getAvailablePorts();
                        if (ports.length > 0) {
                            root.deviceChanged(ports[0]);
                            console.log("Selected device:", ports[0]);
                        }
                    }
                }
            }

            Text {
                text: "Available devices will be auto-detected"
                color: "#888888"
                font.pixelSize: 11
                font.italic: true
            }
        }

        // Baudrate Selection
        ColumnLayout {
            Layout.fillWidth: true
            Layout.preferredWidth: 500
            Layout.alignment: Qt.AlignHCenter
            spacing: 10

            Text {
                text: "Baud Rate"
                color: "#cccccc"
                font.pixelSize: 16
                font.bold: true
            }

            Rectangle {
                Layout.fillWidth: true
                height: 45
                radius: 25
                color: "#1e1e1e"
                border.color: "#30ffffff"
                border.width: 1

                Row {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 10

                    Text {
                        text: root.selectedBaudrate.toString()
                        color: "#ffffff"
                        font.pixelSize: 14
                        font.family: "Monospace"
                        anchors.verticalCenter: parent.verticalCenter
                    }

                    Item {
                        Layout.fillWidth: true
                        width: 1
                    }

                    Text {
                        text: "▼"
                        color: "#888888"
                        font.pixelSize: 12
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }

                MouseArea {
                    id: baudrateSelector
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    property int currentIndex: 2  // Default to 1500000
                    onClicked: {
                        currentIndex = (currentIndex + 1) % root.availableBaudrates.length;
                        root.baudrateChanged(root.availableBaudrates[currentIndex]);
                        console.log("Selected baudrate:", root.availableBaudrates[currentIndex]);
                    }
                }
            }

            Text {
                text: "Common: 9600, 115200, 1500000, 2812000"
                color: "#888888"
                font.pixelSize: 11
                font.italic: true
            }
        }

        // Save Button
        Rectangle {
            Layout.preferredWidth: 200
            Layout.preferredHeight: 45
            Layout.alignment: Qt.AlignHCenter
            Layout.topMargin: 20
            radius: 22
            color: saveMouseArea.containsMouse ? "#66BB6A" : "#4CAF50"

            Behavior on color {
                ColorAnimation {
                    duration: 100
                }
            }

            Text {
                text: "Save Configuration"
                anchors.centerIn: parent
                color: "white"
                font.pixelSize: 14
                font.bold: true
            }

            MouseArea {
                id: saveMouseArea
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    var config = {
                        "device": root.selectedDevice,
                        "baudrate": root.selectedBaudrate
                    };
                    var configStr = JSON.stringify(config, null, 2);
                    console.log("Saving configuration:", configStr);

                    if (!root.isSimulationMode) {
                        root.robotManager.connectRobot(root.selectedDevice, root.selectedBaudrate);
                    }
                    root.saveConfiguration();
                }
            }
        }

        Text {
            text: "Configuration will be saved to config.json"
            color: "#888888"
            font.pixelSize: 11
            font.italic: true
            Layout.alignment: Qt.AlignHCenter
        }

        Item {
            Layout.fillHeight: true
        }
    }
}
