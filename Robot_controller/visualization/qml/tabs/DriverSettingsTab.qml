import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Driver Settings Tab - Joint configuration and tuning
Item {
    id: root

    // Required properties from parent
    required property var currentLinks
    required property var robotManager
    required property bool isSimulationMode
    required property var simulationJointPositions
    required property var jointAngles

    // Signals to update parent state
    signal jointAnglesChanged(var angles)
    signal simulationPositionsChanged(var positions)

    // Sidebar with joint selection
    Rectangle {
        id: driverSidebar
        width: 280
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        color: "#1e1e1e"

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 15
            spacing: 10

            Text {
                text: "Joint Configuration"
                color: "white"
                font.pixelSize: 16
                font.bold: true
                Layout.alignment: Qt.AlignHCenter
            }

            ListView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.currentLinks
                spacing: 8

                delegate: Item {
                    width: ListView.view.width
                    height: modelData.hasJoint ? 65 : 0
                    visible: modelData.hasJoint && modelData.jointName !== "right_finger_joint"

                    Rectangle {
                        anchors.fill: parent
                        color: "#2a2a2a"
                        radius: 5
                        border.color: "#30ffffff"
                        border.width: 1

                        Column {
                            anchors.fill: parent
                            anchors.margins: 10
                            spacing: 5

                            Text {
                                text: modelData.jointName
                                color: "#cccccc"
                                font.pixelSize: 13
                                font.bold: true
                            }

                            Row {
                                spacing: 5
                                Text {
                                    text: "Type:"
                                    color: "#888888"
                                    font.pixelSize: 10
                                }
                                Text {
                                    text: modelData.jointType === 3 ? "Prismatic" : "Revolute"
                                    color: "#4CAF50"
                                    font.pixelSize: 10
                                    font.bold: true
                                }
                            }

                            Row {
                                spacing: 5
                                Text {
                                    text: "Range:"
                                    color: "#888888"
                                    font.pixelSize: 10
                                }
                                Text {
                                    text: modelData.jointLowerLimit.toFixed(2) + " to " + modelData.jointUpperLimit.toFixed(2)
                                    color: "#888888"
                                    font.pixelSize: 10
                                    font.family: "Monospace"
                                }
                            }
                        }

                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                console.log("Selected joint:", modelData.jointName);
                            }
                        }
                    }
                }
            }

            // Reset All button
            Rectangle {
                Layout.fillWidth: true
                height: 45
                radius: 25
                color: resetMouseArea.containsMouse ? "#FF7043" : "#FF5722"

                Behavior on color {
                    ColorAnimation {
                        duration: 100
                    }
                }

                Text {
                    text: "Reset All Joints"
                    anchors.centerIn: parent
                    color: "white"
                    font.pixelSize: 14
                    font.bold: true
                }

                MouseArea {
                    id: resetMouseArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        console.log("Reset all joints to default position");

                        if (root.isSimulationMode) {
                            root.simulationPositionsChanged([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

                            var angles = {};
                            for (var i = 0; i < root.currentLinks.length; i++) {
                                if (root.currentLinks[i].hasJoint) {
                                    angles[root.currentLinks[i].jointName] = 0.0;
                                }
                            }
                            root.jointAnglesChanged(angles);
                        } else {
                            for (var i = 0; i < 7; i++) {
                                root.robotManager.sendJointCommand(i, 0.0, 0.0);
                            }

                            var angles = {};
                            for (var i = 0; i < root.currentLinks.length; i++) {
                                if (root.currentLinks[i].hasJoint) {
                                    angles[root.currentLinks[i].jointName] = 0.0;
                                }
                            }
                            root.jointAnglesChanged(angles);
                        }
                    }
                }
            }
        }
    }

    // Config area
    Rectangle {
        anchors.left: driverSidebar.right
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        color: "#121212"

        ColumnLayout {
            anchors.centerIn: parent
            spacing: 20

            Text {
                text: "Driver Configuration"
                color: "white"
                font.pixelSize: 24
                font.bold: true
                font.family: "Segoe UI"
                Layout.alignment: Qt.AlignHCenter
            }

            Rectangle {
                width: 300
                height: 1
                color: "#40ffffff"
                Layout.alignment: Qt.AlignHCenter
            }

            Text {
                text: "Select a joint to configure"
                color: "#888888"
                font.pixelSize: 16
                font.family: "Segoe UI"
                Layout.alignment: Qt.AlignHCenter
            }
        }
    }
}
