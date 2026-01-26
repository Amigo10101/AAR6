import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Joint Control Panel - Individual joint sliders with +/- buttons
Rectangle {
    id: root
    color: "transparent"

    // robotManager is accessed directly from C++ context (not passed as property)
    // Required properties from parent
    required property var currentLinks
    required property bool isSimulationMode
    required property var simulationJointPositions
    required property var currentJointPositions
    required property var jointAngles
    required property var jointColors
    required property bool useRadians

    // Optional: transform gizmo reference for FK updates
    property var transformGizmo: null

    // Signals to update parent state
    signal simulationPositionsChanged(var positions)
    signal jointAnglesUpdated(var angles)  // Renamed to avoid conflict with property change signal
    signal resetAllJoints
    signal gizmoUpdateRequested

    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        ListView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            model: root.currentLinks
            spacing: 10

            delegate: Item {
                width: ListView.view.width
                height: modelData.hasJoint ? 70 : 0
                visible: modelData.hasJoint && modelData.jointName !== "right_finger_joint"

                // Calculate actual joint index by counting previous joints
                property int jointIndex: {
                    var count = 0;
                    for (var i = 0; i < index; i++) {
                        if (root.currentLinks[i].hasJoint && root.currentLinks[i].jointName !== "right_finger_joint") {
                            count++;
                        }
                    }
                    return count;
                }
                property string jointColor: jointIndex < 7 ? root.jointColors[jointIndex] : "#cccccc"
                property bool buttonsPressed: false

                Rectangle {
                    anchors.fill: parent
                    color: "#1e1e1e"
                    radius: 3

                    Column {
                        anchors.fill: parent
                        anchors.margins: 8
                        spacing: 4

                        Row {
                            width: parent.width
                            spacing: 8

                            // Color indicator
                            Rectangle {
                                width: 12
                                height: 12
                                radius: 6
                                color: jointColor
                                anchors.verticalCenter: parent.verticalCenter
                            }

                            Text {
                                text: modelData.jointName
                                color: jointColor
                                font.pixelSize: 12
                                font.bold: true
                                anchors.verticalCenter: parent.verticalCenter
                            }

                            Text {
                                text: "[J" + jointIndex + "]"
                                color: "#888888"
                                font.pixelSize: 10
                                anchors.verticalCenter: parent.verticalCenter
                            }

                            // Spacer to push everything else to right
                            Item {
                                width: Math.max(0, parent.width - 350)
                                height: 1
                            }

                            // Decrement button (-)
                            Rectangle {
                                width: 24
                                height: 24
                                radius: 4
                                color: "#303030"
                                border.color: jointColor
                                border.width: 1
                                anchors.verticalCenter: parent.verticalCenter

                                Text {
                                    text: "−"
                                    color: jointColor
                                    font.pixelSize: 16
                                    font.bold: true
                                    anchors.centerIn: parent
                                }

                                property real startValue: 0
                                property real currentStep: 0.01
                                property int tickCount: 0

                                Timer {
                                    id: decrementTimer
                                    interval: 100
                                    repeat: true
                                    onTriggered: {
                                        parent.tickCount++;
                                        // Accelerate: 0.01 for first 5 ticks, then gradually increase to 0.1
                                        if (parent.tickCount > 10) {
                                            parent.currentStep = 0.1;
                                        } else if (parent.tickCount > 5) {
                                            parent.currentStep = 0.05;
                                        } else {
                                            parent.currentStep = 0.01;
                                        }

                                        var newVal = slider.value - parent.currentStep;
                                        newVal = Math.max(modelData.jointLowerLimit, Math.min(modelData.jointUpperLimit, newVal));
                                        slider.value = newVal;

                                        if (root.isSimulationMode) {
                                            var simPos = root.simulationJointPositions.slice();
                                            simPos[jointIndex] = newVal;
                                            root.simulationPositionsChanged(simPos);

                                            var angles = Object.assign({}, root.jointAngles);
                                            angles[modelData.jointName] = newVal;
                                            root.jointAnglesUpdated(angles);
                                        }
                                    }
                                }

                                MouseArea {
                                    anchors.fill: parent
                                    cursorShape: Qt.PointingHandCursor
                                    onPressed: {
                                        buttonsPressed = true;
                                        parent.startValue = slider.value;
                                        parent.tickCount = 0;
                                        parent.currentStep = 0.01;
                                        decrementTimer.triggered();
                                        decrementTimer.start();
                                    }
                                    onReleased: {
                                        decrementTimer.stop();
                                        parent.tickCount = 0;

                                        // Capture final value before resetting pressed state
                                        var finalValue = slider.value;

                                        // Send command BEFORE resetting buttonsPressed (Robot mode)
                                        if (!root.isSimulationMode && finalValue !== parent.startValue) {
                                            robotManager.sendJointCommand(jointIndex, finalValue, 0.0);

                                            var angles = Object.assign({}, root.jointAngles);
                                            angles[modelData.jointName] = finalValue;
                                            root.jointAnglesUpdated(angles);
                                        }

                                        // Now reset pressed state to re-enable binding
                                        buttonsPressed = false;
                                    }
                                    onCanceled: {
                                        decrementTimer.stop();
                                        buttonsPressed = false;
                                        parent.tickCount = 0;
                                    }
                                }
                            }

                            // Editable value input
                            Rectangle {
                                width: 50
                                height: 24
                                color: "#2a2a2a"
                                radius: 4
                                border.color: jointColor
                                border.width: 1
                                anchors.verticalCenter: parent.verticalCenter

                                TextInput {
                                    id: valueInput
                                    anchors.fill: parent
                                    anchors.margins: 4
                                    text: slider.value.toFixed(2)
                                    color: jointColor
                                    font.pixelSize: 11
                                    font.family: "Monospace"
                                    font.bold: true
                                    horizontalAlignment: Text.AlignRight
                                    verticalAlignment: Text.AlignVCenter
                                    selectByMouse: true

                                    onEditingFinished: {
                                        var val = parseFloat(text);
                                        if (!isNaN(val)) {
                                            // Clamp to joint limits
                                            val = Math.max(modelData.jointLowerLimit, Math.min(modelData.jointUpperLimit, val));
                                            slider.value = val;
                                            text = val.toFixed(2);

                                            if (root.isSimulationMode) {
                                                // Update simulation position
                                                var simPos = root.simulationJointPositions.slice();
                                                simPos[jointIndex] = val;
                                                root.simulationPositionsChanged(simPos);

                                                // Update 3D model joint angles
                                                var angles = Object.assign({}, root.jointAngles);
                                                angles[modelData.jointName] = val;
                                                root.jointAnglesUpdated(angles);
                                            } else {
                                                // Send command to MCU
                                                robotManager.sendJointCommand(jointIndex, val, 0.0);

                                                // Update 3D model joint angles
                                                var angles = Object.assign({}, root.jointAngles);
                                                angles[modelData.jointName] = val;
                                                root.jointAnglesUpdated(angles);
                                            }
                                        } else {
                                            text = slider.value.toFixed(2);
                                        }
                                    }
                                }
                            }

                            // Increment button (+)
                            Rectangle {
                                width: 24
                                height: 24
                                radius: 4
                                color: "#303030"
                                border.color: jointColor
                                border.width: 1
                                anchors.verticalCenter: parent.verticalCenter

                                Text {
                                    text: "+"
                                    color: jointColor
                                    font.pixelSize: 16
                                    font.bold: true
                                    anchors.centerIn: parent
                                }

                                property real startValue: 0
                                property real currentStep: 0.01
                                property int tickCount: 0

                                Timer {
                                    id: incrementTimer
                                    interval: 100
                                    repeat: true
                                    onTriggered: {
                                        parent.tickCount++;
                                        // Accelerate: 0.01 for first 5 ticks, then gradually increase to 0.1
                                        if (parent.tickCount > 10) {
                                            parent.currentStep = 0.1;
                                        } else if (parent.tickCount > 5) {
                                            parent.currentStep = 0.05;
                                        } else {
                                            parent.currentStep = 0.01;
                                        }

                                        var newVal = slider.value + parent.currentStep;
                                        newVal = Math.max(modelData.jointLowerLimit, Math.min(modelData.jointUpperLimit, newVal));
                                        slider.value = newVal;

                                        if (root.isSimulationMode) {
                                            var simPos = root.simulationJointPositions.slice();
                                            simPos[jointIndex] = newVal;
                                            root.simulationPositionsChanged(simPos);

                                            var angles = Object.assign({}, root.jointAngles);
                                            angles[modelData.jointName] = newVal;
                                            root.jointAnglesUpdated(angles);
                                        }
                                    }
                                }

                                MouseArea {
                                    anchors.fill: parent
                                    cursorShape: Qt.PointingHandCursor
                                    onPressed: {
                                        buttonsPressed = true;
                                        parent.startValue = slider.value;
                                        parent.tickCount = 0;
                                        parent.currentStep = 0.01;
                                        incrementTimer.triggered();
                                        incrementTimer.start();
                                    }
                                    onReleased: {
                                        incrementTimer.stop();
                                        parent.tickCount = 0;

                                        // Capture final value before resetting pressed state
                                        var finalValue = slider.value;

                                        // Send command BEFORE resetting buttonsPressed (Robot mode)
                                        if (!root.isSimulationMode && finalValue !== parent.startValue) {
                                            robotManager.sendJointCommand(jointIndex, finalValue, 0.0);

                                            var angles = Object.assign({}, root.jointAngles);
                                            angles[modelData.jointName] = finalValue;
                                            root.jointAnglesUpdated(angles);
                                        }

                                        // Now reset pressed state to re-enable binding
                                        buttonsPressed = false;
                                    }
                                    onCanceled: {
                                        incrementTimer.stop();
                                        buttonsPressed = false;
                                        parent.tickCount = 0;
                                    }
                                }
                            }

                            Text {
                                text: root.useRadians ? "rad" : "°"
                                color: "#888888"
                                font.pixelSize: 10
                                anchors.verticalCenter: parent.verticalCenter
                            }

                            // Reset button - at far right
                            Rectangle {
                                width: 24
                                height: 24
                                radius: 12
                                color: "#303030"
                                border.color: "#606060"
                                border.width: 1
                                anchors.verticalCenter: parent.verticalCenter

                                Text {
                                    text: "↻"
                                    color: "#cccccc"
                                    font.pixelSize: 16
                                    anchors.centerIn: parent
                                }

                                MouseArea {
                                    anchors.fill: parent
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: {
                                        var resetVal = 0.0;
                                        slider.value = resetVal;

                                        if (root.isSimulationMode) {
                                            var simPos = root.simulationJointPositions.slice();
                                            simPos[jointIndex] = resetVal;
                                            root.simulationPositionsChanged(simPos);

                                            var angles = Object.assign({}, root.jointAngles);
                                            angles[modelData.jointName] = resetVal;
                                            root.jointAnglesUpdated(angles);
                                        } else {
                                            robotManager.sendJointCommand(jointIndex, resetVal, 0.0);

                                            var angles = Object.assign({}, root.jointAngles);
                                            angles[modelData.jointName] = resetVal;
                                            root.jointAnglesUpdated(angles);
                                        }
                                    }
                                }
                            }
                        }

                        Slider {
                            id: slider
                            width: parent.width
                            from: modelData.jointLowerLimit
                            to: modelData.jointUpperLimit
                            // Decoupled value binding - allows smooth dragging
                            Binding on value {
                                when: root.isSimulationMode
                                value: (jointIndex < root.simulationJointPositions.length) ? root.simulationJointPositions[jointIndex] : 0.0
                            }

                            // Only bind to MCU data when NOT dragging and buttons NOT pressed
                            Binding on value {
                                when: !root.isSimulationMode && !slider.pressed && !buttonsPressed
                                value: (jointIndex < root.currentJointPositions.length) ? root.currentJointPositions[jointIndex] : 0.0
                            }

                            onMoved: {
                                if (root.isSimulationMode) {
                                    // Update simulation position
                                    var simPos = root.simulationJointPositions.slice();
                                    simPos[jointIndex] = value;
                                    root.simulationPositionsChanged(simPos);

                                    // Update 3D model joint angles
                                    var angles = Object.assign({}, root.jointAngles);
                                    angles[modelData.jointName] = value;
                                    root.jointAnglesUpdated(angles);

                                    // Update gizmo to follow end-effector (FK)
                                    root.gizmoUpdateRequested();
                                } else {
                                    // ROBOT MODE:
                                    // Do nothing visually to the 3D model while dragging.
                                    // The slider value changes (local target), but 3D model
                                    // should only reflect actual robot state from UART.
                                }
                            }

                            onPressedChanged: {
                                if (!pressed && !root.isSimulationMode) {
                                    // Released in Robot Mode -> Send Command
                                    console.log("Slider released, sending command for J" + jointIndex + ": " + value);
                                    robotManager.sendJointCommand(jointIndex, value, 0.0);
                                }
                            }

                            background: Rectangle {
                                x: slider.leftPadding
                                y: slider.topPadding + slider.availableHeight / 2 - height / 2
                                implicitWidth: 200
                                implicitHeight: 4
                                width: slider.availableWidth
                                height: implicitHeight
                                radius: 2
                                color: "#404040"

                                Rectangle {
                                    width: slider.visualPosition * parent.width
                                    height: parent.height
                                    color: jointColor
                                    radius: 2
                                }
                            }

                            handle: Rectangle {
                                x: slider.leftPadding + slider.visualPosition * (slider.availableWidth - width)
                                y: slider.topPadding + slider.availableHeight / 2 - height / 2
                                implicitWidth: 16
                                implicitHeight: 16
                                radius: 25
                                color: slider.pressed ? Qt.lighter(jointColor, 1.2) : jointColor
                                border.color: "#ffffff"
                                border.width: 2
                            }
                        }
                    }
                }
            }
        }

        // Reset All Button
        Item {
            Layout.fillWidth: true
            Layout.preferredHeight: 45
            Layout.topMargin: 5

            Rectangle {
                width: 280
                height: 40
                radius: 20
                anchors.horizontalCenter: parent.horizontalCenter
                color: "#303030"
                border.color: "#606060"
                border.width: 2

                Text {
                    text: "Reset All Joints"
                    anchors.centerIn: parent
                    color: "#cccccc"
                    font.pixelSize: 14
                    font.bold: true
                    font.family: "Segoe UI"
                }

                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        root.resetAllJoints();
                    }
                }
            }
        }
    }
}
