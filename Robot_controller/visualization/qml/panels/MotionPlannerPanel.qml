import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Motion Planner Panel - Waypoint management and path programming
Rectangle {
    id: root
    color: "transparent"

    // Required properties from parent
    required property var waypointModel
    // Single source of truth for gizmo pose in URDF frame
    required property var getGizmoPoseURDF

    // RobotNode for FK calculations
    property var robotNode: null

    // Motion controller for execution
    property var motionController: null

    // Internal state
    property bool codeMode: false
    property string codeText: ""
    property double lastAddTimestamp: 0

    // Execution state (from motionController)
    property bool isExecuting: motionController ? motionController.isExecuting : false
    property bool isPaused: motionController ? motionController.isPaused : false
    property string executionStatus: motionController ? motionController.statusText : "Idle"
    property int currentWaypointIndex: motionController ? motionController.currentWaypointIndex : -1

    // Signals
    signal previewPath
    signal executePath

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 15
        spacing: 10

        // Header with mode toggle
        RowLayout {
            Layout.fillWidth: true
            spacing: 10

            Text {
                text: "Motion Program"
                color: "#ffffff"
                font.pixelSize: 16
                font.bold: true
            }

            Item {
                Layout.fillWidth: true
            }

            // UI / Code Toggle
            Rectangle {
                width: 100
                height: 28
                radius: 14
                color: "#202020"
                border.color: "#404040"

                Row {
                    anchors.fill: parent

                    Rectangle {
                        width: parent.width / 2
                        height: parent.height
                        radius: 14
                        color: !root.codeMode ? "#FF9800" : "transparent"
                        Text {
                            anchors.centerIn: parent
                            text: "UI"
                            color: !root.codeMode ? "white" : "#888"
                            font.pixelSize: 11
                            font.bold: true
                        }
                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: root.codeMode = false
                        }
                    }

                    Rectangle {
                        width: parent.width / 2
                        height: parent.height
                        radius: 14
                        color: root.codeMode ? "#2196F3" : "transparent"
                        Text {
                            anchors.centerIn: parent
                            text: "Code"
                            color: root.codeMode ? "white" : "#888"
                            font.pixelSize: 11
                            font.bold: true
                        }
                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                root.codeText = root.waypointModel.generateCode();
                                root.codeMode = true;
                            }
                        }
                    }
                }
            }
        }

        // Add Point button row
        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Item {
                Layout.fillWidth: true
            }

            // Add Home button (FK at all zeros)
            Rectangle {
                visible: !root.codeMode
                width: 110
                height: 36
                radius: 18
                color: "#4CAF50"
                border.color: "#66BB6A"
                border.width: 1

                Row {
                    anchors.centerIn: parent
                    spacing: 6
                    Text {
                        text: "🏠"
                        color: "white"
                        font.pixelSize: 14
                    }
                    Text {
                        text: "Add Home"
                        color: "white"
                        font.pixelSize: 12
                        font.bold: true
                    }
                }

                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        var now = Date.now();
                        if (now - root.lastAddTimestamp < 500)
                            return;
                        root.lastAddTimestamp = now;

                        // Compute home position via FK at all zeros
                        if (!root.robotNode) {
                            console.log("Error: robotNode not available for FK calculation");
                            return;
                        }

                        // Call FK with all joints at zero
                        var zeroJoints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
                        var pose = root.robotNode.getEndEffectorPose(zeroJoints);

                        if (!pose.success) {
                            console.log("Error: FK calculation failed");
                            return;
                        }

                        console.log("Adding Home waypoint via FK:", pose.x, pose.y, pose.z);

                        root.waypointModel.addWaypoint("Home", pose.x, pose.y, pose.z, pose.qw, pose.qx, pose.qy, pose.qz, 0);
                    }
                }
            }

            // Capture Current Pose
            Rectangle {
                visible: !root.codeMode
                width: 120
                height: 36
                radius: 18
                color: "#FF9800"
                border.color: "#FFB74D"
                border.width: 1

                Row {
                    anchors.centerIn: parent
                    spacing: 6
                    Text {
                        text: "+"
                        color: "white"
                        font.pixelSize: 16
                        font.bold: true
                    }
                    Text {
                        text: "Add Point"
                        color: "white"
                        font.pixelSize: 12
                        font.bold: true
                    }
                }

                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        var now = Date.now();
                        if (now - root.lastAddTimestamp < 500)
                            return;
                        root.lastAddTimestamp = now;

                        // Capture current end-effector pose
                        // Get pose in URDF frame using single source of truth
                        var pose = root.getGizmoPoseURDF();
                        var urdfX = pose.position.x;
                        var urdfY = pose.position.y;
                        var urdfZ = pose.position.z;
                        var rot = pose.quaternion;

                        console.log("Adding waypoint:", urdfX, urdfY, urdfZ, "quat:", rot.scalar, rot.x, rot.y, rot.z);

                        // Generate unique name
                        var newName = "Point 1";
                        var counter = 1;
                        var unique = false;
                        while (!unique) {
                            newName = "Point " + counter;
                            unique = true;
                            for (var i = 0; i < root.waypointModel.count; ++i) {
                                if (root.waypointModel.internalModel.get(i).name === newName) {
                                    unique = false;
                                    break;
                                }
                            }
                            if (!unique)
                                counter++;
                        }

                        root.waypointModel.addWaypoint(newName, urdfX, urdfY, urdfZ, rot.scalar, rot.x, rot.y, rot.z, 0  // PTP by default
                        );
                    }
                }
            }
        }

        // Waypoint List (UI mode only)
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#1a1a1a"
            radius: 8
            border.color: "#303030"
            border.width: 1
            visible: !root.codeMode

            ListView {
                id: waypointListView
                anchors.fill: parent
                anchors.margins: 8
                clip: true
                model: root.waypointModel.internalModel
                spacing: 6

                delegate: Rectangle {
                    id: delegateItem
                    property int itemIndex: index
                    width: ListView.view.width
                    height: 85
                    radius: 6
                    color: root.waypointModel.selectedIndex === index ? "#404040" : "#2a2a2a"
                    border.color: root.waypointModel.selectedIndex === index ? "#FF9800" : "#303030"
                    border.width: root.waypointModel.selectedIndex === index ? 2 : 1

                    MouseArea {
                        anchors.fill: parent
                        onClicked: root.waypointModel.selectWaypoint(index)
                        z: -1 // Ensure it's behind other interactive elements if necessary, or rely on propagation order?
                        // Actually, if we put it here, it might block the internal mouse areas.
                        // Better to wrap the RowLayout in it or place it before RowLayout?
                        // QML: last defined is on top. If we define it first, other MouseAreas on top will steal events.
                        // So we should define it *inside* the Delegate Rectangle but *before* RowLayout?
                        // Wait, RowLayout is lines 210+.
                        // So inserting here (line 209) puts it *before* RowLayout.
                        // It will be behind RowLayout items visually, but MouseArea without z stacking depends on instantiation order?
                        // Actually, if it fills parent, it covers everything. We want it *behind* the buttons.
                        // So we should put it first in the hierarchy (top of delegate content).
                        // BUT, code wise, if I put it here, it is processed. Child items (RowLayout) will be on top.
                        // If they don't consume mouse events, this one will.
                        // The inner MouseAreas (for Type Selector) *will* consume events.
                        // So this should work fine.
                    }

                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 10

                        // Index badge with motion type color
                        Rectangle {
                            width: 28
                            height: 28
                            radius: 14
                            color: {
                                switch (motionType) {
                                case 0:
                                    return "#4CAF50";  // PTP - Green
                                case 1:
                                    return "#2196F3";  // LIN - Blue
                                case 2:
                                    return "#FF9800";  // CIRC - Orange
                                default:
                                    return "#666666";
                                }
                            }
                            Text {
                                anchors.centerIn: parent
                                text: index + 1
                                color: "white"
                                font.pixelSize: 12
                                font.bold: true
                            }
                        }

                        // Waypoint info
                        Column {
                            Layout.fillWidth: true
                            spacing: 3

                            Text {
                                text: name
                                color: "#ffffff"
                                font.pixelSize: 13
                                font.bold: true
                            }

                            Text {
                                text: "XYZ: " + posX.toFixed(3) + ", " + posY.toFixed(3) + ", " + posZ.toFixed(3)
                                color: "#888888"
                                font.pixelSize: 9
                                font.family: "Monospace"
                            }

                            Text {
                                text: "RPY: " + roll.toFixed(1) + "°, " + pitch.toFixed(1) + "°, " + yaw.toFixed(1) + "°"
                                color: "#7799aa"
                                font.pixelSize: 9
                                font.family: "Monospace"
                            }
                        }

                        // Motion Type Selector
                        Row {
                            spacing: 4
                            Repeater {
                                model: ["PTP", "LIN", "CIRC"]
                                Rectangle {
                                    width: 36
                                    height: 22
                                    radius: 11
                                    color: motionType === index ? (index === 0 ? "#4CAF50" : index === 1 ? "#2196F3" : "#FF9800") : "#303030"
                                    Text {
                                        anchors.centerIn: parent
                                        text: modelData
                                        color: motionType === index ? "white" : "#666666"
                                        font.pixelSize: 9
                                        font.bold: true
                                    }
                                    MouseArea {
                                        anchors.fill: parent
                                        cursorShape: Qt.PointingHandCursor
                                        onClicked: root.waypointModel.updateWaypointMotionType(delegateItem.itemIndex, index)
                                    }
                                }
                            }
                        }

                        // Delete button
                        Rectangle {
                            width: 26
                            height: 26
                            radius: 13
                            color: "#303030"
                            border.color: "#f44336"
                            border.width: 1

                            Text {
                                anchors.centerIn: parent
                                text: "×"
                                color: "#f44336"
                                font.pixelSize: 16
                                font.bold: true
                            }

                            MouseArea {
                                anchors.fill: parent
                                cursorShape: Qt.PointingHandCursor
                                onClicked: root.waypointModel.removeWaypoint(index)
                            }
                        }
                    }

                    MouseArea {
                        anchors.fill: parent
                        z: -1
                        onClicked: root.waypointModel.selectedIndex = index
                    }
                }

                // Empty state
                Text {
                    anchors.centerIn: parent
                    visible: root.waypointModel.count === 0
                    text: "No waypoints defined.\nClick 'Add Point' to capture current pose."
                    color: "#666666"
                    font.pixelSize: 13
                    horizontalAlignment: Text.AlignHCenter
                }
            }
        }

        // Bottom controls (UI mode only)
        RowLayout {
            Layout.fillWidth: true
            spacing: 10
            visible: !root.codeMode && !root.isExecuting

            Rectangle {
                Layout.fillWidth: true
                height: 36
                radius: 18
                color: "#303030"
                border.color: "#606060"
                border.width: 1

                Text {
                    anchors.centerIn: parent
                    text: "Clear All"
                    color: "#cccccc"
                    font.pixelSize: 12
                    font.bold: true
                }

                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: root.waypointModel.clearAll()
                }
            }

            Rectangle {
                Layout.fillWidth: true
                height: 36
                radius: 18
                color: root.waypointModel.count >= 1 ? "#4CAF50" : "#303030"
                border.color: root.waypointModel.count >= 1 ? "#66BB6A" : "#606060"
                border.width: 1
                opacity: root.waypointModel.count >= 1 ? 1.0 : 0.5

                Text {
                    anchors.centerIn: parent
                    text: "▶ Play"
                    color: root.waypointModel.count >= 1 ? "white" : "#666666"
                    font.pixelSize: 12
                    font.bold: true
                }

                MouseArea {
                    anchors.fill: parent
                    cursorShape: root.waypointModel.count >= 1 ? Qt.PointingHandCursor : Qt.ArrowCursor
                    enabled: root.waypointModel.count >= 1
                    onClicked: root.executePath()
                }
            }
        }

        // Execution controls (visible during execution)
        ColumnLayout {
            Layout.fillWidth: true
            spacing: 8
            visible: !root.codeMode && root.isExecuting

            // Status text
            Rectangle {
                Layout.fillWidth: true
                height: 40
                radius: 8
                color: "#1a3a1a"
                border.color: "#4CAF50"
                border.width: 1

                Row {
                    anchors.centerIn: parent
                    spacing: 10

                    // Animated spinner
                    Rectangle {
                        width: 16
                        height: 16
                        radius: 8
                        color: "transparent"
                        border.color: "#4CAF50"
                        border.width: 2

                        Rectangle {
                            width: 6
                            height: 6
                            radius: 3
                            color: "#4CAF50"
                            anchors.top: parent.top
                            anchors.horizontalCenter: parent.horizontalCenter
                            anchors.topMargin: 2
                        }

                        RotationAnimation on rotation {
                            from: 0
                            to: 360
                            duration: 1000
                            loops: Animation.Infinite
                            running: root.isExecuting
                        }
                    }

                    Text {
                        text: root.executionStatus
                        color: "#66BB6A"
                        font.pixelSize: 12
                        font.bold: true
                    }
                }
            }

            // Stop button
            Rectangle {
                Layout.fillWidth: true
                height: 36
                radius: 18
                color: "#f44336"
                border.color: "#ef5350"
                border.width: 1

                Text {
                    anchors.centerIn: parent
                    text: "■ Stop"
                    color: "white"
                    font.pixelSize: 12
                    font.bold: true
                }

                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        if (root.motionController) {
                            root.motionController.stop();
                        }
                    }
                }
            }
        }

        // ================================
        // CODE EDITOR PANEL (Code mode only)
        // ================================
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#1a1a1a"
            radius: 8
            border.color: "#2196F3"
            border.width: 1
            visible: root.codeMode

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 8

                // Code hint
                Text {
                    text: "// ptp(x, y, z, roll, pitch, yaw)  // Point name"
                    color: "#666666"
                    font.pixelSize: 10
                    font.family: "Monospace"
                }

                // Code TextArea
                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true

                    TextArea {
                        id: codeEditor
                        text: root.codeText
                        onTextChanged: root.codeText = text
                        color: "#e0e0e0"
                        font.family: "Monospace"
                        font.pixelSize: 11
                        background: Rectangle {
                            color: "#0a0a0a"
                        }
                        wrapMode: TextEdit.NoWrap
                        selectByMouse: true
                    }
                }
            }
        }

        // Code mode bottom controls
        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            visible: root.codeMode

            // Apply Code button
            Rectangle {
                Layout.fillWidth: true
                height: 36
                radius: 18
                color: "#4CAF50"
                border.color: "#66BB6A"
                border.width: 1

                Text {
                    anchors.centerIn: parent
                    text: "▶ Apply Code"
                    color: "white"
                    font.pixelSize: 12
                    font.bold: true
                }

                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        root.waypointModel.parseCode(root.codeText);
                        root.codeMode = false;
                    }
                }
            }

            // Save button
            Rectangle {
                width: 60
                height: 36
                radius: 18
                color: "#303030"
                border.color: "#606060"

                Text {
                    anchors.centerIn: parent
                    text: "💾 Save"
                    color: "#cccccc"
                    font.pixelSize: 11
                }

                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: console.log("Save dialog - TODO")
                }
            }

            // Load button
            Rectangle {
                width: 60
                height: 36
                radius: 18
                color: "#303030"
                border.color: "#606060"

                Text {
                    anchors.centerIn: parent
                    text: "📂 Load"
                    color: "#cccccc"
                    font.pixelSize: 11
                }

                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: console.log("Load dialog - TODO")
                }
            }
        }
    }
}
