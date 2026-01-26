import QtQuick
import QtQuick.Layouts

// JointGraph - Reusable real-time chart component for joint data visualization
Rectangle {
    id: root
    color: "#1e1e1e"
    radius: 10
    border.color: "#30ffffff"
    border.width: 1

    // Required properties
    required property string title
    required property var jointData         // Object: { joint0: [...], joint1: [...], ... }
    required property var selectedJoints    // Array: [true, false, true, ...]
    required property var jointColors       // Array: ["#ff4444", "#44ff44", ...]

    // Graph state (can be synced across multiple graphs)
    property real timeZoom: 1.0
    property real timeOffset: 1.0
    property int maxDataPoints: 500
    property bool useRadians: true
    property string unit: "rad"

    // Auto-scale smoothing
    property real minSmooth: 0
    property real maxSmooth: 0

    // Signals for graph interaction
    signal zoomChanged(real newZoom)
    signal offsetChanged(real newOffset)

    // Function to get current value for display
    function getCurrentValue(jointIndex) {
        var jointName = "joint" + jointIndex;
        if (root.jointData[jointName] && root.jointData[jointName].length > 0) {
            return root.jointData[jointName][root.jointData[jointName].length - 1];
        }
        return 0;
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 15

        Row {
            Layout.fillWidth: true
            Layout.preferredHeight: 30
            spacing: 10

            Text {
                text: root.title
                color: "#cccccc"
                font.pixelSize: 16
                font.bold: true
                anchors.verticalCenter: parent.verticalCenter
            }

            Item {
                width: 10
            }

            // Real-time values for selected joints
            Row {
                spacing: 12

                Repeater {
                    model: 7

                    Text {
                        visible: root.selectedJoints[index]
                        text: {
                            var val = root.getCurrentValue(index);
                            var displayVal = root.useRadians ? val : (val * 180 / Math.PI);
                            return "J" + index + ": " + displayVal.toFixed(2) + " " + root.unit;
                        }
                        color: root.jointColors[index]
                        font.pixelSize: 11
                        font.family: "Monospace"
                        font.bold: true
                    }
                }
            }

            Item {
                Layout.fillWidth: true
            }
        }

        Canvas {
            id: graphCanvas
            Layout.fillWidth: true
            Layout.fillHeight: true

            onPaint: {
                var ctx = getContext("2d");
                ctx.clearRect(0, 0, width, height);

                // Calculate zoom window
                var visibleWindow = root.maxDataPoints / root.timeZoom;

                // Calculate auto-scale based on visible data only
                var dataMin = 0, dataMax = 0;
                var hasData = false;

                for (var j = 0; j < 7; j++) {
                    if (!root.selectedJoints[j])
                        continue;
                    var jointName = "joint" + j;
                    if (!root.jointData[jointName])
                        continue;
                    var data = root.jointData[jointName];
                    if (data.length < 2)
                        continue;

                    // Calculate visible range
                    var totalAvailable = data.length;
                    var effectiveVisible = Math.min(visibleWindow, totalAvailable);
                    var startFloat = (totalAvailable - effectiveVisible) * root.timeOffset;
                    var endFloat = startFloat + effectiveVisible;
                    var startIndex = Math.floor(Math.max(0, startFloat));
                    var endIndex = Math.ceil(Math.min(totalAvailable, endFloat));

                    // Find min/max in visible range
                    for (var k = startIndex; k < endIndex; k++) {
                        if (!hasData) {
                            dataMin = data[k];
                            dataMax = data[k];
                            hasData = true;
                        } else {
                            dataMin = Math.min(dataMin, data[k]);
                            dataMax = Math.max(dataMax, data[k]);
                        }
                    }
                }

                // Add 10% padding
                var dataRange = Math.max(0.1, dataMax - dataMin);
                var targetMin = dataMin - dataRange * 0.1;
                var targetMax = dataMax + dataRange * 0.1;

                // Apply exponential smoothing
                var alpha = 0.2;
                if (root.minSmooth === 0 && root.maxSmooth === 0) {
                    root.minSmooth = targetMin;
                    root.maxSmooth = targetMax;
                } else {
                    root.minSmooth = root.minSmooth * (1 - alpha) + targetMin * alpha;
                    root.maxSmooth = root.maxSmooth * (1 - alpha) + targetMax * alpha;
                }

                var scaleMin = root.minSmooth;
                var scaleMax = root.maxSmooth;
                var range = scaleMax - scaleMin;
                if (range === 0)
                    range = 1;

                // Draw grid
                ctx.strokeStyle = "#2a2a2a";
                ctx.lineWidth = 1;
                for (var i = 0; i <= 5; i++) {
                    var y = height * i / 5;
                    ctx.beginPath();
                    ctx.moveTo(40, y);
                    ctx.lineTo(width, y);
                    ctx.stroke();

                    // Y-axis labels
                    var value = scaleMax - (range * i / 5);
                    ctx.fillStyle = "#888888";
                    ctx.font = "10px monospace";
                    ctx.fillText(value.toFixed(2), 2, y + 3);
                }

                // Draw joint data
                for (var j = 0; j < 7; j++) {
                    if (!root.selectedJoints[j])
                        continue;
                    var jointName = "joint" + j;
                    if (!root.jointData[jointName])
                        continue;
                    var data = root.jointData[jointName];
                    if (data.length < 2)
                        continue;

                    var totalAvailable = data.length;
                    var effectiveVisible = Math.min(visibleWindow, totalAvailable);
                    var startFloat = (totalAvailable - effectiveVisible) * root.timeOffset;
                    var endFloat = startFloat + effectiveVisible;
                    var startIndex = Math.floor(Math.max(0, startFloat));
                    var endIndex = Math.ceil(Math.min(totalAvailable, endFloat));

                    // Optimize rendering for large datasets
                    var graphWidth = width - 40;
                    var step = 1;
                    if (effectiveVisible > graphWidth) {
                        step = Math.ceil(effectiveVisible / graphWidth);
                    }

                    ctx.strokeStyle = root.jointColors[j];
                    ctx.lineWidth = 2;
                    ctx.beginPath();

                    var firstPoint = true;
                    for (var i = startIndex; i < endIndex; i += step) {
                        var xNorm = (i - startFloat) / effectiveVisible;
                        var x = 40 + (xNorm * graphWidth);

                        if (x < 40)
                            continue;
                        if (x > width)
                            break;

                        var normalized = (data[i] - scaleMin) / range;
                        var y = height - (normalized * height);

                        if (firstPoint) {
                            ctx.moveTo(x, y);
                            firstPoint = false;
                        } else {
                            ctx.lineTo(x, y);
                        }
                    }
                    ctx.stroke();
                }
            }

            // Request repaint when data changes
            Connections {
                target: root
                function onJointDataChanged() {
                    graphCanvas.requestPaint();
                }
                function onSelectedJointsChanged() {
                    graphCanvas.requestPaint();
                }
                function onTimeZoomChanged() {
                    graphCanvas.requestPaint();
                }
                function onTimeOffsetChanged() {
                    graphCanvas.requestPaint();
                }
            }

            MouseArea {
                anchors.fill: parent
                property real startX
                property real startOffset

                onWheel: wheel => {
                    if (wheel.angleDelta.y > 0) {
                        root.timeZoom = Math.min(root.timeZoom * 1.2, 20.0);
                    } else {
                        root.timeZoom = Math.max(root.timeZoom * 0.8, 1.0);
                    }
                    root.zoomChanged(root.timeZoom);
                    graphCanvas.requestPaint();
                }

                onPressed: mouse => {
                    startX = mouse.x;
                    startOffset = root.timeOffset;
                }

                onPositionChanged: mouse => {
                    if (pressed && root.timeZoom > 1.0) {
                        var graphWidth = width - 40;
                        var deltaNorm = (mouse.x - startX) / graphWidth;
                        var change = deltaNorm / (root.timeZoom - 1.0) * -1.0;
                        root.timeOffset = Math.max(0.0, Math.min(startOffset + change, 1.0));
                        root.offsetChanged(root.timeOffset);
                        graphCanvas.requestPaint();
                    }
                }
            }
        }
    }

    // Refresh method for external trigger
    function refresh() {
        graphCanvas.requestPaint();
    }
}
