import QtQuick
import QtQuick.Window
import QtQuick3D
import QtQuick3D.Helpers
import QtQuick3D.AssetUtils
import QtQuick.Layouts
import QtQuick.Controls
import RobotVisualization 1.0
import "panels" as Panels
import "view3d"

ApplicationWindow {
    id: window
    width: 1400
    height: 900
    visible: true
    visibility: Window.Maximized
    title: "Robot Visualization - AAR6"
    color: "#1a1a1a" // Fix for white background artifact

    property int showAxesMode: 0  // 0=Off, 1=EE link only, 2=All joints
    property bool showGizmo: true  // Show 3D transform gizmo at ee_link

    property var currentLinks: []
    // Increased camera distance to match robot scale (100x)
    property real cameraDistance: 200.0
    property real panSpeed: 0.2
    property real orbitSpeed: 1.0
    property real zoomSpeed: 20.0

    // Jog velocity options (in m/s for linear, rad/s for rotation)
    property var jogSpeedOptions: [0.10, 0.20, 0.30, 0.40]  // 100, 200, 300, 400 mm/s
    property var jogSpeedLabels: ["100", "200", "300", "400"]  // Labels for UI (mm/s)
    property var rotJogSpeedOptions: [1.0, 2.0, 3.0, 4.0]  // rad/s (~60, 120, 180, 240 deg/s)
    property int jogSpeedIndex: 0  // Default index (100 mm/s - slowest)

    // Mode control
    property bool isSimulationMode: false
    property var jointAngles: ({})  // Map of joint names to angles

    // IK control
    property bool ikEnabled: true  // Enable IK when gizmo is dragged
    property bool ikSolving: false
    property string ikStatusText: ""
    property vector3d lastValidGizmoPosition: Qt.vector3d(0, 0, 0)  // mm, last IK-valid position (inits to gizmo start)

    // Velocity-based control (for robot mode)
    property bool useVelocityMode: true   // Use velocity IK in robot mode
    property real velocityScale: 5.0      // Scale factor for drag-to-velocity
    property real lastDragTime: 0         // For velocity calculation

    // Separate gizmo state for each mode (so switching doesn't carry over positions)
    property vector3d simGizmoPosition: Qt.vector3d(0, 0, 0)
    property vector3d simGizmoRotation: Qt.vector3d(0, 0, 0)
    property bool simGizmoHasBeenDragged: false

    property vector3d robotGizmoPosition: Qt.vector3d(0, 0, 0)
    property vector3d robotGizmoRotation: Qt.vector3d(0, 0, 0)
    property bool robotGizmoHasBeenDragged: false

    // Joint names in order for IK (arm joints only)
    property var ikJointNames: ["L1", "L2", "L3", "L4", "L5", "L6"]

    // Helper: Get joint value by index, using jointAngles in simulation mode
    function getJointValueByIndex(index) {
        if (isSimulationMode) {
            // In simulation mode, use jointAngles map
            if (index < ikJointNames.length) {
                return jointAngles[ikJointNames[index]] || 0.0;
            } else if (index === 6) {
                return jointAngles["left_finger_joint"] || 0.0;
            }
            return 0.0;
        } else {
            // In robot mode, use currentJointPositions from robot
            return (index < currentJointPositions.length) ? currentJointPositions[index] : 0.0;
        }
    }

    // Function to solve IK and apply joint angles - uses URDF pose directly
    function solveIKWithURDFPose(urdfPos, urdfQuat) {
        if (!robotNode || !robotNode.ikInitialized || !ikEnabled) {
            console.log("IK not available or disabled");
            return;
        }

        ikSolving = true;
        ikStatusText = "Solving...";

        // Get current joint angles as array for initial guess
        var currentJoints = [];
        for (var i = 0; i < ikJointNames.length; i++) {
            var jointName = ikJointNames[i];
            currentJoints.push(jointAngles[jointName] || 0);
        }

        console.log("[IK] Target (URDF frame):", urdfPos.x.toFixed(4), urdfPos.y.toFixed(4), urdfPos.z.toFixed(4), "m");
        console.log("[IK] Quat (w,x,y,z):", urdfQuat.scalar.toFixed(4), urdfQuat.x.toFixed(4), urdfQuat.y.toFixed(4), urdfQuat.z.toFixed(4));

        // Call IK solver
        var result = robotNode.solveIKFromGizmo(urdfPos, urdfQuat, currentJoints);

        if (result.success) {
            var newAngles = result.jointAngles;

            if (isSimulationMode) {
                // SIMULATION MODE: Apply joint angles directly to visualization
                for (var j = 0; j < ikJointNames.length && j < newAngles.length; j++) {
                    jointAngles[ikJointNames[j]] = newAngles[j];
                }
                // Trigger update
                jointAngles = Object.assign({}, jointAngles);

                // Also update simulationJointPositions for slider sync
                var simPos = simulationJointPositions.slice();
                for (var k = 0; k < ikJointNames.length && k < newAngles.length; k++) {
                    simPos[k] = newAngles[k];
                }
                simulationJointPositions = simPos;
            } else {
                // ROBOT MODE: Send to MCU, visualization updates from feedback
                console.log("[IK->MCU] Sending joint positions:", newAngles);
                robotManager.sendAllJoints(newAngles);
            }

            // Save position for recovery
            lastValidGizmoPosition = transformGizmo.gizmoPosition;

            ikStatusText = "IK solved in " + result.solveTimeMs.toFixed(1) + "ms";
        } else {
            ikStatusText = "IK failed: " + result.message;
            console.log("IK failed:", result.message, "error:", result.error);
        }

        ikSolving = false;
    }

    // Transform a local-frame vector to world frame using the tool's rotation
    // This enables tool-frame Cartesian control
    function transformToWorldFrame(localVec, rotation) {
        // Quaternion rotation of a vector: v' = q * v * q^-1
        // For QQuaternion, we can use the built-in method or manual calculation
        // rotation is a QQuaternion (w, x, y, z)
        var qw = rotation.scalar;
        var qx = rotation.x;
        var qy = rotation.y;
        var qz = rotation.z;

        var vx = localVec.x;
        var vy = localVec.y;
        var vz = localVec.z;

        // Rotation matrix from quaternion
        var r00 = 1 - 2 * (qy * qy + qz * qz);
        var r01 = 2 * (qx * qy - qz * qw);
        var r02 = 2 * (qx * qz + qy * qw);
        var r10 = 2 * (qx * qy + qz * qw);
        var r11 = 1 - 2 * (qx * qx + qz * qz);
        var r12 = 2 * (qy * qz - qx * qw);
        var r20 = 2 * (qx * qz - qy * qw);
        var r21 = 2 * (qy * qz + qx * qw);
        var r22 = 1 - 2 * (qx * qx + qy * qy);

        return Qt.vector3d(r00 * vx + r01 * vy + r02 * vz, r10 * vx + r11 * vy + r12 * vz, r20 * vx + r21 * vy + r22 * vz);
    }

    // Convert euler angles (in degrees) to quaternion
    // Uses ZYX (yaw-pitch-roll) convention
    function eulerToQuaternion(euler) {
        // Convert degrees to radians
        var roll = euler.x * Math.PI / 180.0;   // X rotation
        var pitch = euler.y * Math.PI / 180.0;  // Y rotation
        var yaw = euler.z * Math.PI / 180.0;    // Z rotation

        var cy = Math.cos(yaw * 0.5);
        var sy = Math.sin(yaw * 0.5);
        var cp = Math.cos(pitch * 0.5);
        var sp = Math.sin(pitch * 0.5);
        var cr = Math.cos(roll * 0.5);
        var sr = Math.sin(roll * 0.5);

        var w = cr * cp * cy + sr * sp * sy;
        var x = sr * cp * cy - cr * sp * sy;
        var y = cr * sp * cy + sr * cp * sy;
        var z = cr * cp * sy - sr * sp * cy;

        return Qt.quaternion(w, x, y, z);
    }

    // Convert quaternion to euler angles (degrees)
    // Uses ZYX (yaw-pitch-roll) convention - inverse of eulerToQuaternion
    function quaternionToEuler(q) {
        var qw = q.scalar;
        var qx = q.x;
        var qy = q.y;
        var qz = q.z;

        // Roll (X-axis rotation)
        var sinr_cosp = 2 * (qw * qx + qy * qz);
        var cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        var roll = Math.atan2(sinr_cosp, cosr_cosp);

        // Pitch (Y-axis rotation)
        var sinp = 2 * (qw * qy - qz * qx);
        var pitch;
        if (Math.abs(sinp) >= 1) {
            pitch = Math.sign(sinp) * Math.PI / 2; // use 90 degrees if out of range
        } else {
            pitch = Math.asin(sinp);
        }

        // Yaw (Z-axis rotation)
        var siny_cosp = 2 * (qw * qz + qx * qy);
        var cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        var yaw = Math.atan2(siny_cosp, cosy_cosp);

        // Convert to degrees
        return Qt.vector3d(roll * 180 / Math.PI, pitch * 180 / Math.PI, yaw * 180 / Math.PI);
    }

    // Transform quaternion from scene frame to URDF frame
    // Scene has -90° X rotation relative to URDF, so we apply +90° X rotation
    function sceneQuatToUrdf(sceneQuat) {
        // Quaternion for +90° X rotation: q_corr = (cos(45°), sin(45°), 0, 0)
        var qCorrW = 0.7071067811865476;
        var qCorrX = 0.7071067811865476;
        var qCorrY = 0.0;
        var qCorrZ = 0.0;

        var qSceneW = sceneQuat.scalar;
        var qSceneX = sceneQuat.x;
        var qSceneY = sceneQuat.y;
        var qSceneZ = sceneQuat.z;

        // Quaternion multiplication: q_urdf = q_corr * q_scene
        var qUrdfW = qCorrW * qSceneW - qCorrX * qSceneX - qCorrY * qSceneY - qCorrZ * qSceneZ;
        var qUrdfX = qCorrW * qSceneX + qCorrX * qSceneW + qCorrY * qSceneZ - qCorrZ * qSceneY;
        var qUrdfY = qCorrW * qSceneY - qCorrX * qSceneZ + qCorrY * qSceneW + qCorrZ * qSceneX;
        var qUrdfZ = qCorrW * qSceneZ + qCorrX * qSceneY - qCorrY * qSceneX + qCorrZ * qSceneW;

        return Qt.quaternion(qUrdfW, qUrdfX, qUrdfY, qUrdfZ);
    }

    // Single source of truth: Get gizmo pose in URDF frame
    function getGizmoPoseURDF() {
        var pos = transformGizmo.gizmoPosition;
        var euler = transformGizmo.gizmoEulerRotation;

        // Position: Scene -> URDF (scale 100, +90°X rotation)
        var urdfPos = Qt.vector3d(pos.x / 100.0, -pos.z / 100.0, pos.y / 100.0);

        // Quaternion: Scene -> URDF
        var sceneQuat = eulerToQuaternion(euler);
        var urdfQuat = sceneQuatToUrdf(sceneQuat);

        return {
            position: urdfPos,
            quaternion: urdfQuat
        };
    }

    // Solve IK using gizmo's current position and euler rotation
    function solveIKFromGizmo() {
        var pose = getGizmoPoseURDF();
        window.solveIKWithURDFPose(pose.position, pose.quaternion);
    }

    // Send constant Cartesian velocity for button-based control
    // linearVel and angularVel are in URDF frame (m/s and rad/s)
    function sendCartesianVelocity(linearVel, angularVel) {
        if (!robotNode || !robotNode.ikInitialized || !ikEnabled)
            return;

        // Get current joint positions
        var currentJoints = [];
        for (var i = 0; i < currentJointPositions.length && i < 6; i++) {
            currentJoints.push(currentJointPositions[i]);
        }

        // Get current EE pose via FK
        var fkResult = robotNode.getEndEffectorPose(currentJoints);
        if (!fkResult.success)
            return;

        // Use larger dt for responsive jog control (200ms look-ahead)
        var dt = 0.2;

        // Compute target position by integrating linear velocity
        var targetPos = Qt.vector3d(fkResult.x + linearVel.x * dt, fkResult.y + linearVel.y * dt, fkResult.z + linearVel.z * dt);

        // Compute target orientation by integrating angular velocity
        // For small angles: q_new = q * (1, wx*dt/2, wy*dt/2, wz*dt/2) normalized
        var dax = angularVel.x * dt * 0.5;
        var day = angularVel.y * dt * 0.5;
        var daz = angularVel.z * dt * 0.5;

        // Quaternion multiplication: q_target = q_current * dq
        var qw = fkResult.qw;
        var qx = fkResult.qx;
        var qy = fkResult.qy;
        var qz = fkResult.qz;

        var newQw = qw - dax * qx - day * qy - daz * qz;
        var newQx = qx + dax * qw + day * qz - daz * qy;
        var newQy = qy - dax * qz + day * qw + daz * qx;
        var newQz = qz + dax * qy - day * qx + daz * qw;

        // Normalize
        var norm = Math.sqrt(newQw * newQw + newQx * newQx + newQy * newQy + newQz * newQz);
        if (norm > 0.0001) {
            newQw /= norm;
            newQx /= norm;
            newQy /= norm;
            newQz /= norm;
        }

        var targetQuat = Qt.quaternion(newQw, newQx, newQy, newQz);

        // Use unified IK solver (switch in RobotNode.h USE_SQP_IK)
        var jointVels = robotNode.solveCartesianIK(currentJoints, currentJointVelocities, targetPos, targetQuat, dt);

        // Update velocity state for visualization
        if (jointVels.length >= 6) {
            currentJointVelocities = [jointVels[0], jointVels[1], jointVels[2], jointVels[3], jointVels[4], jointVels[5]];
        }

        // Send velocities to robot
        robotManager.sendJointVelocities(jointVels);
    }

    // Cartesian jog speed - controlled by UI selector
    property real cartesianJogSpeed: jogSpeedOptions[jogSpeedIndex]  // m/s
    property real rotationJogSpeed: rotJogSpeedOptions[jogSpeedIndex]  // rad/s

    // Stop all Cartesian velocity - send zeros directly to robot
    function stopCartesianVelocity() {
        if (!isSimulationMode && useVelocityMode) {
            // Send zeros directly, bypassing IK to ensure immediate stop
            var zeroVels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            robotManager.sendJointVelocities(zeroVels);
            currentJointVelocities = zeroVels;
        }
    }

    // Handle gizmo IK based on mode (velocity mode for robot, position mode for simulation)
    // This is the main entry point for all axis handlers during dragging
    function handleGizmoIK() {
        if (!window.ikEnabled)
            return;

        if (!isSimulationMode && useVelocityMode) {
            // VELOCITY IK MODE (Robot): Cartesian velocity control via IK
            var dtMs = Date.now() - window.lastDragTime;
            var dt = Math.max(dtMs / 1000.0, 0.01); // At least 10ms
            window.lastDragTime = Date.now();

            // Get target pose in URDF frame using single source of truth
            var pose = getGizmoPoseURDF();

            // Get current joint positions
            var currentJoints = [];
            for (var i = 0; i < currentJointPositions.length && i < 6; i++) {
                currentJoints.push(currentJointPositions[i]);
            }

            // Use unified IK solver (switch between PositionIK/SqpIK in RobotNode.h USE_SQP_IK)
            var jointVels = robotNode.solveCartesianIK(currentJoints, currentJointVelocities  // Velocity state for SqpIK
            , pose.position, pose.quaternion, dt);

            // Update velocity state for visualization and next iteration
            if (jointVels.length >= 6) {
                currentJointVelocities = [jointVels[0], jointVels[1], jointVels[2], jointVels[3], jointVels[4], jointVels[5]];
            }

            // Send velocities to robot
            robotManager.sendJointVelocities(jointVels);
        } else {
            // POSITION MODE (Simulation or position-based robot control)
            window.solveIKFromGizmo();
        }
    }

    // Function to reset gizmo to current end-effector position (FK sync)
    function resetGizmo() {
        var eeLinkNode = findNode(robotRoot, "ee_link");
        if (!eeLinkNode) {
            return;
        }

        // Sync gizmo position and rotation from FK
        transformGizmo.hasBeenDragged = false;
        transformGizmo.gizmoPosition = eeLinkNode.scenePosition;
        transformGizmo.rotation = eeLinkNode.sceneRotation;
        transformGizmo.gizmoEulerRotation = quaternionToEuler(eeLinkNode.sceneRotation);
    }
    // DEBUG: Hardcoded collision geometry world transforms from test_collision_checker
    // These are the transforms the collision checker computes for comparison
    property bool useDebugCollisionTransforms: false  // Set to true to visualize collision checker's view
    property var debugCollisionTransforms: ({
            "L1": {
                pos: Qt.vector3d(0.1450, 0.0000, 0.0690),
                quat: Qt.quaternion(0.0897, 0.0000, 0.0000, 0.9960)
            },
            "L2": {
                pos: Qt.vector3d(0.1220, 0.0042, 0.1795),
                quat: Qt.quaternion(-0.3874, -0.2755, 0.6512, -0.5915)
            },
            "L3": {
                pos: Qt.vector3d(0.2690, -0.0225, 0.2798),
                quat: Qt.quaternion(0.5221, -0.5989, -0.3760, -0.4769)
            },
            "L4": {
                pos: Qt.vector3d(0.2805, -0.0246, 0.3217),
                quat: Qt.quaternion(0.6011, 0.2350, 0.7603, 0.0726)
            },
            "L5": {
                pos: Qt.vector3d(0.1132, 0.0058, 0.3687),
                quat: Qt.quaternion(0.8324, -0.5368, 0.1238, -0.0596)
            },
            "L6": {
                pos: Qt.vector3d(0.0611, 0.0290, 0.2536),
                quat: Qt.quaternion(0.7851, 0.1816, -0.1130, 0.5812)
            },
            "left_finger": {
                pos: Qt.vector3d(0.1024, 0.0471, 0.3358),
                quat: Qt.quaternion(0.0399, -0.5907, 0.7565, 0.2778)
            },
            "right_finger": {
                pos: Qt.vector3d(0.1122, -0.0213, 0.3041),
                quat: Qt.quaternion(-0.0826, 0.8016, 0.5627, 0.1844)
            }
        })

    // Collision detection - per-link with color indices
    property var collidingPairs: []  // List of {link1, link2, colorIndex}
    property var collidingLinks: ({})  // Map of link name -> colorIndex

    // Collision colors (different colors for different collision pairs)
    property var collisionColors: ["#FF0000", "#FF8800", "#FFFF00", "#FF00FF", "#00FFFF", "#8800FF"]

    // Function to get collision color for a link, or null if not colliding
    function getCollisionColor(linkName) {
        if (collidingLinks.hasOwnProperty(linkName)) {
            var colorIdx = collidingLinks[linkName];
            return collisionColors[colorIdx % collisionColors.length];
        }
        return null;
    }

    // Function to check collision - works in both simulation and robot mode
    function checkCollision() {
        if (!robotNode)
            return;

        // Determine which joint positions to use based on mode
        var angles = [];
        if (isSimulationMode) {
            // Simulation mode uses simulationJointPositions
            for (var i = 0; i < 7; i++) {
                angles.push(simulationJointPositions[i] || 0.0);
            }
        } else {
            // Robot mode uses actual joint positions from MCU
            for (var i = 0; i < currentJointPositions.length && i < 7; i++) {
                angles.push(currentJointPositions[i] || 0.0);
            }
            // Pad if needed
            while (angles.length < 7)
                angles.push(0.0);
        }

        // Call the collision checker
        var pairs = robotNode.getCollidingPairs(angles);
        collidingPairs = pairs;

        // Build collidingLinks map for mesh coloring
        var links = {};
        for (var i = 0; i < pairs.length; i++) {
            var pair = pairs[i];
            if (pair.link1)
                links[pair.link1] = i;
            if (pair.link2)
                links[pair.link2] = i;
        }
        collidingLinks = links;
    }

    // Check collision in any mode (called when positions change)
    function checkCollisionIfSimulation() {
        checkCollision();
    }

    // Graph data storage (circular buffers for each joint)
    property int maxDataPoints: 2000
    property int dataUpdateTrigger: 0
    property real graphTimeZoom: 1.0
    property real graphTimeOffset: 1.0 // 1.0 = aligned to right (newest data)

    property var jointPositionData: ({})
    property var jointVelocityData: ({})
    property var jointAccelerationData: ({})
    property int dataIndex: 0

    // Joint selection for graphing (7 joints)
    property var selectedJoints: [false, false, false, false, false, false, false]

    // Current joint positions from MCU (updated live)
    property var currentJointPositions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    // Current joint velocities for SQP IK (tracked from velocity commands)
    property var currentJointVelocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    // Simulation mode joint positions (independent from MCU)
    property var simulationJointPositions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    // Check collision whenever simulation positions change
    onSimulationJointPositionsChanged: checkCollisionIfSimulation()

    // Check collision whenever robot positions change (from MCU)
    onCurrentJointPositionsChanged: if (!isSimulationMode)
        checkCollision()

    // Joint colors - consistent across all UI elements
    property var jointColors: ["#FF5252", "#4CAF50", "#2196F3", "#FF9800", "#9C27B0", "#00BCD4", "#FFEB3B"]

    // Toggle between radians and degrees for angle display
    property bool useRadians: true

    // Smoothed auto-scaling ranges (exponential moving average)
    property real positionMinSmooth: 0
    property real positionMaxSmooth: 0
    property real velocityMinSmooth: 0
    property real velocityMaxSmooth: 0
    property real accelerationMinSmooth: 0
    property real accelerationMaxSmooth: 0

    // Waypoint Model implementation for Motion Planner
    property var waypointModel: QtObject {
        property int selectedIndex: -1
        property alias count: internalModel.count

        property var internalModel: ListModel {
            id: internalModel
        }

        // Make the internal model accessible as 'model' for ListView
        // Note: ListView checks for standard model roles/methods
        function get(index) {
            return internalModel.get(index);
        }
        function set(index, obj) {
            internalModel.set(index, obj);
        }
        function move(from, to, n) {
            internalModel.move(from, to, n);
        }
        function remove(index, count) {
            internalModel.remove(index, count);
        }
        function clear() {
            internalModel.clear();
        }
        function append(dict) {
            internalModel.append(dict);
        }
        function insert(index, dict) {
            internalModel.insert(index, dict);
        }

        function addWaypoint(name, x, y, z, qw, qx, qy, qz, motionType) {
            // Calculate Euler angles for display
            var roll = Math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / Math.PI;
            var pitch = Math.asin(2.0 * (qw * qy - qz * qx)) * 180.0 / Math.PI;
            var yaw = Math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)) * 180.0 / Math.PI;

            console.log("Adding waypoint:", name, x, y, z);
            internalModel.append({
                "name": name,
                "posX": x,
                "posY": y,
                "posZ": z,
                "quatW": qw,
                "quatX": qx,
                "quatY": qy,
                "quatZ": qz,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "motionType": motionType // 0=PTP, 1=LIN, 2=CIRC
                ,
                "isSelected": false
            });
        }

        function removeWaypoint(index) {
            if (index >= 0 && index < internalModel.count) {
                internalModel.remove(index);
                if (selectedIndex === index)
                    selectedIndex = -1;
            }
        }

        function updateWaypointMotionType(index, type) {
            if (index >= 0 && index < internalModel.count) {
                internalModel.setProperty(index, "motionType", type);
            }
        }

        function selectWaypoint(index) {
            if (selectedIndex !== -1 && selectedIndex < internalModel.count) {
                internalModel.setProperty(selectedIndex, "isSelected", false);
            }
            selectedIndex = index;
            if (selectedIndex !== -1 && selectedIndex < internalModel.count) {
                internalModel.setProperty(selectedIndex, "isSelected", true);
            }
        }

        function clearAll() {
            internalModel.clear();
            selectedIndex = -1;
        }

        function generateCode() {
            var code = "";
            for (var i = 0; i < internalModel.count; ++i) {
                var wp = internalModel.get(i);
                var typeStr = wp.motionType === 1 ? "lin" : (wp.motionType === 2 ? "circ" : "ptp");
                // ptp(x, y, z, roll, pitch, yaw)
                code += typeStr + "(" + wp.posX.toFixed(4) + ", " + wp.posY.toFixed(4) + ", " + wp.posZ.toFixed(4) + ", " + wp.roll.toFixed(2) + ", " + wp.pitch.toFixed(2) + ", " + wp.yaw.toFixed(2) + "); // " + wp.name + "\n";
            }
            return code;
        }

        function parseCode(text) {
            // Simple parser for demonstration
            console.log("Parsing code: " + text);
            // TODO: Implement actual parsing logic
        }
    }

    // Disconnection detection
    property var lastDataTimestamp: Date.now()
    property int disconnectTimeout: 2000 // 2 seconds timeout
    property bool isConnected: false

    Timer {
        id: disconnectTimer
        interval: 500 // Check every 500ms
        running: true
        repeat: true
        onTriggered: {
            // Only check if we're connected, in robot mode, AND robot is ready (STANDSTILL or higher)
            // This prevents false disconnects during startup when only logs are being sent
            if (isConnected && !isSimulationMode && robotManager && robotManager.robotReady) {
                var currentTime = Date.now();
                var timeSinceLastData = currentTime - lastDataTimestamp;

                // Only disconnect if timeout exceeded
                if (timeSinceLastData > disconnectTimeout) {
                    console.log("Robot disconnected - no data for", timeSinceLastData, "ms");

                    // Clean up backend
                    robotManager.disconnectRobot();

                    // Reset robot states
                    currentJointPositions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

                    // Reset simulation states
                    var zeroPos = [];
                    for (var k = 0; k < 7; k++)
                        zeroPos.push(0.0);
                    simulationJointPositions = zeroPos;

                    // Reset 3D model
                    var angles = {};
                    for (var i = 0; i < currentLinks.length; i++) {
                        if (currentLinks[i].hasJoint && currentLinks[i].jointName !== "right_finger_joint") {
                            angles[currentLinks[i].jointName] = 0.0;
                        }
                    }
                    jointAngles = angles;

                    isConnected = false;
                    isSimulationMode = true;
                    disconnectPopup.open();
                }
            }
        }
    }

    // Connection settings
    property string selectedDevice: "/dev/ttyACM0"
    property int selectedBaudrate: 1500000
    property var availableBaudrates: [9600, 115200, 1500000, 2812000]

    // Robot detection popup
    Rectangle {
        id: robotFoundPopup
        visible: false
        anchors.fill: parent
        color: "#80000000"
        z: 9999

        Rectangle {
            width: 400
            height: 200
            anchors.centerIn: parent
            color: "#2a2a2a"
            radius: 12
            border.color: "#4CAF50"
            border.width: 2

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 30
                spacing: 20

                Text {
                    text: "Robot Found!"
                    color: "#4CAF50"
                    font.pixelSize: 24
                    font.bold: true
                    Layout.alignment: Qt.AlignHCenter
                }

                Text {
                    id: robotFoundText
                    text: "Robot detected at /dev/ttyACM0"
                    color: "white"
                    font.pixelSize: 16
                    Layout.alignment: Qt.AlignHCenter
                }

                Text {
                    text: "Start homing procedure?"
                    color: "#cccccc"
                    font.pixelSize: 14
                    Layout.alignment: Qt.AlignHCenter
                }

                Row {
                    spacing: 20
                    Layout.alignment: Qt.AlignHCenter

                    Rectangle {
                        width: 120
                        height: 45
                        radius: 22
                        color: "#606060"

                        Text {
                            text: "Cancel"
                            anchors.centerIn: parent
                            color: "white"
                            font.pixelSize: 14
                            font.bold: true
                        }

                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                robotFoundPopup.visible = false;
                            }
                        }
                    }

                    Rectangle {
                        width: 120
                        height: 45
                        radius: 22
                        color: "#4CAF50"

                        Text {
                            text: "Start"
                            anchors.centerIn: parent
                            color: "white"
                            font.pixelSize: 14
                            font.bold: true
                        }

                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                robotFoundPopup.visible = false;
                                // Send CMD_START (2) to initialize robot
                                console.log("Sending CMD_START to initialize");
                                robotManager.sendCommand(2);  // CMD_START
                            }
                        }
                    }
                }
            }
        }
    }

    // Connect to robot detection
    Connections {
        target: robotManager
        function onRobotFound(port) {
            console.log("Robot detected at:", port);
            robotFoundText.text = "Robot detected at " + port;
            selectedDevice = port;
            // Just show popup - don't auto-connect
            robotFoundPopup.visible = true;
        }

        function onJointDataUpdated() {
            // Get joint states from robot
            var states = robotManager.getJointStates();
            if (states.length === 0)
                return;

            // Update connection tracking
            lastDataTimestamp = Date.now();
            if (!isConnected) {
                isConnected = true;
            }

            // Update current joint positions for sliders
            var positions = [];
            for (var i = 0; i < 7; i++) {
                positions[i] = (i < states.length) ? states[i].position : 0.0;
            }
            currentJointPositions = positions;

            // Update 3D model jointAngles in robot mode (not simulation)
            if (!isSimulationMode && currentLinks.length > 0) {
                var angles = {};
                var jointIdx = 0;

                // Map joint states to link joint names
                for (var i = 0; i < currentLinks.length && jointIdx < states.length; i++) {
                    var link = currentLinks[i];
                    if (link.hasJoint && link.jointName !== "right_finger_joint") {
                        angles[link.jointName] = states[jointIdx].position;
                        jointIdx++;
                    }
                }

                jointAngles = angles;

                // Sync gizmo to EE position when robot moves via non-gizmo controls
                // Only sync when:
                // 1. Gizmo is NOT currently being dragged (user not controlling via gizmo)
                // 2. This allows gizmo to track slider/reset movements
                if (!transformGizmo.isDragging) {
                    transformGizmo.hasBeenDragged = false;  // Allow tracking
                    transformGizmo.updatePosition(true);    // Force update
                }
            }

            // Update circular buffers for each joint
            for (var i = 0; i < states.length && i < 7; i++) {
                var jointName = "joint" + i;

                // Initialize arrays if needed
                if (!jointPositionData[jointName]) {
                    jointPositionData[jointName] = [];
                    jointVelocityData[jointName] = [];
                    jointAccelerationData[jointName] = [];
                }

                var posArr = jointPositionData[jointName];
                var velArr = jointVelocityData[jointName];
                var accArr = jointAccelerationData[jointName];

                // Add new data points
                posArr.push(states[i].position);
                velArr.push(states[i].velocity);
                accArr.push(states[i].acceleration);

                // Keep only last maxDataPoints
                if (posArr.length > maxDataPoints)
                    posArr.shift();
                if (velArr.length > maxDataPoints)
                    velArr.shift();
                if (accArr.length > maxDataPoints)
                    accArr.shift();
            }

            dataIndex++;

            // Request repaint of canvases
            positionCanvas.requestPaint();
            velocityCanvas.requestPaint();
            accelerationCanvas.requestPaint();

            // Force text updates
            dataUpdateTrigger++;
        }

        function onLogReceived(level, message) {
            // Forward MCU log messages to the logs panel
            logsPanelRight.addLog(level, message);

            // IMPORTANT: Log packets also count as "data" - keep connection alive
            // This prevents false disconnects during motor initialization
            lastDataTimestamp = Date.now();
        }
    }

    RobotNode {
        id: robotNode
        onLinksChanged: {
            console.log("Links changed signal received. Count:", links.length);
            currentLinks = links;
        }
    }

    // Motion Controller for executing waypoint sequences
    MotionController {
        id: motionController

        onWaypointReached: index => {
            console.log("[MotionController] Reached waypoint", index);
        }
        onExecutionComplete: {
            console.log("[MotionController] Motion sequence complete");
        }
        onError: message => {
            console.log("[MotionController] Error:", message);
        }
    }

    // Delayed initialization for MotionController dependencies
    // Timer is separate because C++ QObjects don't have default property for children
    Timer {
        id: motionControllerInitTimer
        interval: 100
        repeat: false
        running: true
        onTriggered: {
            console.log("[MotionController] Delayed init - robotNode:", robotNode);
            console.log("[MotionController] Delayed init - robotManager:", robotManager);
            if (robotNode) {
                motionController.robotNode = robotNode;
            }
            if (robotManager) {
                motionController.robotManager = robotManager;
            } else {
                console.log("[MotionController] robotManager still null, retrying...");
                motionControllerInitTimer.start();  // Retry
            }
        }
    }

    // Disconnection Popup
    Rectangle {
        id: disconnectPopup
        visible: false
        anchors.fill: parent
        color: "#80000000"
        z: 9999

        function open() {
            visible = true;
        }

        function close() {
            visible = false;
        }

        Rectangle {
            width: 400
            height: 200
            anchors.centerIn: parent
            color: "#2a2a2a"
            radius: 10
            border.color: "#FF5722"
            border.width: 2

            Column {
                anchors.centerIn: parent
                spacing: 20

                Text {
                    text: "⚠ Robot Disconnected"
                    color: "#FF5722"
                    font.pixelSize: 20
                    font.bold: true
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Text {
                    text: "Connection to robot lost.\nSwitched to Simulation mode."
                    color: "#ffffff"
                    font.pixelSize: 14
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Rectangle {
                    width: 100
                    height: 40
                    radius: 20
                    color: "#4CAF50"
                    anchors.horizontalCenter: parent.horizontalCenter

                    Text {
                        text: "OK"
                        anchors.centerIn: parent
                        color: "white"
                        font.pixelSize: 14
                        font.bold: true
                    }

                    MouseArea {
                        anchors.fill: parent
                        cursorShape: Qt.PointingHandCursor
                        onClicked: {
                            disconnectPopup.close();
                        }
                    }
                }
            }
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // Header / TabBar Container
        Item {
            Layout.fillWidth: true
            height: 60
            z: 10 // Ensure it's on top

            // Glass Tab Bar (Compact & Floating)
            Rectangle {
                width: bar.implicitWidth + 24
                height: 48
                anchors.top: parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.topMargin: 16

                color: "#aa1e1e1e"
                radius: 24
                border.color: "#30ffffff"
                border.width: 1

                TabBar {
                    id: bar
                    anchors.fill: parent
                    anchors.margins: 6
                    spacing: 4

                    background: Rectangle {
                        color: "transparent"
                    }

                    Repeater {
                        model: ["Control", "Graphs", "Settings", "Driver Settings"]

                        TabButton {
                            id: tabBtn
                            width: 110
                            height: parent.height

                            contentItem: Text {
                                text: modelData
                                font.family: "Segoe UI"
                                font.pixelSize: 14
                                font.bold: true
                                color: tabBtn.checked ? "#ffffff" : "#80ffffff"
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }

                            background: Rectangle {
                                color: tabBtn.checked ? "#40ffffff" : "transparent"
                                radius: 18
                            }
                        }
                    }
                }
            }
        }

        // Main content area: Row with tabs on left and logs panel on right
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 0

            // Tab content
            StackLayout {
                currentIndex: bar.currentIndex
                Layout.fillWidth: true
                Layout.fillHeight: true

                // Tab 1: Control (3D View + Sidebar)
                Item {
                    anchors.fill: parent

                    // 1. Sidebar (Control Panel) - Left Side
                    Rectangle {
                        id: controlPanel
                        width: 500
                        anchors.left: parent.left
                        anchors.top: parent.top
                        anchors.bottom: parent.bottom
                        color: "#1e1e1e" // Solid dark sidebar (not glass anymore)
                        z: 100 // Ensure sidebar is well above 3D view

                        // Divider line
                        Rectangle {
                            anchors.right: parent.right
                            anchors.top: parent.top
                            anchors.bottom: parent.bottom
                            width: 1
                            color: "#30ffffff"
                        }

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 20
                            spacing: 15

                            // Tab Bar with Glass Effect
                            TabBar {
                                id: controlTabBar
                                Layout.fillWidth: true
                                Layout.preferredHeight: 45
                                background: Rectangle {
                                    color: "#aa202020"
                                    border.color: "#30ffffff"
                                    border.width: 1
                                    radius: 8
                                }

                                TabButton {
                                    text: "Joint Control"
                                    font.pixelSize: 13
                                    font.bold: true
                                    background: Rectangle {
                                        color: controlTabBar.currentIndex === 0 ? "#aa4CAF50" : "transparent"
                                        radius: 6
                                        border.color: controlTabBar.currentIndex === 0 ? "#66BB6A" : "transparent"
                                        border.width: 1
                                    }
                                    contentItem: Text {
                                        text: parent.text
                                        font: parent.font
                                        color: controlTabBar.currentIndex === 0 ? "#ffffff" : "#888888"
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                        leftPadding: 15
                                        rightPadding: 15
                                    }
                                }

                                TabButton {
                                    text: "Cartesian Control"
                                    font.pixelSize: 13
                                    font.bold: true
                                    background: Rectangle {
                                        color: controlTabBar.currentIndex === 1 ? "#aa2196F3" : "transparent"
                                        radius: 6
                                        border.color: controlTabBar.currentIndex === 1 ? "#42A5F5" : "transparent"
                                        border.width: 1
                                    }
                                    contentItem: Text {
                                        text: parent.text
                                        font: parent.font
                                        color: controlTabBar.currentIndex === 1 ? "#ffffff" : "#888888"
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                        leftPadding: 15
                                        rightPadding: 15
                                    }
                                }
                            }

                            // Content Stack
                            StackLayout {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                currentIndex: controlTabBar.currentIndex

                                // Tab 0: Joint Control - Using extracted component
                                Panels.JointControlPanel {
                                    // robotManager accessed directly from C++ context
                                    currentLinks: window.currentLinks
                                    isSimulationMode: window.isSimulationMode
                                    simulationJointPositions: window.simulationJointPositions
                                    currentJointPositions: window.currentJointPositions
                                    jointAngles: window.jointAngles
                                    jointColors: window.jointColors
                                    useRadians: window.useRadians

                                    onSimulationPositionsChanged: function (positions) {
                                        window.simulationJointPositions = positions;
                                    }
                                    onJointAnglesUpdated: function (angles) {
                                        window.jointAngles = angles;
                                    }
                                    onResetAllJoints: {
                                        if (window.isSimulationMode) {
                                            window.simulationJointPositions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
                                            window.jointAngles = {};
                                            window.resetGizmo();
                                        } else {
                                            robotManager.resetAllJoints();  // Use context property directly
                                        }
                                    }
                                    onGizmoUpdateRequested: {
                                        if (typeof transformGizmo !== 'undefined' && transformGizmo) {
                                            transformGizmo.updatePosition(true);
                                        }
                                    }
                                }
                                // Tab 1: Cartesian Control
                                Item {
                                    ColumnLayout {
                                        anchors.fill: parent
                                        anchors.margins: 15
                                        spacing: 15

                                        // Frame Toggle
                                        Rectangle {
                                            Layout.fillWidth: true
                                            Layout.preferredHeight: 50
                                            color: "#2a2a2a"
                                            radius: 25
                                            border.color: "#404040"
                                            border.width: 1

                                            Row {
                                                anchors.centerIn: parent
                                                spacing: 10

                                                Text {
                                                    text: "Reference Frame:"
                                                    color: "#cccccc"
                                                    font.pixelSize: 11
                                                    font.bold: true
                                                    anchors.verticalCenter: parent.verticalCenter
                                                }

                                                Rectangle {
                                                    width: 180
                                                    height: 32
                                                    radius: 16
                                                    color: "#303030"
                                                    border.color: "#505050"
                                                    border.width: 1

                                                    Row {
                                                        anchors.fill: parent
                                                        spacing: 0

                                                        Rectangle {
                                                            width: parent.width / 2
                                                            height: parent.height
                                                            radius: 16
                                                            color: "#2196F3"
                                                            border.color: "#42A5F5"
                                                            border.width: 1

                                                            Text {
                                                                anchors.centerIn: parent
                                                                text: "World"
                                                                color: "white"
                                                                font.pixelSize: 11
                                                                font.bold: true
                                                            }

                                                            MouseArea {
                                                                anchors.fill: parent
                                                                cursorShape: Qt.PointingHandCursor
                                                                onClicked: console.log("World frame selected")
                                                            }
                                                        }

                                                        Rectangle {
                                                            width: parent.width / 2
                                                            height: parent.height
                                                            radius: 16
                                                            color: "transparent"

                                                            Text {
                                                                anchors.centerIn: parent
                                                                text: "Tool"
                                                                color: "#888888"
                                                                font.pixelSize: 11
                                                                font.bold: true
                                                            }

                                                            MouseArea {
                                                                anchors.fill: parent
                                                                cursorShape: Qt.PointingHandCursor
                                                                onClicked: console.log("Tool frame selected")
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }

                                        // Jog Velocity Selector
                                        Rectangle {
                                            Layout.fillWidth: true
                                            Layout.preferredHeight: 60
                                            color: "#2a2a2a"
                                            radius: 25
                                            border.color: "#404040"
                                            border.width: 1

                                            ColumnLayout {
                                                anchors.fill: parent
                                                anchors.margins: 10
                                                spacing: 5

                                                Text {
                                                    text: "Jog Speed (mm/s)"
                                                    color: "#cccccc"
                                                    font.pixelSize: 11
                                                    font.bold: true
                                                }

                                                Row {
                                                    spacing: 8

                                                    Repeater {
                                                        model: jogSpeedLabels
                                                        Rectangle {
                                                            width: 70
                                                            height: 28
                                                            radius: 14
                                                            color: jogSpeedIndex === index ? "#4CAF50" : "#404040"
                                                            border.color: jogSpeedIndex === index ? "#66BB6A" : "#606060"
                                                            border.width: 1

                                                            Text {
                                                                anchors.centerIn: parent
                                                                text: modelData
                                                                color: jogSpeedIndex === index ? "#ffffff" : "#888888"
                                                                font.pixelSize: 10
                                                                font.bold: jogSpeedIndex === index
                                                            }

                                                            MouseArea {
                                                                anchors.fill: parent
                                                                cursorShape: Qt.PointingHandCursor
                                                                onClicked: {
                                                                    jogSpeedIndex = index;
                                                                    console.log("Jog speed:", jogSpeedLabels[index], "mm/s");
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }

                                        // Translation Controls
                                        Rectangle {
                                            Layout.fillWidth: true
                                            Layout.preferredHeight: 260
                                            color: "#aa202020"
                                            radius: 8
                                            border.color: "#30ffffff"
                                            border.width: 1

                                            ColumnLayout {
                                                anchors.fill: parent
                                                anchors.margins: 15
                                                spacing: 10

                                                Text {
                                                    text: "Translation (Position)"
                                                    color: "#ffffff"
                                                    font.pixelSize: 13
                                                    font.bold: true
                                                    Layout.alignment: Qt.AlignHCenter
                                                }

                                                // Current Position Values
                                                Row {
                                                    Layout.alignment: Qt.AlignHCenter
                                                    spacing: 20

                                                    Text {
                                                        text: "X: " + (transformGizmo.gizmoPosition.z / 1000).toFixed(3) + " m"
                                                        color: "#ef5350"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        font.family: "Monospace"
                                                    }
                                                    Text {
                                                        text: "Y: " + (transformGizmo.gizmoPosition.y / 1000).toFixed(3) + " m"
                                                        color: "#66bb6a"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        font.family: "Monospace"
                                                    }
                                                    Text {
                                                        text: "Z: " + (transformGizmo.gizmoPosition.x / 1000).toFixed(3) + " m"
                                                        color: "#42a5f5"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        font.family: "Monospace"
                                                    }
                                                }

                                                // X Axis (Red) - Left/Right
                                                Row {
                                                    Layout.alignment: Qt.AlignHCenter
                                                    spacing: 15

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#c62828"
                                                        border.color: "#e53935"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "◄"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "X-"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#b71c1c";
                                                                xMinusAction();
                                                                xMinusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#c62828";
                                                                xMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#c62828";
                                                                xMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function xMinusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    // VELOCITY MODE: Send constant Cartesian velocity in URDF X- direction
                                                                    // Button X- = URDF X- (scene Z-)
                                                                    window.sendCartesianVelocity(Qt.vector3d(-window.cartesianJogSpeed, 0, 0)  // Linear velocity
                                                                    , Qt.vector3d(0, 0, 0)   // Angular velocity
                                                                    );
                                                                } else {
                                                                    // POSITION MODE: Step gizmo and solve IK

                                                                    var pos = transformGizmo.gizmoPosition;
                                                                    transformGizmo.gizmoPosition = Qt.vector3d(pos.x, pos.y, pos.z - step);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: xMinusTimer
                                                                interval: 50  // Faster for velocity mode
                                                                repeat: true
                                                                onTriggered: parent.xMinusAction()
                                                            }
                                                        }
                                                    }

                                                    Text {
                                                        text: "X-Axis"
                                                        color: "#ef5350"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#c62828"
                                                        border.color: "#e53935"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "►"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "X+"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#b71c1c";
                                                                xPlusAction();
                                                                xPlusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#c62828";
                                                                xPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#c62828";
                                                                xPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function xPlusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    window.sendCartesianVelocity(Qt.vector3d(window.cartesianJogSpeed, 0, 0), Qt.vector3d(0, 0, 0));
                                                                } else {
                                                                    var pos = transformGizmo.gizmoPosition;
                                                                    transformGizmo.gizmoPosition = Qt.vector3d(pos.x, pos.y, pos.z + step);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: xPlusTimer
                                                                interval: 50
                                                                repeat: true
                                                                onTriggered: parent.xPlusAction()
                                                            }
                                                        }
                                                    }
                                                }

                                                // Y Axis (Green) - Forward/Backward
                                                Row {
                                                    Layout.alignment: Qt.AlignHCenter
                                                    spacing: 15

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#2e7d32"
                                                        border.color: "#43a047"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "▼"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "Y-"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#1b5e20";
                                                                yMinusAction();
                                                                yMinusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#2e7d32";
                                                                yMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#2e7d32";
                                                                yMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function yMinusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    // URDF Y- = scene Z+ (after transform)
                                                                    window.sendCartesianVelocity(Qt.vector3d(0, -window.cartesianJogSpeed, 0), Qt.vector3d(0, 0, 0));
                                                                } else {
                                                                    var pos = transformGizmo.gizmoPosition;
                                                                    transformGizmo.gizmoPosition = Qt.vector3d(pos.x, pos.y - step, pos.z);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: yMinusTimer
                                                                interval: 50
                                                                repeat: true
                                                                onTriggered: parent.yMinusAction()
                                                            }
                                                        }
                                                    }

                                                    Text {
                                                        text: "Y-Axis"
                                                        color: "#66bb6a"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#2e7d32"
                                                        border.color: "#43a047"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "▲"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "Y+"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#1b5e20";
                                                                yPlusAction();
                                                                yPlusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#2e7d32";
                                                                yPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#2e7d32";
                                                                yPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function yPlusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    window.sendCartesianVelocity(Qt.vector3d(0, window.cartesianJogSpeed, 0), Qt.vector3d(0, 0, 0));
                                                                } else {
                                                                    var pos = transformGizmo.gizmoPosition;
                                                                    transformGizmo.gizmoPosition = Qt.vector3d(pos.x, pos.y + step, pos.z);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: yPlusTimer
                                                                interval: 50
                                                                repeat: true
                                                                onTriggered: parent.yPlusAction()
                                                            }
                                                        }
                                                    }
                                                }

                                                // Z Axis (Blue) - Up/Down
                                                Row {
                                                    Layout.alignment: Qt.AlignHCenter
                                                    spacing: 15

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#1565c0"
                                                        border.color: "#1e88e5"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "↓"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "Z-"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#0d47a1";
                                                                zMinusAction();
                                                                zMinusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#1565c0";
                                                                zMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#1565c0";
                                                                zMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function zMinusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    // URDF Z- (down)
                                                                    window.sendCartesianVelocity(Qt.vector3d(0, 0, -window.cartesianJogSpeed), Qt.vector3d(0, 0, 0));
                                                                } else {
                                                                    var pos = transformGizmo.gizmoPosition;
                                                                    transformGizmo.gizmoPosition = Qt.vector3d(pos.x - step, pos.y, pos.z);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: zMinusTimer
                                                                interval: 50
                                                                repeat: true
                                                                onTriggered: parent.zMinusAction()
                                                            }
                                                        }
                                                    }

                                                    Text {
                                                        text: "Z-Axis"
                                                        color: "#42a5f5"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#1565c0"
                                                        border.color: "#1e88e5"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "↑"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "Z+"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#0d47a1";
                                                                zPlusAction();
                                                                zPlusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#1565c0";
                                                                zPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#1565c0";
                                                                zPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function zPlusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    // URDF Z+ (up)
                                                                    window.sendCartesianVelocity(Qt.vector3d(0, 0, window.cartesianJogSpeed), Qt.vector3d(0, 0, 0));
                                                                } else {
                                                                    var pos = transformGizmo.gizmoPosition;
                                                                    transformGizmo.gizmoPosition = Qt.vector3d(pos.x + step, pos.y, pos.z);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: zPlusTimer
                                                                interval: 50
                                                                repeat: true
                                                                onTriggered: parent.zPlusAction()
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }

                                        // Rotation Controls
                                        Rectangle {
                                            Layout.fillWidth: true
                                            Layout.preferredHeight: 260
                                            color: "#aa202020"
                                            radius: 8
                                            border.color: "#30ffffff"
                                            border.width: 1

                                            ColumnLayout {
                                                anchors.fill: parent
                                                anchors.margins: 15
                                                spacing: 10

                                                Text {
                                                    text: "Rotation (Orientation)"
                                                    color: "#ffffff"
                                                    font.pixelSize: 13
                                                    font.bold: true
                                                    Layout.alignment: Qt.AlignHCenter
                                                }

                                                // Current Rotation Values
                                                Row {
                                                    Layout.alignment: Qt.AlignHCenter
                                                    spacing: 15

                                                    Text {
                                                        text: "Roll: " + transformGizmo.gizmoEulerRotation.x.toFixed(2) + "°"
                                                        color: "#ef5350"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        font.family: "Monospace"
                                                    }
                                                    Text {
                                                        text: "Pitch: " + transformGizmo.gizmoEulerRotation.y.toFixed(2) + "°"
                                                        color: "#66bb6a"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        font.family: "Monospace"
                                                    }
                                                    Text {
                                                        text: "Yaw: " + transformGizmo.gizmoEulerRotation.z.toFixed(2) + "°"
                                                        color: "#42a5f5"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        font.family: "Monospace"
                                                    }
                                                }

                                                // Roll (Red) - Rotate around X
                                                Row {
                                                    Layout.alignment: Qt.AlignHCenter
                                                    spacing: 15

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#c62828"
                                                        border.color: "#e53935"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "↶"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "Roll-"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#b71c1c";
                                                                rollMinusAction();
                                                                rollMinusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#c62828";
                                                                rollMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#c62828";
                                                                rollMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function rollMinusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    // URDF Roll- (X-axis rotation)
                                                                    window.sendCartesianVelocity(Qt.vector3d(0, 0, 0), Qt.vector3d(-window.rotationJogSpeed, 0, 0));
                                                                } else {
                                                                    var rot = transformGizmo.gizmoEulerRotation;
                                                                    transformGizmo.gizmoEulerRotation = Qt.vector3d(rot.x - rotationStepSize, rot.y, rot.z);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: rollMinusTimer
                                                                interval: 50
                                                                repeat: true
                                                                onTriggered: parent.rollMinusAction()
                                                            }
                                                        }
                                                    }

                                                    Text {
                                                        text: "Roll (X)"
                                                        color: "#ef5350"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#c62828"
                                                        border.color: "#e53935"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "↷"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "Roll+"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#b71c1c";
                                                                rollPlusAction();
                                                                rollPlusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#c62828";
                                                                rollPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#c62828";
                                                                rollPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function rollPlusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    window.sendCartesianVelocity(Qt.vector3d(0, 0, 0), Qt.vector3d(window.rotationJogSpeed, 0, 0));
                                                                } else {
                                                                    var rot = transformGizmo.gizmoEulerRotation;
                                                                    transformGizmo.gizmoEulerRotation = Qt.vector3d(rot.x + rotationStepSize, rot.y, rot.z);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: rollPlusTimer
                                                                interval: 50
                                                                repeat: true
                                                                onTriggered: parent.rollPlusAction()
                                                            }
                                                        }
                                                    }
                                                }

                                                // Pitch (Green) - Rotate around Y
                                                Row {
                                                    Layout.alignment: Qt.AlignHCenter
                                                    spacing: 15

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#2e7d32"
                                                        border.color: "#43a047"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "⟲"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "Pitch-"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#1b5e20";
                                                                pitchMinusAction();
                                                                pitchMinusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#2e7d32";
                                                                pitchMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#2e7d32";
                                                                pitchMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function pitchMinusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    window.sendCartesianVelocity(Qt.vector3d(0, 0, 0), Qt.vector3d(0, -window.rotationJogSpeed, 0));
                                                                } else {
                                                                    var rot = transformGizmo.gizmoEulerRotation;
                                                                    transformGizmo.gizmoEulerRotation = Qt.vector3d(rot.x, rot.y - rotationStepSize, rot.z);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: pitchMinusTimer
                                                                interval: 50
                                                                repeat: true
                                                                onTriggered: parent.pitchMinusAction()
                                                            }
                                                        }
                                                    }

                                                    Text {
                                                        text: "Pitch (Y)"
                                                        color: "#66bb6a"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#2e7d32"
                                                        border.color: "#43a047"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "⟳"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "Pitch+"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#1b5e20";
                                                                pitchPlusAction();
                                                                pitchPlusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#2e7d32";
                                                                pitchPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#2e7d32";
                                                                pitchPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function pitchPlusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    window.sendCartesianVelocity(Qt.vector3d(0, 0, 0), Qt.vector3d(0, window.rotationJogSpeed, 0));
                                                                } else {
                                                                    var rot = transformGizmo.gizmoEulerRotation;
                                                                    transformGizmo.gizmoEulerRotation = Qt.vector3d(rot.x, rot.y + rotationStepSize, rot.z);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: pitchPlusTimer
                                                                interval: 50
                                                                repeat: true
                                                                onTriggered: parent.pitchPlusAction()
                                                            }
                                                        }
                                                    }
                                                }

                                                // Yaw (Blue) - Rotate around Z
                                                Row {
                                                    Layout.alignment: Qt.AlignHCenter
                                                    spacing: 15

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#1565c0"
                                                        border.color: "#1e88e5"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "⤹"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "Yaw-"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#0d47a1";
                                                                yawMinusAction();
                                                                yawMinusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#1565c0";
                                                                yawMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#1565c0";
                                                                yawMinusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function yawMinusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    window.sendCartesianVelocity(Qt.vector3d(0, 0, 0), Qt.vector3d(0, 0, -window.rotationJogSpeed));
                                                                } else {
                                                                    var rot = transformGizmo.gizmoEulerRotation;
                                                                    transformGizmo.gizmoEulerRotation = Qt.vector3d(rot.x, rot.y, rot.z - rotationStepSize);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: yawMinusTimer
                                                                interval: 50
                                                                repeat: true
                                                                onTriggered: parent.yawMinusAction()
                                                            }
                                                        }
                                                    }

                                                    Text {
                                                        text: "Yaw (Z)"
                                                        color: "#42a5f5"
                                                        font.pixelSize: 11
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }

                                                    Rectangle {
                                                        width: 50
                                                        height: 50
                                                        radius: 25
                                                        color: "#1565c0"
                                                        border.color: "#1e88e5"
                                                        border.width: 2

                                                        Column {
                                                            anchors.centerIn: parent
                                                            spacing: 2
                                                            Text {
                                                                text: "⤸"
                                                                color: "white"
                                                                font.pixelSize: 20
                                                                font.bold: true
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                            Text {
                                                                text: "Yaw+"
                                                                color: "white"
                                                                font.pixelSize: 9
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                            }
                                                        }

                                                        MouseArea {
                                                            anchors.fill: parent
                                                            cursorShape: Qt.PointingHandCursor
                                                            onPressed: {
                                                                parent.color = "#0d47a1";
                                                                yawPlusAction();
                                                                yawPlusTimer.start();
                                                            }
                                                            onReleased: {
                                                                parent.color = "#1565c0";
                                                                yawPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            onCanceled: {
                                                                parent.color = "#1565c0";
                                                                yawPlusTimer.stop();
                                                                window.stopCartesianVelocity();
                                                            }
                                                            function yawPlusAction() {
                                                                if (!isSimulationMode && useVelocityMode) {
                                                                    window.sendCartesianVelocity(Qt.vector3d(0, 0, 0), Qt.vector3d(0, 0, window.rotationJogSpeed));
                                                                } else {
                                                                    var rot = transformGizmo.gizmoEulerRotation;
                                                                    transformGizmo.gizmoEulerRotation = Qt.vector3d(rot.x, rot.y, rot.z + rotationStepSize);
                                                                    transformGizmo.hasBeenDragged = true;
                                                                    window.solveIKFromGizmo();
                                                                }
                                                            }
                                                            Timer {
                                                                id: yawPlusTimer
                                                                interval: 50
                                                                repeat: true
                                                                onTriggered: parent.yawPlusAction()
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }

                                        // Reset Button
                                        Item {
                                            Layout.fillWidth: true
                                            Layout.preferredHeight: 50
                                            Layout.topMargin: 10

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
                                                        if (isSimulationMode) {
                                                            // Simulation mode: just reset local values
                                                            simulationJointPositions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
                                                            jointAngles = {};
                                                            console.log("Reset all joints to 0 (Simulation)");

                                                            // Also update gizmo to new end-effector position (FK at zero)
                                                            window.resetGizmo();
                                                        } else {
                                                            // Robot mode: send zero command to all joints at once
                                                            robotManager.resetAllJoints();
                                                            // Reset gizmo to track EE position as robot moves
                                                            transformGizmo.hasBeenDragged = false;
                                                            robotGizmoHasBeenDragged = false;
                                                            transformGizmo.updatePosition(true);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        Item {
                                            Layout.fillHeight: true
                                        }
                                    }
                                }
                                // Motion Planner tab moved to right panel
                            }
                        }
                    }

                    // 2. 3D View Container - Right Side
                    Item {
                        id: viewContainer
                        anchors.left: controlPanel.right
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.bottom: parent.bottom

                        View3D {
                            id: view3d
                            anchors.fill: parent

                            environment: SceneEnvironment {
                                clearColor: "#121212"
                                backgroundMode: SceneEnvironment.Color
                                antialiasingMode: SceneEnvironment.MSAA
                                antialiasingQuality: SceneEnvironment.High
                            }

                            // Picking logic
                            MouseArea {
                                anchors.fill: parent
                                onClicked: mouse => {
                                    // Perform pick
                                    var result = view3d.pick(mouse.x, mouse.y);
                                    if (result.objectHit) {
                                        var pickedObject = result.objectHit;
                                        // Walk up to find the WaypointMarkers delegate root or identify by property
                                        // Since we are using Repeater3D, the picked object is the Model.
                                        // We need to access the model index.
                                        // The Repeater3D delegate has the context of 'index' and 'modelData'.
                                        // We can try to map the picked object to the index.

                                        // Helper: traversing up to find if we hit a waypoint marker
                                        var current = pickedObject;
                                        while (current && current !== view3d) {
                                            // Check if this node has our specific "waypointMarkerNode" id or property
                                            // In Qt 6, we can attach a property to the Node in the delegate
                                            if (current.isWaypoint) {
                                                console.log("Picked waypoint:", current.index);
                                                window.waypointModel.selectWaypoint(current.index);
                                                return;
                                            }
                                            current = current.parent;
                                        }
                                    }

                                // Deselect if clicked background? Optional.
                                }
                            }

                            // Camera Rig
                            Node {
                                id: cameraPivot
                                position: Qt.vector3d(0, 50, 0)

                                Node {
                                    id: cameraRotationNode
                                    eulerRotation.x: -20
                                    eulerRotation.y: 45

                                    // Property to control when animation is active (for smooth axis button transitions)
                                    property bool animateCamera: false

                                    // Timer to reset animateCamera after animation completes
                                    Timer {
                                        id: resetAnimateCameraTimer
                                        interval: 450  // Slightly longer than animation duration (400ms)
                                        onTriggered: cameraRotationNode.animateCamera = false
                                    }

                                    Behavior on eulerRotation.x {
                                        enabled: cameraRotationNode.animateCamera
                                        NumberAnimation {
                                            duration: 400
                                            easing.type: Easing.OutCubic
                                        }
                                    }

                                    Behavior on eulerRotation.y {
                                        enabled: cameraRotationNode.animateCamera
                                        NumberAnimation {
                                            duration: 400
                                            easing.type: Easing.OutCubic
                                        }
                                    }

                                    PerspectiveCamera {
                                        id: camera
                                        position: Qt.vector3d(0, 0, cameraDistance)
                                        clipNear: 1.0
                                        clipFar: 10000.0
                                    }
                                }
                            }

                            DirectionalLight {
                                eulerRotation.x: -30
                                eulerRotation.y: -30
                                brightness: 1.5
                            }
                            DirectionalLight {
                                eulerRotation.x: 30
                                eulerRotation.y: 150
                                brightness: 1.5
                            }
                            PointLight {
                                position: Qt.vector3d(0, 300, 0)
                                brightness: 2.0
                            }

                            // Ground plane for reference
                            Model {
                                source: "#Rectangle"
                                scale: Qt.vector3d(50, 50, 1)
                                position: Qt.vector3d(0, 0, 0)
                                eulerRotation.x: -90
                                materials: DefaultMaterial {
                                    diffuseColor: "#1a1a1a"
                                    lighting: DefaultMaterial.NoLighting
                                }
                            }

                            WaypointMarkers {
                                waypointData: window.waypointModel.internalModel
                            }

                            Node {
                                id: robotRoot
                                scale: Qt.vector3d(100, 100, 100)
                                eulerRotation.x: -90

                                Repeater3D {
                                    model: currentLinks

                                    Node {
                                        id: linkNode
                                        objectName: modelData.name

                                        // Position - DEBUG: Override with collision checker's transforms if enabled
                                        position: {
                                            // DEBUG MODE: Use hardcoded collision transforms
                                            if (useDebugCollisionTransforms && debugCollisionTransforms.hasOwnProperty(modelData.name)) {
                                                var dbgT = debugCollisionTransforms[modelData.name];
                                                // Scale by 100 (robotRoot scale) - but transforms are already in world space
                                                return Qt.vector3d(dbgT.pos.x, dbgT.pos.y, dbgT.pos.z);
                                            }

                                            var basePos = Qt.vector3d(modelData.position.x, modelData.position.y, modelData.position.z);

                                            // Apply translation for prismatic joints (works in both modes)
                                            if (modelData.hasJoint) {
                                                // Check if this is a mimic joint (right_finger mimics left_finger)
                                                var jointName = modelData.jointName;
                                                var jointValue = 0;

                                                if (jointName === "right_finger_joint") {
                                                    // Use left_finger value
                                                    jointValue = jointAngles["left_finger_joint"] || 0;
                                                } else {
                                                    jointValue = jointAngles[jointName] || 0;
                                                }

                                                // PRISMATIC joint type from URDF enum
                                                if (modelData.jointType === 3 && modelData.jointAxis && jointValue !== 0) {
                                                    var axis = Qt.vector3d(modelData.jointAxis.x, modelData.jointAxis.y, modelData.jointAxis.z);

                                                    // Get the parent joint's rotation to transform axis to parent frame
                                                    var parentRot = Qt.quaternion(modelData.rotation.scalar, modelData.rotation.x, modelData.rotation.y, modelData.rotation.z);

                                                    // Rotate the axis by the parent rotation to get it in parent's coordinate frame
                                                    var translationVec = Qt.vector3d(axis.x * jointValue, axis.y * jointValue, axis.z * jointValue);

                                                    // Transform translation vector by parent rotation
                                                    var rotatedTranslation = parentRot.times(translationVec);

                                                    // Apply translation
                                                    return Qt.vector3d(basePos.x + rotatedTranslation.x, basePos.y + rotatedTranslation.y, basePos.z + rotatedTranslation.z);
                                                }
                                            }

                                            return basePos;
                                        }

                                        rotation: {
                                            // DEBUG MODE: Use hardcoded collision rotation
                                            if (useDebugCollisionTransforms && debugCollisionTransforms.hasOwnProperty(modelData.name)) {
                                                var dbgT = debugCollisionTransforms[modelData.name];
                                                return dbgT.quat;
                                            }

                                            // Base rotation from URDF
                                            var baseRot = Qt.quaternion(modelData.rotation.scalar, modelData.rotation.x, modelData.rotation.y, modelData.rotation.z);

                                            // Apply joint rotation for REVOLUTE joints (works in both modes)
                                            if (modelData.hasJoint) {
                                                if (modelData.jointType !== 3 && modelData.jointAxis) {  // NOT prismatic
                                                    var jointAngle = jointAngles[modelData.jointName] || 0;
                                                    var axis = Qt.vector3d(modelData.jointAxis.x, modelData.jointAxis.y, modelData.jointAxis.z);
                                                    // Create rotation quaternion from joint axis and angle
                                                    var jointRot = quaternionFromAxisAngle(axis, jointAngle);
                                                    // Combine base rotation with joint rotation
                                                    return multiplyQuaternions(baseRot, jointRot);
                                                }
                                            }

                                            return baseRot;
                                        }

                                        // Visual XYZ Axes
                                        Node {
                                            visible: {
                                                if (window.showAxesMode === 0)
                                                    return false;
                                                if (window.showAxesMode === 1)
                                                    return modelData.name === "ee_link";
                                                return true; // showAxesMode === 2
                                            }
                                            Model {
                                                position: Qt.vector3d(0.00125, 0, 0)
                                                scale: Qt.vector3d(0.0025, 0.000033, 0.000033)
                                                source: "#Cube"
                                                materials: DefaultMaterial {
                                                    diffuseColor: "#ff4444"
                                                    lighting: DefaultMaterial.NoLighting
                                                }
                                            }
                                            Model {
                                                position: Qt.vector3d(0, 0.00125, 0)
                                                scale: Qt.vector3d(0.000033, 0.0025, 0.000033)
                                                source: "#Cube"
                                                materials: DefaultMaterial {
                                                    diffuseColor: "#44ff44"
                                                    lighting: DefaultMaterial.NoLighting
                                                }
                                            }
                                            Model {
                                                position: Qt.vector3d(0, 0, 0.00125)
                                                scale: Qt.vector3d(0.000033, 0.000033, 0.0025)
                                                source: "#Cube"
                                                materials: DefaultMaterial {
                                                    diffuseColor: "#4444ff"
                                                    lighting: DefaultMaterial.NoLighting
                                                }
                                            }
                                        }

                                        // Visual Mesh
                                        Model {
                                            id: visualMesh
                                            source: modelData.meshSource || ""
                                            scale: modelData.scale || Qt.vector3d(1, 1, 1)
                                            position: modelData.visualPosition ? Qt.vector3d(modelData.visualPosition.x, modelData.visualPosition.y, modelData.visualPosition.z) : Qt.vector3d(0, 0, 0)
                                            rotation: modelData.visualRotation ? Qt.quaternion(modelData.visualRotation.scalar, modelData.visualRotation.x, modelData.visualRotation.y, modelData.visualRotation.z) : Qt.quaternion(1, 0, 0, 0)

                                            // Collision-aware material color
                                            property string linkName: modelData.name || ""
                                            property var collisionColor: getCollisionColor(linkName)

                                            materials: DefaultMaterial {
                                                diffuseColor: visualMesh.collisionColor || "#aaaaaa"
                                                specularAmount: 0.5
                                            }
                                            visible: source != ""
                                        }

                                        Node {
                                            id: jointNode
                                        }

                                        Component.onCompleted: {
                                            if (modelData.parentName !== "") {
                                                var parentNode = findNode(robotRoot, modelData.parentName);
                                                if (parentNode) {
                                                    parent = parentNode;
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            // =====================================================
                            // EE POSITION INDICATOR - Shows actual robot EE position in robot mode
                            // =====================================================
                            Model {
                                id: eeIndicatorSphere
                                visible: !window.isSimulationMode  // Only show in robot mode
                                source: "#Sphere"
                                scale: Qt.vector3d(0.015, 0.015, 0.015)  // 15mm diameter sphere

                                // Position follows ee_link
                                position: {
                                    var eeLinkNode = findNode(robotRoot, "ee_link");
                                    if (eeLinkNode) {
                                        return eeLinkNode.scenePosition;
                                    }
                                    return Qt.vector3d(0, 0, 0);
                                }

                                materials: [
                                    PrincipledMaterial {
                                        baseColor: "#00FF00"  // Bright green
                                        metalness: 0.3
                                        roughness: 0.2
                                    }
                                ]
                            }

                            // =====================================================
                            // 3D TRANSFORM GIZMO - Interactive, Positioned at ee_link
                            // =====================================================
                            Node {
                                id: transformGizmo
                                visible: window.showGizmo  // Show in both simulation and robot modes

                                // Gizmo's own position (can be moved by dragging)
                                property vector3d gizmoPosition: Qt.vector3d(0, 0, 0)
                                property vector3d gizmoEulerRotation: Qt.vector3d(0, 0, 0)  // Euler rotation in degrees
                                property bool isDragging: false
                                property bool hasBeenDragged: false  // Once dragged, don't snap back
                                property string dragAxis: ""  // "x", "y", "z", "free", "rx", "ry", "rz"
                                property point dragStartPos: Qt.point(0, 0)
                                property vector3d dragStartGizmoPos: Qt.vector3d(0, 0, 0)

                                // Smooth animation for gizmo position (triggered by button presses)
                                Behavior on gizmoPosition {
                                    enabled: !transformGizmo.isDragging  // Disable during drag for responsiveness
                                    Vector3dAnimation {
                                        duration: 80
                                        easing.type: Easing.OutCubic
                                    }
                                }

                                // Smooth animation for gizmo rotation (triggered by button presses)
                                Behavior on gizmoEulerRotation {
                                    enabled: !transformGizmo.isDragging
                                    Vector3dAnimation {
                                        duration: 80
                                        easing.type: Easing.OutCubic
                                    }
                                }

                                // Track ee_link when not dragging
                                property var eeLinkNode: null

                                function updatePosition(forceUpdate) {
                                    // Only auto-update if not dragging AND (hasn't been manually dragged OR forceUpdate)
                                    if (!isDragging && (!hasBeenDragged || forceUpdate === true)) {
                                        eeLinkNode = findNode(robotRoot, "ee_link");
                                        if (eeLinkNode) {
                                            gizmoPosition = eeLinkNode.scenePosition;
                                            rotation = eeLinkNode.sceneRotation;
                                            // Sync euler angles from scene rotation for button controls
                                            gizmoEulerRotation = window.quaternionToEuler(eeLinkNode.sceneRotation);
                                        }
                                    }
                                }

                                position: gizmoPosition

                                Timer {
                                    running: transformGizmo.visible
                                    repeat: true
                                    interval: 50
                                    onTriggered: transformGizmo.updatePosition()
                                }

                                // CONTROL LOOP TIMER: Continuously call handleGizmoIK while dragging
                                // This ensures the robot converges to target even when mouse is stationary
                                Timer {
                                    id: controlLoopTimer
                                    running: transformGizmo.isDragging && !isSimulationMode && useVelocityMode
                                    repeat: true
                                    interval: 20  // 50Hz control rate
                                    onTriggered: {
                                        window.handleGizmoIK();
                                    }
                                }

                                Component.onCompleted: updatePosition()

                                // Gizmo scale - 2x bigger now (0.16)
                                property real gizmoSize: 0.32
                                // ---- Local Translation Handles (Local Frame - arrows rotate with EE) ----
                                Node {
                                    id: translationNode
                                    eulerRotation: transformGizmo.gizmoEulerRotation // Inherit rotation for local alignment
                                    // ---- Center Sphere (free movement) ----
                                    Model {
                                        id: centerSphere
                                        objectName: "gizmo_free"
                                        pickable: true
                                        source: "#Sphere"
                                        scale: Qt.vector3d(transformGizmo.gizmoSize * 0.2, transformGizmo.gizmoSize * 0.2, transformGizmo.gizmoSize * 0.2)
                                        materials: DefaultMaterial {
                                            // Orange in simulation mode, magenta in robot mode
                                            diffuseColor: transformGizmo.dragAxis === "free" ? "#ffffff" : (window.isSimulationMode ? "#ffaa00" : "#404040")
                                            lighting: DefaultMaterial.NoLighting
                                        }
                                    }

                                    // ---- X Axis (Red) - Translation Arrow ----
                                    // Arrow shaft
                                    Model {
                                        objectName: "gizmo_x"
                                        pickable: true
                                        source: "#Cylinder"
                                        visible: true
                                        position: Qt.vector3d(transformGizmo.gizmoSize * 1.241, 0, 0)
                                        eulerRotation.z: -90
                                        scale: Qt.vector3d(transformGizmo.gizmoSize * 0.025, transformGizmo.gizmoSize * 0.741, transformGizmo.gizmoSize * 0.025)
                                        materials: DefaultMaterial {
                                            diffuseColor: transformGizmo.dragAxis === "x" ? "#ffffff" : (transformGizmo.isDragging && transformGizmo.dragAxis !== "x" && transformGizmo.dragAxis !== "rx" && transformGizmo.dragAxis !== "ry" && transformGizmo.dragAxis !== "rz" ? "#ffffaa" : "#44ff44")
                                            lighting: DefaultMaterial.NoLighting
                                        }
                                    }
                                    // Arrow head (cone) with gap
                                    Node {
                                        position: Qt.vector3d(transformGizmo.gizmoSize * 1.982 + 20, 0, 0)
                                        visible: true
                                        Model {
                                            objectName: "gizmo_x"
                                            pickable: true
                                            source: "#Cone"
                                            eulerRotation.z: -90
                                            scale: Qt.vector3d(transformGizmo.gizmoSize * 0.05, transformGizmo.gizmoSize * 0.1, transformGizmo.gizmoSize * 0.05)
                                            materials: DefaultMaterial {
                                                diffuseColor: transformGizmo.dragAxis === "x" ? "#ffffff" : (transformGizmo.isDragging && transformGizmo.dragAxis !== "x" && transformGizmo.dragAxis !== "rx" && transformGizmo.dragAxis !== "ry" && transformGizmo.dragAxis !== "rz" ? "#ffffaa" : "#44ff44")
                                                lighting: DefaultMaterial.NoLighting
                                            }
                                        }
                                    }

                                    // ---- Y Axis (Green) - Translation Arrow ----
                                    // Arrow shaft
                                    Model {
                                        objectName: "gizmo_y"
                                        pickable: true
                                        source: "#Cylinder"
                                        visible: true
                                        position: Qt.vector3d(0, transformGizmo.gizmoSize * 1.241, 0)
                                        scale: Qt.vector3d(transformGizmo.gizmoSize * 0.025, transformGizmo.gizmoSize * 0.741, transformGizmo.gizmoSize * 0.025)
                                        materials: DefaultMaterial {
                                            diffuseColor: transformGizmo.dragAxis === "y" ? "#ffffff" : (transformGizmo.isDragging && transformGizmo.dragAxis !== "y" && transformGizmo.dragAxis !== "rx" && transformGizmo.dragAxis !== "ry" && transformGizmo.dragAxis !== "rz" ? "#ffffaa" : "#ff4444")
                                            lighting: DefaultMaterial.NoLighting
                                        }
                                    }
                                    // Arrow head (cone) with gap
                                    Node {
                                        position: Qt.vector3d(0, transformGizmo.gizmoSize * 1.982 + 20, 0)
                                        visible: true
                                        Model {
                                            objectName: "gizmo_y"
                                            pickable: true
                                            source: "#Cone"
                                            scale: Qt.vector3d(transformGizmo.gizmoSize * 0.05, transformGizmo.gizmoSize * 0.1, transformGizmo.gizmoSize * 0.05)
                                            materials: DefaultMaterial {
                                                diffuseColor: transformGizmo.dragAxis === "y" ? "#ffffff" : (transformGizmo.isDragging && transformGizmo.dragAxis !== "y" && transformGizmo.dragAxis !== "rx" && transformGizmo.dragAxis !== "ry" && transformGizmo.dragAxis !== "rz" ? "#ffffaa" : "#ff4444")
                                                lighting: DefaultMaterial.NoLighting
                                            }
                                        }
                                    }

                                    // ---- Z Axis (Blue) - Translation Arrow ----
                                    // Arrow shaft
                                    Model {
                                        objectName: "gizmo_z"
                                        pickable: true
                                        source: "#Cylinder"
                                        visible: true
                                        position: Qt.vector3d(0, 0, transformGizmo.gizmoSize * 1.241)
                                        eulerRotation.x: 90
                                        scale: Qt.vector3d(transformGizmo.gizmoSize * 0.025, transformGizmo.gizmoSize * 0.741, transformGizmo.gizmoSize * 0.025)
                                        materials: DefaultMaterial {
                                            diffuseColor: transformGizmo.dragAxis === "z" ? "#ffffff" : (transformGizmo.isDragging && transformGizmo.dragAxis !== "z" && transformGizmo.dragAxis !== "rx" && transformGizmo.dragAxis !== "ry" && transformGizmo.dragAxis !== "rz" ? "#ffffaa" : "#4444ff")
                                            lighting: DefaultMaterial.NoLighting
                                        }
                                    }
                                    // Arrow head (cone) with gap
                                    Node {
                                        position: Qt.vector3d(0, 0, transformGizmo.gizmoSize * 1.982 + 20)
                                        visible: true
                                        Model {
                                            objectName: "gizmo_z"
                                            pickable: true
                                            source: "#Cone"
                                            eulerRotation.x: 90
                                            scale: Qt.vector3d(transformGizmo.gizmoSize * 0.05, transformGizmo.gizmoSize * 0.1, transformGizmo.gizmoSize * 0.05)
                                            materials: DefaultMaterial {
                                                diffuseColor: transformGizmo.dragAxis === "z" ? "#ffffff" : (transformGizmo.isDragging && transformGizmo.dragAxis !== "z" && transformGizmo.dragAxis !== "rx" && transformGizmo.dragAxis !== "ry" && transformGizmo.dragAxis !== "rz" ? "#ffffaa" : "#4444ff")
                                                lighting: DefaultMaterial.NoLighting
                                            }
                                        }
                                    }
                                } // End translationNode

                                // ---- Rotation Rings ----
                                // Red Ring - vertical disc in YZ plane (rotate around X)
                                Node {
                                    id: xRotationRingNode
                                    visible: true
                                    eulerRotation: Qt.vector3d(transformGizmo.gizmoEulerRotation.x, 0, 0)  // Rotate cone with X axis
                                    Model {
                                        id: xRotationRing
                                        objectName: "gizmo_rx"
                                        pickable: true
                                        source: "file:///home/idgaf/Documents/robot/Robot_controller%20/urdf_standalone/visualization/resources/3d/meshes/node.mesh"
                                        position: Qt.vector3d(0, 0, 0)
                                        eulerRotation: Qt.vector3d(180, 90, 90)  // 180x for extra rotation, 90y for YZ plane, 90z to flatten
                                        scale: Qt.vector3d(transformGizmo.gizmoSize * 0.35, transformGizmo.gizmoSize * 0.35, transformGizmo.gizmoSize * 0.35)
                                        materials: DefaultMaterial {
                                            diffuseColor: transformGizmo.dragAxis === "rx" ? "#ffffff" : (transformGizmo.isDragging && transformGizmo.dragAxis !== "rx" ? "#ffffaa" : "#66ff66")
                                            lighting: DefaultMaterial.NoLighting
                                        }
                                        opacity: 0.5
                                    }
                                    // Zero degree marker cone
                                    Model {
                                        source: "#Cone"
                                        position: Qt.vector3d(0, transformGizmo.gizmoSize * 35, 0)  // At ring edge (radius 100 * 0.35)
                                        eulerRotation: Qt.vector3d(0, 0, 0)
                                        scale: Qt.vector3d(transformGizmo.gizmoSize * 0.026, transformGizmo.gizmoSize * 0.052, transformGizmo.gizmoSize * 0.026)
                                        materials: DefaultMaterial {
                                            diffuseColor: "#66ff66"
                                            lighting: DefaultMaterial.NoLighting
                                        }
                                    }
                                }

                                // Green Ring - horizontal disc in XZ plane (rotate around Y)
                                Node {
                                    id: yRotationRingNode
                                    visible: true
                                    eulerRotation: Qt.vector3d(0, transformGizmo.gizmoEulerRotation.z, 0)  // Swapped: rotate with Z axis value
                                    Model {
                                        id: yRotationRing
                                        objectName: "gizmo_ry"
                                        pickable: true
                                        source: "file:///home/idgaf/Documents/robot/Robot_controller%20/urdf_standalone/visualization/resources/3d/meshes/node.mesh"
                                        position: Qt.vector3d(0, 0, 0)
                                        eulerRotation: Qt.vector3d(90, 0, 0)  // 90x to flatten, in XZ plane
                                        scale: Qt.vector3d(transformGizmo.gizmoSize * 0.375, transformGizmo.gizmoSize * 0.375, transformGizmo.gizmoSize * 0.375)
                                        materials: DefaultMaterial {
                                            diffuseColor: transformGizmo.dragAxis === "ry" ? "#ffffff" : (transformGizmo.isDragging && transformGizmo.dragAxis !== "ry" ? "#ffffaa" : "#ff6666")
                                            lighting: DefaultMaterial.NoLighting
                                        }
                                        opacity: 0.5
                                    }
                                    // Zero degree marker cone
                                    Model {
                                        source: "#Cone"
                                        position: Qt.vector3d(transformGizmo.gizmoSize * 37.5, 0, 0)  // At ring edge (radius 100 * 0.375)
                                        eulerRotation: Qt.vector3d(0, 0, -90)  // Point inward
                                        scale: Qt.vector3d(transformGizmo.gizmoSize * 0.026, transformGizmo.gizmoSize * 0.052, transformGizmo.gizmoSize * 0.026)
                                        materials: DefaultMaterial {
                                            diffuseColor: "#ff6666"
                                            lighting: DefaultMaterial.NoLighting
                                        }
                                    }
                                }

                                // Blue Ring - vertical disc in XY plane (rotate around Z)
                                Node {
                                    id: zRotationRingNode
                                    visible: true
                                    eulerRotation: Qt.vector3d(0, 0, transformGizmo.gizmoEulerRotation.y)  // Swapped: rotate with Y axis value
                                    Model {
                                        id: zRotationRing
                                        objectName: "gizmo_rz"
                                        pickable: true
                                        source: "file:///home/idgaf/Documents/robot/Robot_controller%20/urdf_standalone/visualization/resources/3d/meshes/node.mesh"
                                        position: Qt.vector3d(0, 0, 0)
                                        eulerRotation: Qt.vector3d(0, 0, 0)  // Natural XY plane
                                        scale: Qt.vector3d(transformGizmo.gizmoSize * 0.4, transformGizmo.gizmoSize * 0.4, transformGizmo.gizmoSize * 0.4)
                                        materials: DefaultMaterial {
                                            diffuseColor: transformGizmo.dragAxis === "rz" ? "#ffffff" : (transformGizmo.isDragging && transformGizmo.dragAxis !== "rz" ? "#ffffaa" : "#6666ff")
                                            lighting: DefaultMaterial.NoLighting
                                        }
                                        opacity: 0.5
                                    }
                                    // Zero degree marker cone
                                    Model {
                                        source: "#Cone"
                                        position: Qt.vector3d(transformGizmo.gizmoSize * 40, 0, 0)  // At ring edge (radius 100 * 0.4)
                                        eulerRotation: Qt.vector3d(0, 0, -90)  // Point inward
                                        scale: Qt.vector3d(transformGizmo.gizmoSize * 0.026, transformGizmo.gizmoSize * 0.052, transformGizmo.gizmoSize * 0.026)
                                        materials: DefaultMaterial {
                                            diffuseColor: "#6666ff"
                                            lighting: DefaultMaterial.NoLighting
                                        }
                                    }
                                }
                            }
                            // =====================================================
                        } // End View3D

                        // Collision Warning Banner (minimal - mesh coloring does the work)
                        Rectangle {
                            id: collisionBanner
                            anchors.top: parent.top
                            anchors.horizontalCenter: parent.horizontalCenter
                            anchors.topMargin: 80
                            width: collisionText.width + 40
                            height: 40
                            radius: 20
                            color: "#CC000000"
                            border.color: "#FF0000"
                            border.width: 2
                            z: 999
                            visible: collidingPairs.length > 0 && isSimulationMode

                            Text {
                                id: collisionText
                                anchors.centerIn: parent
                                text: "⚠️ COLLISION: " + collidingPairs.length + " pair(s)"
                                color: "#FF5252"
                                font.pixelSize: 16
                                font.bold: true
                            }
                        }

                        // Floating Control Mode Switcher (centered at top)
                        Rectangle {
                            anchors.horizontalCenter: parent.horizontalCenter
                            anchors.top: parent.top
                            anchors.topMargin: 20
                            width: 260
                            height: 60
                            radius: 12
                            color: "#aa1e1e1e"
                            border.color: "#30ffffff"
                            border.width: 1
                            z: 200

                            Item {
                                anchors.fill: parent

                                Row {
                                    anchors.centerIn: parent
                                    spacing: 10

                                    // Robot Mode Button
                                    Rectangle {
                                        width: 110
                                        height: 40
                                        radius: 20
                                        color: !isSimulationMode ? "#ffffff" : "#303030"
                                        border.color: !isSimulationMode ? "#ffffff" : "#606060"
                                        border.width: 2

                                        Text {
                                            text: "Robot"
                                            anchors.centerIn: parent
                                            color: !isSimulationMode ? "#1e1e1e" : "#888888"
                                            font.pixelSize: 13
                                            font.bold: true
                                            font.family: "Segoe UI"
                                        }

                                        MouseArea {
                                            anchors.fill: parent
                                            cursorShape: Qt.PointingHandCursor
                                            onClicked: {
                                                if (isSimulationMode) {
                                                    // Save simulation gizmo state before switching
                                                    simGizmoPosition = transformGizmo.gizmoPosition;
                                                    simGizmoRotation = transformGizmo.gizmoEulerRotation;
                                                    simGizmoHasBeenDragged = transformGizmo.hasBeenDragged;

                                                    // Restore robot gizmo state (or reset to EE position)
                                                    transformGizmo.hasBeenDragged = robotGizmoHasBeenDragged;
                                                    if (robotGizmoHasBeenDragged) {
                                                        transformGizmo.gizmoPosition = robotGizmoPosition;
                                                        transformGizmo.gizmoEulerRotation = robotGizmoRotation;
                                                    } else {
                                                        transformGizmo.updatePosition(true);  // Sync to current EE
                                                    }

                                                    console.log("Switching to Robot mode - starting auto-detection");
                                                    robotManager.scanForRobot();
                                                }
                                                isSimulationMode = false;
                                                logsPanelRight.clearLogs(); // Clear logs on mode switch
                                                var angles = {};
                                                var jointIdx = 0;
                                                for (var i = 0; i < currentLinks.length; i++) {
                                                    if (currentLinks[i].hasJoint && currentLinks[i].jointName !== "right_finger_joint") {
                                                        angles[currentLinks[i].jointName] = currentJointPositions[jointIdx];
                                                        jointIdx++;
                                                    }
                                                }
                                                jointAngles = angles;
                                            }
                                        }
                                    }

                                    // Simulation Mode Button
                                    Rectangle {
                                        width: 110
                                        height: 40
                                        radius: 20
                                        color: isSimulationMode ? "#ffffff" : "#303030"
                                        border.color: isSimulationMode ? "#ffffff" : "#606060"
                                        border.width: 2

                                        Text {
                                            text: "Simulation"
                                            anchors.centerIn: parent
                                            color: isSimulationMode ? "#1e1e1e" : "#888888"
                                            font.pixelSize: 13
                                            font.bold: true
                                            font.family: "Segoe UI"
                                        }

                                        MouseArea {
                                            anchors.fill: parent
                                            cursorShape: Qt.PointingHandCursor
                                            onClicked: {
                                                if (!isSimulationMode) {
                                                    // Save robot gizmo state before switching
                                                    robotGizmoPosition = transformGizmo.gizmoPosition;
                                                    robotGizmoRotation = transformGizmo.gizmoEulerRotation;
                                                    robotGizmoHasBeenDragged = transformGizmo.hasBeenDragged;

                                                    // Restore simulation gizmo state (or reset to EE position)
                                                    transformGizmo.hasBeenDragged = simGizmoHasBeenDragged;
                                                    if (simGizmoHasBeenDragged) {
                                                        transformGizmo.gizmoPosition = simGizmoPosition;
                                                        transformGizmo.gizmoEulerRotation = simGizmoRotation;
                                                    } else {
                                                        transformGizmo.updatePosition(true);  // Sync to current EE
                                                    }

                                                    console.log("Sending CMD_DEHOME and switching to Simulation");
                                                    robotManager.sendDehome();
                                                    robotManager.disconnectRobot();
                                                }
                                                isSimulationMode = true;
                                                logsPanelRight.clearLogs(); // Clear logs on mode switch
                                                var zeroPos = [];
                                                for (var k = 0; k < 7; k++)
                                                    zeroPos.push(0.0);
                                                simulationJointPositions = zeroPos;
                                                var angles = {};
                                                var jointIdx = 0;
                                                for (var i = 0; i < currentLinks.length; i++) {
                                                    if (currentLinks[i].hasJoint && currentLinks[i].jointName !== "right_finger_joint") {
                                                        angles[currentLinks[i].jointName] = 0.0;
                                                        jointIdx++;
                                                    }
                                                }
                                                jointAngles = angles;
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        // Floating Status Panel (bottom-left orange area)
                        Rectangle {
                            anchors.left: parent.left
                            anchors.bottom: parent.bottom
                            anchors.margins: 20
                            width: 500
                            height: 60
                            radius: 12
                            color: "#aa202020"
                            border.color: "#30ffffff"
                            border.width: 1
                            z: 200

                            Row {
                                anchors.fill: parent
                                anchors.margins: 12
                                spacing: 15

                                Text {
                                    text: "Status:"
                                    color: "#ffffff"
                                    font.pixelSize: 12
                                    font.bold: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }

                                // Mode
                                Row {
                                    spacing: 5
                                    anchors.verticalCenter: parent.verticalCenter
                                    Text {
                                        text: "Mode:"
                                        color: "#888888"
                                        font.pixelSize: 10
                                    }
                                    Text {
                                        text: isSimulationMode ? "Simulation" : "Robot"
                                        color: isSimulationMode ? "#FF9800" : "#4CAF50"
                                        font.pixelSize: 10
                                        font.bold: true
                                    }
                                }

                                // Connection
                                Row {
                                    visible: !isSimulationMode
                                    spacing: 5
                                    anchors.verticalCenter: parent.verticalCenter
                                    Text {
                                        text: "Connected:"
                                        color: "#888888"
                                        font.pixelSize: 10
                                    }
                                    Text {
                                        text: robotManager ? (robotManager.connected ? "Yes" : "No") : "No"
                                        color: robotManager && robotManager.connected ? "#4CAF50" : "#f44336"
                                        font.pixelSize: 10
                                        font.bold: true
                                    }
                                }

                                // Port
                                Row {
                                    visible: !isSimulationMode && robotManager && robotManager.connected
                                    spacing: 5
                                    anchors.verticalCenter: parent.verticalCenter
                                    Text {
                                        text: "Port:"
                                        color: "#888888"
                                        font.pixelSize: 10
                                    }
                                    Text {
                                        text: robotManager ? robotManager.portName : ""
                                        color: "#cccccc"
                                        font.pixelSize: 10
                                        font.family: "Monospace"
                                    }
                                }

                                // Robot State
                                Row {
                                    visible: !isSimulationMode
                                    spacing: 5
                                    anchors.verticalCenter: parent.verticalCenter
                                    Text {
                                        text: "State:"
                                        color: "#888888"
                                        font.pixelSize: 10
                                    }
                                    Text {
                                        text: robotManager ? robotManager.robotStateText : "N/A"
                                        color: "#2196F3"
                                        font.pixelSize: 10
                                        font.bold: true
                                    }
                                }
                            }
                        }

                        // Custom Camera Controller (Restricted to ViewContainer)
                        // Also handles gizmo picking and dragging
                        MouseArea {
                            anchors.fill: parent
                            acceptedButtons: Qt.LeftButton | Qt.RightButton
                            hoverEnabled: true

                            property point lastPos

                            // Check if objectName starts with "gizmo_"
                            function isGizmoPick(objectName) {
                                return objectName && objectName.indexOf("gizmo_") === 0;
                            }

                            // Extract axis from objectName (e.g., "gizmo_x" -> "x")
                            function getGizmoAxis(objectName) {
                                if (!objectName)
                                    return "";
                                return objectName.replace("gizmo_", "");
                            }

                            onPressed: mouse => {
                                lastPos = Qt.point(mouse.x, mouse.y);

                                // Try to pick gizmo with left button (works in both simulation and robot mode)
                                if (mouse.button === Qt.LeftButton) {
                                    var pickResult = view3d.pick(mouse.x, mouse.y);
                                    if (pickResult.objectHit && isGizmoPick(pickResult.objectHit.objectName)) {
                                        var axis = getGizmoAxis(pickResult.objectHit.objectName);
                                        console.log("Gizmo picked:", axis);
                                        transformGizmo.isDragging = true;
                                        transformGizmo.hasBeenDragged = true;  // Prevent snapping back after release
                                        transformGizmo.dragAxis = axis;
                                        transformGizmo.dragStartPos = Qt.point(mouse.x, mouse.y);
                                        transformGizmo.dragStartGizmoPos = transformGizmo.gizmoPosition;
                                        mouse.accepted = true;
                                        return;
                                    }
                                }
                            }

                            onPositionChanged: mouse => {
                                var dx = mouse.x - lastPos.x;
                                var dy = mouse.y - lastPos.y;

                                // Handle gizmo dragging
                                if (transformGizmo.isDragging) {
                                    var dragSpeed = 0.12;  // Sensitivity - higher = faster response
                                    var axis = transformGizmo.dragAxis;
                                    var pos = transformGizmo.gizmoPosition;

                                    // Camera-relative vectors for all axis modes
                                    var radY = cameraRotationNode.eulerRotation.y * Math.PI / 180;
                                    var radX = cameraRotationNode.eulerRotation.x * Math.PI / 180;
                                    var rightX = -Math.cos(radY);
                                    var rightZ = -Math.sin(radY);
                                    var upY = Math.cos(radX);
                                    var upForward = -Math.sin(radX);
                                    var forwardX = Math.sin(radY);
                                    var forwardZ = -Math.cos(radY);

                                    if (axis === "x" || axis === "y" || axis === "z") {
                                        // Local Frame Dragging
                                        var rot = window.eulerToQuaternion(transformGizmo.gizmoEulerRotation);

                                        var localAxis;
                                        if (axis === "x")
                                            localAxis = Qt.vector3d(1, 0, 0);
                                        else
                                        // Red
                                        if (axis === "y")
                                            localAxis = Qt.vector3d(0, 1, 0);
                                        else
                                        // Green
                                        if (axis === "z")
                                            localAxis = Qt.vector3d(0, 0, 1);  // Blue

                                        var worldAxis = window.transformToWorldFrame(localAxis, rot);

                                        // Project mouse movement onto the screen-projected world axis
                                        var camRight = Qt.vector3d(rightX, 0, rightZ);
                                        var camUp = Qt.vector3d(upForward * forwardX, upY, upForward * forwardZ);

                                        var screenProjX = worldAxis.dotProduct(camRight);
                                        var screenProjY = worldAxis.dotProduct(camUp);

                                        // Calculate move amount (dy is inverted)
                                        var moveAmount = (dx * screenProjX + (-dy) * screenProjY) * dragSpeed;

                                        // Apply to position
                                        var moveVec = worldAxis.times(moveAmount);
                                        transformGizmo.gizmoPosition = transformGizmo.gizmoPosition.plus(moveVec);

                                        if (window.ikEnabled) {
                                            window.handleGizmoIK();
                                        }
                                    } else if (axis === "rx") {
                                        // Red ring: rotate around X axis - use horizontal mouse
                                        var rotSpeed = 0.5;  // degrees per pixel
                                        var rot = transformGizmo.gizmoEulerRotation;
                                        transformGizmo.gizmoEulerRotation = Qt.vector3d(rot.x + dx * rotSpeed, rot.y, rot.z);
                                        if (window.ikEnabled) {
                                            window.handleGizmoIK();
                                        }
                                    } else if (axis === "ry") {
                                        // Red ring: rotate around Z axis (swapped) - use vertical mouse
                                        var rotSpeed = 0.5;
                                        var rot = transformGizmo.gizmoEulerRotation;
                                        transformGizmo.gizmoEulerRotation = Qt.vector3d(rot.x, rot.y, rot.z + dy * rotSpeed);
                                        if (window.ikEnabled) {
                                            window.handleGizmoIK();
                                        }
                                    } else if (axis === "rz") {
                                        // Blue ring: rotate around Y axis (swapped) - use horizontal mouse
                                        var rotSpeed = 0.5;
                                        var rot = transformGizmo.gizmoEulerRotation;
                                        transformGizmo.gizmoEulerRotation = Qt.vector3d(rot.x, rot.y + dx * rotSpeed, rot.z);
                                        if (window.ikEnabled) {
                                            window.handleGizmoIK();
                                        }
                                    } else if (axis === "free") {
                                        // Free: FULL camera-relative movement using both yaw AND pitch
                                        var radY = cameraRotationNode.eulerRotation.y * Math.PI / 180;
                                        var radX = cameraRotationNode.eulerRotation.x * Math.PI / 180;

                                        // Camera right vector (for horizontal mouse → left/right visual movement)
                                        // Negated to match visual expectation (right=positive screen X)
                                        var rightX = -Math.cos(radY);
                                        var rightZ = -Math.sin(radY);

                                        // Camera up vector in world space (depends on pitch)
                                        // When pitch=0, up is (0,1,0). When pitch=90, up points forward
                                        var upY = Math.cos(radX);  // How much of "up" is world Y
                                        var upForward = -Math.sin(radX);  // Negated: how much of "up" is camera forward

                                        // Camera forward vector in XZ plane (also negated)
                                        var forwardX = Math.sin(radY);
                                        var forwardZ = -Math.cos(radY);

                                        // Horizontal mouse → camera right direction (pure XZ plane)
                                        var moveX = dx * rightX * dragSpeed;
                                        var moveZ = dx * rightZ * dragSpeed;

                                        // Vertical mouse → combination of Y and forward based on pitch
                                        // dy negative means mouse up = positive visual up
                                        var moveY = -dy * upY * dragSpeed;  // Y component
                                        moveX += -dy * upForward * forwardX * dragSpeed;  // Forward X component
                                        moveZ += -dy * upForward * forwardZ * dragSpeed;  // Forward Z component

                                        // Debug logging during movement
                                        console.log("FREE DRAG: dx=" + dx.toFixed(1) + " dy=" + dy.toFixed(1) + " camY=" + cameraRotationNode.eulerRotation.y.toFixed(1) + "° moveX=" + moveX.toFixed(2) + " moveZ=" + moveZ.toFixed(2) + " moveY=" + moveY.toFixed(2));

                                        transformGizmo.gizmoPosition = Qt.vector3d(pos.x - moveX, pos.y + moveY, pos.z + moveZ);

                                        // Choose control mode based on settings
                                        if (window.ikEnabled) {
                                            window.handleGizmoIK();
                                        }
                                    }

                                    lastPos = Qt.point(mouse.x, mouse.y);
                                    return;
                                }

                                // Normal camera control
                                if (mouse.buttons & Qt.LeftButton) {
                                    cameraRotationNode.eulerRotation.y -= dx * orbitSpeed;
                                    cameraRotationNode.eulerRotation.x -= dy * orbitSpeed;
                                    cameraRotationNode.eulerRotation.x = Math.max(-90, Math.min(90, cameraRotationNode.eulerRotation.x));
                                } else if (mouse.buttons & Qt.RightButton) {
                                    var radY = cameraRotationNode.eulerRotation.y * Math.PI / 180;
                                    var rightX = Math.cos(radY);
                                    var rightZ = Math.sin(radY);
                                    var speed = panSpeed * (cameraDistance / 200.0);
                                    cameraPivot.position = Qt.vector3d(cameraPivot.position.x - (dx * rightX * speed), cameraPivot.position.y + (dy * speed), cameraPivot.position.z - (dx * rightZ * speed));
                                }
                                lastPos = Qt.point(mouse.x, mouse.y);
                            }

                            onReleased: mouse => {
                                // End gizmo dragging
                                if (transformGizmo.isDragging) {
                                    console.log("Gizmo drag ended, position:", transformGizmo.gizmoPosition, "axis:", transformGizmo.dragAxis);

                                    // Stop motion in velocity mode
                                    if (!isSimulationMode && useVelocityMode) {
                                        robotManager.stopMotion();
                                    }

                                    // Solve final position IK (for simulation mode or position mode)
                                    if (window.ikEnabled && (isSimulationMode || !useVelocityMode)) {
                                        window.solveIKFromGizmo();
                                    }

                                    transformGizmo.isDragging = false;
                                    transformGizmo.dragAxis = "";
                                }
                            }

                            onWheel: wheel => {
                                var delta = wheel.angleDelta.y / 120.0;
                                cameraDistance -= delta * zoomSpeed;
                                cameraDistance = Math.max(10.0, Math.min(5000.0, cameraDistance));
                            }
                        }

                        // World Frame Axis Triad Overlay
                        Item {
                            id: axisTriadContainer
                            anchors.right: parent.right
                            anchors.top: parent.top
                            anchors.margins: 20
                            width: 120
                            height: 120

                            // Background circle
                            Rectangle {
                                anchors.fill: parent
                                radius: width / 2
                                color: "#40000000"
                                border.color: "#30ffffff"
                                border.width: 1
                            }

                            // Secondary 3D View for axis triad
                            View3D {
                                id: axisTriadView
                                anchors.fill: parent

                                environment: SceneEnvironment {
                                    clearColor: "transparent"
                                    backgroundMode: SceneEnvironment.Transparent
                                    antialiasingMode: SceneEnvironment.MSAA
                                    antialiasingQuality: SceneEnvironment.High
                                }

                                // Camera that follows main camera rotation
                                Node {
                                    id: triadCameraPivot

                                    Node {
                                        id: triadCameraRotation
                                        // Sync with main camera rotation
                                        eulerRotation.x: cameraRotationNode.eulerRotation.x
                                        eulerRotation.y: cameraRotationNode.eulerRotation.y

                                        PerspectiveCamera {
                                            id: triadCamera
                                            position: Qt.vector3d(0, 0, 200)
                                            clipNear: 1.0
                                            clipFar: 1000.0
                                        }
                                    }
                                }

                                // Ambient light
                                DirectionalLight {
                                    eulerRotation.x: -30
                                    eulerRotation.y: 30
                                    brightness: 1.5
                                }

                                // X Axis (Red) - Points right
                                Model {
                                    source: "#Cylinder"
                                    scale: Qt.vector3d(0.03, 0.5, 0.03)
                                    position: Qt.vector3d(25, 0, 0)
                                    eulerRotation.z: -90
                                    materials: [
                                        DefaultMaterial {
                                            diffuseColor: "#ef5350"
                                        }
                                    ]
                                }
                                Model {
                                    id: xSphere
                                    source: "#Sphere"
                                    scale: Qt.vector3d(0.15, 0.15, 0.15)
                                    position: Qt.vector3d(55, 0, 0)
                                    materials: [
                                        DefaultMaterial {
                                            diffuseColor: "#ef5350"
                                        }
                                    ]
                                }

                                // Y Axis (Green) - Points up
                                Model {
                                    source: "#Cylinder"
                                    scale: Qt.vector3d(0.03, 0.5, 0.03)
                                    position: Qt.vector3d(0, 25, 0)
                                    materials: [
                                        DefaultMaterial {
                                            diffuseColor: "#66bb6a"
                                        }
                                    ]
                                }
                                Model {
                                    id: ySphere
                                    source: "#Sphere"
                                    scale: Qt.vector3d(0.15, 0.15, 0.15)
                                    position: Qt.vector3d(0, 55, 0)
                                    materials: [
                                        DefaultMaterial {
                                            diffuseColor: "#66bb6a"
                                        }
                                    ]
                                }

                                // Z Axis (Blue) - Points toward viewer
                                Model {
                                    source: "#Cylinder"
                                    scale: Qt.vector3d(0.03, 0.5, 0.03)
                                    position: Qt.vector3d(0, 0, 25)
                                    eulerRotation.x: 90
                                    materials: [
                                        DefaultMaterial {
                                            diffuseColor: "#42a5f5"
                                        }
                                    ]
                                }
                                Model {
                                    id: zSphere
                                    source: "#Sphere"
                                    scale: Qt.vector3d(0.15, 0.15, 0.15)
                                    position: Qt.vector3d(0, 0, 55)
                                    materials: [
                                        DefaultMaterial {
                                            diffuseColor: "#42a5f5"
                                        }
                                    ]
                                }

                                // Center sphere
                                Model {
                                    source: "#Sphere"
                                    scale: Qt.vector3d(0.12, 0.12, 0.12)
                                    position: Qt.vector3d(0, 0, 0)
                                    materials: [
                                        DefaultMaterial {
                                            diffuseColor: "#ffffff"
                                        }
                                    ]
                                }
                            }

                            // X Label (Red) - positioned dynamically based on camera
                            Text {
                                id: xLabel
                                text: "X"
                                color: "#ef5350"
                                font.pixelSize: 14
                                font.bold: true
                                x: parent.width / 2 + 45 - width / 2
                                y: parent.height / 2 - height / 2

                                MouseArea {
                                    anchors.fill: parent
                                    anchors.margins: -5
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: {
                                        // View from +X axis (looking toward origin)
                                        cameraRotationNode.animateCamera = true;
                                        resetAnimateCameraTimer.restart();
                                        cameraRotationNode.eulerRotation.x = 0;
                                        cameraRotationNode.eulerRotation.y = -90;
                                        console.log("Camera aligned to X axis view");
                                    }
                                }
                            }

                            // Y Label (Green)
                            Text {
                                id: yLabel
                                text: "Y"
                                color: "#66bb6a"
                                font.pixelSize: 14
                                font.bold: true
                                x: parent.width / 2 - width / 2
                                y: parent.height / 2 - 45 - height / 2

                                MouseArea {
                                    anchors.fill: parent
                                    anchors.margins: -5
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: {
                                        // View from +Y axis (looking down)
                                        cameraRotationNode.animateCamera = true;
                                        resetAnimateCameraTimer.restart();
                                        cameraRotationNode.eulerRotation.x = -89;
                                        cameraRotationNode.eulerRotation.y = 0;
                                        console.log("Camera aligned to Y axis view (top)");
                                    }
                                }
                            }

                            // Z Label (Blue)
                            Text {
                                id: zLabel
                                text: "Z"
                                color: "#42a5f5"
                                font.pixelSize: 14
                                font.bold: true
                                x: parent.width / 2 - width / 2
                                y: parent.height / 2 + 45 - height / 2

                                MouseArea {
                                    anchors.fill: parent
                                    anchors.margins: -5
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: {
                                        // View from +Z axis (front view)
                                        cameraRotationNode.animateCamera = true;
                                        resetAnimateCameraTimer.restart();
                                        cameraRotationNode.eulerRotation.x = 0;
                                        cameraRotationNode.eulerRotation.y = 0;
                                        console.log("Camera aligned to Z axis view (front)");
                                    }
                                }
                            }

                            // Make entire triad clickable to reset to default view
                            MouseArea {
                                anchors.fill: parent
                                z: -1  // Behind the labels
                                onDoubleClicked: {
                                    // Reset to default isometric view
                                    cameraRotationNode.animateCamera = true;
                                    resetAnimateCameraTimer.restart();
                                    cameraRotationNode.eulerRotation.x = -20;
                                    cameraRotationNode.eulerRotation.y = 45;
                                    console.log("Camera reset to default isometric view");
                                }
                            }

                            // Axes Toggle Button - positioned below the triad circle
                            Rectangle {
                                id: axesToggleButton
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.top: parent.bottom
                                anchors.topMargin: 10
                                width: 44
                                height: 44
                                radius: 22
                                color: "#aa1e1e1e"
                                border.color: "#30ffffff"
                                border.width: 1

                                Text {
                                    text: showAxesMode === 0 ? "○" : (showAxesMode === 1 ? "◉" : "◎")
                                    anchors.centerIn: parent
                                    color: showAxesMode === 0 ? "#888888" : (showAxesMode === 1 ? "#ffaa44" : "#ffffff")
                                    font.pixelSize: 24
                                    font.bold: true
                                }

                                MouseArea {
                                    anchors.fill: parent
                                    cursorShape: Qt.PointingHandCursor
                                    hoverEnabled: true
                                    onClicked: {
                                        showAxesMode = (showAxesMode + 1) % 3;
                                        console.log("Axes visibility:", showAxesMode === 0 ? "Off" : (showAxesMode === 1 ? "EE Link" : "All"));
                                    }

                                    ToolTip.visible: containsMouse
                                    ToolTip.text: "Axes: " + (showAxesMode === 0 ? "Off" : (showAxesMode === 1 ? "EE Link" : "All Joints"))
                                    ToolTip.delay: 500
                                }
                            }
                        }

                        // Native Helper Overlay (Anchored to ViewContainer)
                        Item {
                            anchors.right: parent.right
                            anchors.bottom: parent.bottom
                            anchors.margins: 20
                            width: 260
                            height: 100

                            Column {
                                anchors.right: parent.right
                                anchors.bottom: parent.bottom
                                spacing: 8

                                // 1. Rotate
                                Row {
                                    spacing: 10
                                    Item {
                                        width: 18
                                        height: 24
                                        Rectangle {
                                            width: 12
                                            height: 20
                                            radius: 4
                                            border.color: "#80ffffff"
                                            border.width: 1.5
                                            color: "transparent"
                                            anchors.centerIn: parent
                                        }
                                        Rectangle {
                                            width: 4
                                            height: 6
                                            color: "#ffffff"
                                            x: 3
                                            y: 4
                                            radius: 1
                                        }
                                    }
                                    Text {
                                        text: "Drag to rotate the perspective"
                                        color: "#a0ffffff"
                                        font.pixelSize: 12
                                        font.family: "Segoe UI"
                                        anchors.verticalCenter: parent.verticalCenter
                                    }
                                }
                                // 2. Pan
                                Row {
                                    spacing: 10
                                    Item {
                                        width: 18
                                        height: 24
                                        Rectangle {
                                            width: 12
                                            height: 20
                                            radius: 4
                                            border.color: "#80ffffff"
                                            border.width: 1.5
                                            color: "transparent"
                                            anchors.centerIn: parent
                                        }
                                        Rectangle {
                                            width: 4
                                            height: 6
                                            color: "#ffffff"
                                            x: 7
                                            y: 4
                                            radius: 1
                                        }
                                    }
                                    Text {
                                        text: "Drag to move the perspective"
                                        color: "#a0ffffff"
                                        font.pixelSize: 12
                                        font.family: "Segoe UI"
                                        anchors.verticalCenter: parent.verticalCenter
                                    }
                                }
                                // 3. Zoom
                                Row {
                                    spacing: 10
                                    Item {
                                        width: 18
                                        height: 24
                                        Rectangle {
                                            width: 12
                                            height: 20
                                            radius: 4
                                            border.color: "#80ffffff"
                                            border.width: 1.5
                                            color: "transparent"
                                            anchors.centerIn: parent
                                        }
                                        Rectangle {
                                            width: 4
                                            height: 5
                                            color: "#ffffff"
                                            anchors.top: parent.top
                                            anchors.horizontalCenter: parent.horizontalCenter
                                            anchors.topMargin: 3
                                            radius: 1.5
                                        }
                                    }
                                    Text {
                                        text: "Scroll to zoom"
                                        color: "#a0ffffff"
                                        font.pixelSize: 12
                                        font.family: "Segoe UI"
                                        anchors.verticalCenter: parent.verticalCenter
                                    }
                                }
                            }
                        }
                    } // End Item (viewContainer)
                } // End Tab 1 (Control)

                // Tab 2: Graphs
                Item {
                    anchors.fill: parent

                    // Sidebar with joint sliders
                    Rectangle {
                        id: graphsSidebar
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
                                text: "Joint Selection"
                                color: "white"
                                font.pixelSize: 16
                                font.bold: true
                                Layout.alignment: Qt.AlignHCenter
                            }

                            ListView {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                clip: true
                                model: currentLinks
                                spacing: 8

                                property int jointCounter: 0

                                delegate: Item {
                                    width: ListView.view.width
                                    height: modelData.hasJoint ? (expanded ? 120 : 50) : 0
                                    visible: modelData.hasJoint && modelData.jointName !== "right_finger_joint"

                                    property bool expanded: false

                                    // Calculate actual joint index by counting previous joints
                                    property int jointIndex: {
                                        var count = 0;
                                        for (var i = 0; i < index; i++) {
                                            if (currentLinks[i].hasJoint && currentLinks[i].jointName !== "right_finger_joint") {
                                                count++;
                                            }
                                        }
                                        return count;
                                    }
                                    property bool isSelected: jointIndex < 7 ? selectedJoints[jointIndex] : false
                                    property string jointColor: jointIndex < 7 ? jointColors[jointIndex] : "#cccccc"

                                    Behavior on height {
                                        NumberAnimation {
                                            duration: 200
                                            easing.type: Easing.InOutQuad
                                        }
                                    }

                                    Rectangle {
                                        anchors.fill: parent
                                        color: isSelected ? "#3a3a3a" : "#2a2a2a"
                                        radius: 5
                                        border.color: isSelected ? jointColor : "transparent"
                                        border.width: 2

                                        Column {
                                            anchors.fill: parent
                                            anchors.margins: 8
                                            spacing: 5

                                            // Header row (always visible)
                                            Item {
                                                width: parent.width
                                                height: 20

                                                Row {
                                                    spacing: 5
                                                    anchors.left: parent.left
                                                    anchors.verticalCenter: parent.verticalCenter

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
                                                        color: isSelected ? jointColor : "#cccccc"
                                                        font.pixelSize: 12
                                                        font.bold: true
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }

                                                    Text {
                                                        text: "[J" + jointIndex + "]"
                                                        color: "#666666"
                                                        font.pixelSize: 10
                                                        anchors.verticalCenter: parent.verticalCenter
                                                    }
                                                }

                                                // Expand button
                                                Rectangle {
                                                    width: 20
                                                    height: 20
                                                    radius: 10
                                                    color: expandButtonArea.pressed ? "#505050" : (expandButtonArea.containsMouse ? "#454545" : "#3a3a3a")
                                                    border.color: "#666666"
                                                    border.width: 1
                                                    anchors.right: parent.right
                                                    anchors.verticalCenter: parent.verticalCenter

                                                    Text {
                                                        text: expanded ? "▽" : "▷"
                                                        color: "#aaaaaa"
                                                        font.pixelSize: 10
                                                        anchors.centerIn: parent
                                                    }

                                                    MouseArea {
                                                        id: expandButtonArea
                                                        anchors.fill: parent
                                                        hoverEnabled: true
                                                        cursorShape: Qt.PointingHandCursor
                                                        propagateComposedEvents: false
                                                        onClicked: function (mouse) {
                                                            expanded = !expanded;
                                                            mouse.accepted = true;
                                                        }
                                                    }
                                                }
                                            }

                                            Row {
                                                width: parent.width
                                                spacing: 5

                                                Text {
                                                    text: "Min: " + modelData.jointLowerLimit.toFixed(2)
                                                    color: "#888888"
                                                    font.pixelSize: 10
                                                }

                                                Text {
                                                    text: "Max: " + modelData.jointUpperLimit.toFixed(2)
                                                    color: "#888888"
                                                    font.pixelSize: 10
                                                }
                                            }

                                            // Slider section (visible when expanded)
                                            Column {
                                                width: parent.width
                                                spacing: 3
                                                visible: expanded
                                                opacity: expanded ? 1.0 : 0.0

                                                Behavior on opacity {
                                                    NumberAnimation {
                                                        duration: 200
                                                    }
                                                }

                                                Row {
                                                    width: parent.width
                                                    spacing: 5

                                                    Text {
                                                        text: "Control:"
                                                        color: "#aaaaaa"
                                                        font.pixelSize: 10
                                                    }

                                                    Text {
                                                        id: valueDisplay
                                                        text: jointSlider.value.toFixed(2) + " rad"
                                                        color: jointColor
                                                        font.pixelSize: 10
                                                        font.family: "Monospace"
                                                        font.bold: true
                                                    }
                                                }

                                                Slider {
                                                    id: jointSlider
                                                    width: parent.width
                                                    from: modelData.jointLowerLimit
                                                    to: modelData.jointUpperLimit

                                                    // Decoupled binding
                                                    Binding on value {
                                                        when: !jointSlider.pressed
                                                        value: window.getJointValueByIndex(jointIndex)
                                                    }

                                                    onMoved: {
                                                        // Don't send command while dragging
                                                    }

                                                    onPressedChanged: {
                                                        if (!pressed) {
                                                            // Send command on release
                                                            robotManager.sendJointCommand(jointIndex, value, 0.0);
                                                        }
                                                    }

                                                    background: Rectangle {
                                                        x: jointSlider.leftPadding
                                                        y: jointSlider.topPadding + jointSlider.availableHeight / 2 - height / 2
                                                        implicitWidth: 200
                                                        implicitHeight: 4
                                                        width: jointSlider.availableWidth
                                                        height: implicitHeight
                                                        radius: 2
                                                        color: "#404040"

                                                        Rectangle {
                                                            width: jointSlider.visualPosition * parent.width
                                                            height: parent.height
                                                            color: jointColor
                                                            radius: 2
                                                        }
                                                    }

                                                    handle: Rectangle {
                                                        x: jointSlider.leftPadding + jointSlider.visualPosition * (jointSlider.availableWidth - width)
                                                        y: jointSlider.topPadding + jointSlider.availableHeight / 2 - height / 2
                                                        implicitWidth: 16
                                                        implicitHeight: 16
                                                        radius: 25
                                                        color: jointSlider.pressed ? Qt.lighter(jointColor, 1.2) : jointColor
                                                        border.color: "#ffffff"
                                                        border.width: 2
                                                    }
                                                }
                                            }
                                        }

                                        // Background MouseArea for selection (lower z-order)
                                        MouseArea {
                                            anchors.fill: parent
                                            z: -1
                                            cursorShape: Qt.PointingHandCursor
                                            onClicked: {
                                                if (jointIndex < 7) {
                                                    // Toggle selection
                                                    var newSelection = selectedJoints.slice();
                                                    newSelection[jointIndex] = !newSelection[jointIndex];
                                                    selectedJoints = newSelection;

                                                    console.log("Toggled joint", jointIndex, "to", newSelection[jointIndex]);

                                                    // Force canvas repaints
                                                    positionCanvas.requestPaint();
                                                    velocityCanvas.requestPaint();
                                                    accelerationCanvas.requestPaint();
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // Charts area
                    Rectangle {
                        anchors.left: graphsSidebar.right
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.bottom: parent.bottom
                        color: "#121212"

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 20
                            spacing: 15

                            Text {
                                text: "Real-time Joint Data"
                                color: "white"
                                font.pixelSize: 24
                                font.bold: true
                                font.family: "Segoe UI"
                                Layout.alignment: Qt.AlignHCenter
                            }

                            // Position Chart
                            Rectangle {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                color: "#1e1e1e"
                                radius: 10
                                border.color: "#30ffffff"
                                border.width: 1

                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 15
                                    Row {
                                        Layout.fillWidth: true
                                        Layout.preferredHeight: 30
                                        spacing: 10

                                        Text {
                                            text: "Position"
                                            color: "#cccccc"
                                            font.pixelSize: 16
                                            font.bold: true
                                            anchors.verticalCenter: parent.verticalCenter
                                        }

                                        Item {
                                            width: 10
                                        }

                                        // Real-time values

                                        // Real-time values for selected joints
                                        Row {
                                            spacing: 12

                                            Repeater {
                                                model: 7

                                                Text {
                                                    visible: selectedJoints[index]
                                                    text: {
                                                        var _t = dataUpdateTrigger;
                                                        var val = window.getJointValueByIndex(index);
                                                        var displayVal = useRadians ? val : (val * 180 / Math.PI);
                                                        var unit = useRadians ? " rad" : "°";
                                                        return "J" + index + ": " + displayVal.toFixed(2) + unit;
                                                    }
                                                    color: jointColors[index]
                                                    font.pixelSize: 11
                                                    font.family: "Monospace"
                                                    font.bold: true
                                                }
                                            }
                                        }

                                        Item {
                                            Layout.fillWidth: true
                                        }

                                        // Deg/Rad toggle button
                                        Rectangle {
                                            width: 60
                                            height: 24
                                            radius: 12
                                            color: "#3a3a3a"
                                            border.color: "#666666"
                                            border.width: 1
                                            anchors.verticalCenter: parent.verticalCenter

                                            Text {
                                                text: useRadians ? "RAD" : "DEG"
                                                color: "#4CAF50"
                                                font.pixelSize: 11
                                                font.bold: true
                                                anchors.centerIn: parent
                                            }

                                            MouseArea {
                                                anchors.fill: parent
                                                cursorShape: Qt.PointingHandCursor
                                                onClicked: useRadians = !useRadians
                                            }
                                        }
                                    }

                                    Canvas {
                                        id: positionCanvas
                                        Layout.fillWidth: true
                                        Layout.fillHeight: true

                                        onPaint: {
                                            var ctx = getContext("2d");
                                            ctx.clearRect(0, 0, width, height);

                                            // Calculate zoom window
                                            var visibleWindow = maxDataPoints / graphTimeZoom;

                                            // Calculate auto-scale based on VISIBLE data only
                                            var pMin = 0, pMax = 0;
                                            var hasData = false;

                                            for (var j = 0; j < 7; j++) {
                                                if (!selectedJoints[j])
                                                    continue;
                                                var jointName = "joint" + j;
                                                if (!jointPositionData[jointName])
                                                    continue;
                                                var data = jointPositionData[jointName];
                                                if (data.length < 2)
                                                    continue;

                                                // Calculate visible range
                                                var totalAvailable = data.length;
                                                var effectiveVisible = Math.min(visibleWindow, totalAvailable);
                                                var startFloat = (totalAvailable - effectiveVisible) * graphTimeOffset;
                                                var endFloat = startFloat + effectiveVisible;
                                                var startIndex = Math.floor(Math.max(0, startFloat));
                                                var endIndex = Math.ceil(Math.min(totalAvailable, endFloat));

                                                // Find min/max in visible range only
                                                for (var k = startIndex; k < endIndex; k++) {
                                                    if (!hasData) {
                                                        pMin = data[k];
                                                        pMax = data[k];
                                                        hasData = true;
                                                    } else {
                                                        pMin = Math.min(pMin, data[k]);
                                                        pMax = Math.max(pMax, data[k]);
                                                    }
                                                }
                                            }

                                            // Add 10% padding to raw values
                                            var pRange = Math.max(0.1, pMax - pMin);
                                            var targetMin = pMin - pRange * 0.1;
                                            var targetMax = pMax + pRange * 0.1;

                                            // Apply exponential smoothing to prevent flickering
                                            var alpha = 0.2;  // Smoothing factor (0=no change, 1=instant)
                                            if (positionMinSmooth === 0 && positionMaxSmooth === 0) {
                                                // First frame - initialize
                                                positionMinSmooth = targetMin;
                                                positionMaxSmooth = targetMax;
                                            } else {
                                                // Smooth transition
                                                positionMinSmooth = positionMinSmooth * (1 - alpha) + targetMin * alpha;
                                                positionMaxSmooth = positionMaxSmooth * (1 - alpha) + targetMax * alpha;
                                            }

                                            var positionMin = positionMinSmooth;
                                            var positionMax = positionMaxSmooth;
                                            var range = positionMax - positionMin;
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
                                                var value = positionMax - (range * i / 5);
                                                ctx.fillStyle = "#888888";
                                                ctx.font = "10px monospace";
                                                ctx.fillText(value.toFixed(2), 2, y + 3);
                                            }

                                            // Draw joint data - consistent colors
                                            var colors = jointColors;

                                            for (var j = 0; j < 7; j++) {
                                                if (!selectedJoints[j])
                                                    continue;
                                                var jointName = "joint" + j;
                                                if (!jointPositionData[jointName])
                                                    continue;
                                                var data = jointPositionData[jointName];
                                                if (data.length < 2)
                                                    continue;

                                                // Determine data range to draw
                                                var totalAvailable = data.length;
                                                var effectiveVisible = Math.min(visibleWindow, totalAvailable);
                                                var startFloat = (totalAvailable - effectiveVisible) * graphTimeOffset;
                                                var endFloat = startFloat + effectiveVisible;

                                                var startIndex = Math.floor(Math.max(0, startFloat));
                                                var endIndex = Math.ceil(Math.min(totalAvailable, endFloat));

                                                // Optimize rendering for large datasets
                                                var graphWidth = width - 40;
                                                var step = 1;
                                                if (effectiveVisible > graphWidth) {
                                                    step = Math.ceil(effectiveVisible / graphWidth);
                                                }

                                                ctx.strokeStyle = colors[j];
                                                ctx.lineWidth = 2;
                                                ctx.beginPath();

                                                var firstPoint = true;
                                                for (var i = startIndex; i < endIndex; i += step) {
                                                    // Normalized X within the visible window
                                                    // (i - startFloat) goes from 0 to effectiveVisible
                                                    var xNorm = (i - startFloat) / effectiveVisible;
                                                    var x = 40 + (xNorm * graphWidth);

                                                    // Clip drawing to graph area
                                                    if (x < 40)
                                                        continue;
                                                    if (x > width)
                                                        break;
                                                    var normalized = (data[i] - positionMin) / range;
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

                                        MouseArea {
                                            anchors.fill: parent
                                            property real startX
                                            property real startOffset

                                            onWheel: {
                                                if (wheel.angleDelta.y > 0) {
                                                    graphTimeZoom = Math.min(graphTimeZoom * 1.2, 20.0); // Max 20x zoom
                                                } else {
                                                    graphTimeZoom = Math.max(graphTimeZoom * 0.8, 1.0); // Min 1x zoom
                                                }
                                                positionCanvas.requestPaint();
                                                velocityCanvas.requestPaint();
                                                accelerationCanvas.requestPaint();
                                            }

                                            onPressed: {
                                                startX = mouse.x;
                                                startOffset = graphTimeOffset;
                                            }

                                            onPositionChanged: {
                                                if (pressed) {
                                                    var deltaX = mouse.x - startX;
                                                    // Width of graph area is approx width - 40
                                                    var graphWidth = width - 40;
                                                    var deltaNorm = deltaX / graphWidth;

                                                    // When zoomed in, a drag of full width corresponds to 1 visibleWindow
                                                    // graphTimeOffset goes 0..1 covering the invisible range

                                                    // Sensitivity adjustment:
                                                    // If zoom=1, offset has no effect (visible=total), drag shouldn't move?
                                                    // If zoom=2, visible=0.5 total. offset moves the 0.5 window over 0.5 hidden.

                                                    if (graphTimeZoom > 1.0) {
                                                        // Move offset inverse to drag
                                                        var offsetDelta = deltaNorm / (1.0 - (1.0 / graphTimeZoom));
                                                        // Actually simpler: just scale appropriately
                                                        // Let's rely on empirical feel or simple fraction
                                                        var change = deltaNorm / (graphTimeZoom - 1.0) * -1.0;

                                                        graphTimeOffset = Math.max(0.0, Math.min(startOffset + change, 1.0));

                                                        positionCanvas.requestPaint();
                                                        velocityCanvas.requestPaint();
                                                        accelerationCanvas.requestPaint();
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            // Velocity Chart
                            Rectangle {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                color: "#1e1e1e"
                                radius: 10
                                border.color: "#30ffffff"
                                border.width: 1

                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 15
                                    Row {
                                        Layout.fillWidth: true
                                        Layout.preferredHeight: 30
                                        spacing: 10

                                        Text {
                                            text: "Velocity"
                                            color: "#cccccc"
                                            font.pixelSize: 16
                                            font.bold: true
                                            anchors.verticalCenter: parent.verticalCenter
                                        }

                                        Item {
                                            width: 10
                                        }

                                        // Real-time values
                                        Row {
                                            spacing: 8
                                            anchors.verticalCenter: parent.verticalCenter

                                            Repeater {
                                                model: 7

                                                Text {
                                                    visible: selectedJoints[index]
                                                    text: {
                                                        var _t = dataUpdateTrigger;
                                                        var jointName = "joint" + index;
                                                        if (jointVelocityData[jointName] && jointVelocityData[jointName].length > 0) {
                                                            var val = jointVelocityData[jointName][jointVelocityData[jointName].length - 1];
                                                            var converted = useRadians ? val : (val * 180 / Math.PI);
                                                            return "J" + index + ": " + converted.toFixed(2) + (useRadians ? " rad/s" : "°/s");
                                                        }
                                                        return "J" + index + ": --";
                                                    }
                                                    color: jointColors[index]
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
                                        id: velocityCanvas
                                        Layout.fillWidth: true
                                        Layout.fillHeight: true

                                        onPaint: {
                                            var ctx = getContext("2d");
                                            ctx.clearRect(0, 0, width, height);

                                            // Calculate zoom window
                                            var visibleWindow = maxDataPoints / graphTimeZoom;

                                            // Calculate auto-scale based on VISIBLE data only
                                            var vMin = 0, vMax = 0;
                                            var hasData = false;

                                            for (var j = 0; j < 7; j++) {
                                                if (!selectedJoints[j])
                                                    continue;
                                                var jointName = "joint" + j;
                                                if (!jointVelocityData[jointName])
                                                    continue;
                                                var data = jointVelocityData[jointName];
                                                if (data.length < 2)
                                                    continue;

                                                // Calculate visible range
                                                var totalAvailable = data.length;
                                                var effectiveVisible = Math.min(visibleWindow, totalAvailable);
                                                var startFloat = (totalAvailable - effectiveVisible) * graphTimeOffset;
                                                var endFloat = startFloat + effectiveVisible;
                                                var startIndex = Math.floor(Math.max(0, startFloat));
                                                var endIndex = Math.ceil(Math.min(totalAvailable, endFloat));

                                                // Find min/max in visible range only
                                                for (var k = startIndex; k < endIndex; k++) {
                                                    if (!hasData) {
                                                        vMin = data[k];
                                                        vMax = data[k];
                                                        hasData = true;
                                                    } else {
                                                        vMin = Math.min(vMin, data[k]);
                                                        vMax = Math.max(vMax, data[k]);
                                                    }
                                                }
                                            }

                                            // Add 10% padding to raw values
                                            var vRange = Math.max(0.1, vMax - vMin);
                                            var targetMin = vMin - vRange * 0.1;
                                            var targetMax = vMax + vRange * 0.1;

                                            // Apply exponential smoothing to prevent flickering
                                            var alpha = 0.2;  // Smoothing factor
                                            if (velocityMinSmooth === 0 && velocityMaxSmooth === 0) {
                                                velocityMinSmooth = targetMin;
                                                velocityMaxSmooth = targetMax;
                                            } else {
                                                velocityMinSmooth = velocityMinSmooth * (1 - alpha) + targetMin * alpha;
                                                velocityMaxSmooth = velocityMaxSmooth * (1 - alpha) + targetMax * alpha;
                                            }

                                            var velocityMin = velocityMinSmooth;
                                            var velocityMax = velocityMaxSmooth;
                                            var range = velocityMax - velocityMin;
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

                                                var value = velocityMax - (range * i / 5);
                                                ctx.fillStyle = "#888888";
                                                ctx.font = "10px monospace";
                                                ctx.fillText(value.toFixed(2), 2, y + 3);
                                            }

                                            var colors = jointColors;

                                            for (var j = 0; j < 7; j++) {
                                                if (!selectedJoints[j])
                                                    continue;
                                                var jointName = "joint" + j;
                                                if (!jointVelocityData[jointName])
                                                    continue;
                                                var data = jointVelocityData[jointName];
                                                if (data.length < 2)
                                                    continue;
                                                var totalAvailable = data.length;
                                                var effectiveVisible = Math.min(visibleWindow, totalAvailable);
                                                var startFloat = (totalAvailable - effectiveVisible) * graphTimeOffset;
                                                var endFloat = startFloat + effectiveVisible;

                                                var startIndex = Math.floor(Math.max(0, startFloat));
                                                var endIndex = Math.ceil(Math.min(totalAvailable, endFloat));

                                                // Optimize rendering for large datasets
                                                var graphWidth = width - 40;
                                                var step = 1;
                                                if (effectiveVisible > graphWidth) {
                                                    step = Math.ceil(effectiveVisible / graphWidth);
                                                }

                                                ctx.strokeStyle = colors[j];
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
                                                    var normalized = (data[i] - velocityMin) / range;
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

                                        MouseArea {
                                            anchors.fill: parent
                                            property real startX
                                            property real startOffset
                                            onWheel: {
                                                if (wheel.angleDelta.y > 0)
                                                    graphTimeZoom = Math.min(graphTimeZoom * 1.2, 20.0);
                                                else
                                                    graphTimeZoom = Math.max(graphTimeZoom * 0.8, 1.0);
                                                positionCanvas.requestPaint();
                                                velocityCanvas.requestPaint();
                                                accelerationCanvas.requestPaint();
                                            }
                                            onPressed: {
                                                startX = mouse.x;
                                                startOffset = graphTimeOffset;
                                            }
                                            onPositionChanged: {
                                                if (pressed && graphTimeZoom > 1.0) {
                                                    var deltaNorm = (mouse.x - startX) / (width - 40);
                                                    var change = deltaNorm / (graphTimeZoom - 1.0) * -1.0;
                                                    graphTimeOffset = Math.max(0.0, Math.min(startOffset + change, 1.0));
                                                    positionCanvas.requestPaint();
                                                    velocityCanvas.requestPaint();
                                                    accelerationCanvas.requestPaint();
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            // Acceleration Chart
                            Rectangle {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                color: "#1e1e1e"
                                radius: 10
                                border.color: "#30ffffff"
                                border.width: 1

                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 15
                                    Row {
                                        Layout.fillWidth: true
                                        Layout.preferredHeight: 30
                                        spacing: 10

                                        Text {
                                            text: "Acceleration"
                                            color: "#cccccc"
                                            font.pixelSize: 16
                                            font.bold: true
                                            anchors.verticalCenter: parent.verticalCenter
                                        }

                                        Item {
                                            width: 10
                                        }

                                        // Real-time values
                                        Row {
                                            spacing: 8
                                            anchors.verticalCenter: parent.verticalCenter

                                            Repeater {
                                                model: 7

                                                Text {
                                                    visible: selectedJoints[index]
                                                    text: {
                                                        var _t = dataUpdateTrigger;
                                                        var jointName = "joint" + index;
                                                        if (jointAccelerationData[jointName] && jointAccelerationData[jointName].length > 0) {
                                                            var val = jointAccelerationData[jointName][jointAccelerationData[jointName].length - 1];
                                                            var converted = useRadians ? val : (val * 180 / Math.PI);
                                                            return "J" + index + ": " + converted.toFixed(2) + (useRadians ? " rad/s²" : "°/s²");
                                                        }
                                                        return "J" + index + ": --";
                                                    }
                                                    color: jointColors[index]
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
                                        id: accelerationCanvas
                                        Layout.fillWidth: true
                                        Layout.fillHeight: true

                                        onPaint: {
                                            var ctx = getContext("2d");
                                            ctx.clearRect(0, 0, width, height);

                                            // Calculate zoom window
                                            var visibleWindow = maxDataPoints / graphTimeZoom;

                                            // Calculate auto-scale based on VISIBLE data only
                                            var aMin = 0, aMax = 0;
                                            var hasData = false;

                                            for (var j = 0; j < 7; j++) {
                                                if (!selectedJoints[j])
                                                    continue;
                                                var jointName = "joint" + j;
                                                if (!jointAccelerationData[jointName])
                                                    continue;
                                                var data = jointAccelerationData[jointName];
                                                if (data.length < 2)
                                                    continue;

                                                // Calculate visible range
                                                var totalAvailable = data.length;
                                                var effectiveVisible = Math.min(visibleWindow, totalAvailable);
                                                var startFloat = (totalAvailable - effectiveVisible) * graphTimeOffset;
                                                var endFloat = startFloat + effectiveVisible;
                                                var startIndex = Math.floor(Math.max(0, startFloat));
                                                var endIndex = Math.ceil(Math.min(totalAvailable, endFloat));

                                                // Find min/max in visible range only
                                                for (var k = startIndex; k < endIndex; k++) {
                                                    if (!hasData) {
                                                        aMin = data[k];
                                                        aMax = data[k];
                                                        hasData = true;
                                                    } else {
                                                        aMin = Math.min(aMin, data[k]);
                                                        aMax = Math.max(aMax, data[k]);
                                                    }
                                                }
                                            }

                                            // Add 10% padding to raw values
                                            var aRange = Math.max(0.1, aMax - aMin);
                                            var targetMin = aMin - aRange * 0.1;
                                            var targetMax = aMax + aRange * 0.1;

                                            // Apply exponential smoothing to prevent flickering
                                            var alpha = 0.2;  // Smoothing factor
                                            if (window.accelerationMinSmooth === 0 && window.accelerationMaxSmooth === 0) {
                                                window.accelerationMinSmooth = targetMin;
                                                window.accelerationMaxSmooth = targetMax;
                                            } else {
                                                window.accelerationMinSmooth = window.accelerationMinSmooth * (1 - alpha) + targetMin * alpha;
                                                window.accelerationMaxSmooth = window.accelerationMaxSmooth * (1 - alpha) + targetMax * alpha;
                                            }

                                            var accelerationMin = window.accelerationMinSmooth;
                                            var accelerationMax = window.accelerationMaxSmooth;
                                            var range = accelerationMax - accelerationMin;
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

                                                var value = accelerationMax - (range * i / 5);
                                                ctx.fillStyle = "#888888";
                                                ctx.font = "10px monospace";
                                                ctx.fillText(value.toFixed(2), 2, y + 3);
                                            }

                                            var colors = jointColors;

                                            for (var j = 0; j < 7; j++) {
                                                if (!selectedJoints[j])
                                                    continue;
                                                var jointName = "joint" + j;
                                                if (!jointAccelerationData[jointName])
                                                    continue;
                                                var data = jointAccelerationData[jointName];
                                                if (data.length < 2)
                                                    continue;
                                                var totalAvailable = data.length;
                                                var effectiveVisible = Math.min(visibleWindow, totalAvailable);
                                                var startFloat = (totalAvailable - effectiveVisible) * graphTimeOffset;
                                                var endFloat = startFloat + effectiveVisible;

                                                var startIndex = Math.floor(Math.max(0, startFloat));
                                                var endIndex = Math.ceil(Math.min(totalAvailable, endFloat));

                                                // Optimize rendering for large datasets
                                                var graphWidth = width - 40;
                                                var step = 1;
                                                if (effectiveVisible > graphWidth) {
                                                    step = Math.ceil(effectiveVisible / graphWidth);
                                                }

                                                ctx.strokeStyle = colors[j];
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
                                                    var normalized = (data[i] - accelerationMin) / range;
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

                                        MouseArea {
                                            anchors.fill: parent
                                            property real startX
                                            property real startOffset
                                            onWheel: {
                                                if (wheel.angleDelta.y > 0)
                                                    graphTimeZoom = Math.min(graphTimeZoom * 1.2, 20.0);
                                                else
                                                    graphTimeZoom = Math.max(graphTimeZoom * 0.8, 1.0);
                                                positionCanvas.requestPaint();
                                                velocityCanvas.requestPaint();
                                                accelerationCanvas.requestPaint();
                                            }
                                            onPressed: {
                                                startX = mouse.x;
                                                startOffset = graphTimeOffset;
                                            }
                                            onPositionChanged: {
                                                if (pressed && graphTimeZoom > 1.0) {
                                                    var deltaNorm = (mouse.x - startX) / (width - 40);
                                                    var change = deltaNorm / (graphTimeZoom - 1.0) * -1.0;
                                                    graphTimeOffset = Math.max(0.0, Math.min(startOffset + change, 1.0));
                                                    positionCanvas.requestPaint();
                                                    velocityCanvas.requestPaint();
                                                    accelerationCanvas.requestPaint();
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // Tab 3: Settings
                Rectangle {
                    color: "#2a2a2a"

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
                                        text: selectedDevice
                                        color: "#ffffff"
                                        font.pixelSize: 14
                                        font.family: "Monospace"
                                        anchors.verticalCenter: parent.verticalCenter
                                    }

                                    Item {
                                        Layout.fillWidth: true
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
                                        var ports = robotManager.getAvailablePorts();
                                        if (ports.length > 0) {
                                            selectedDevice = ports[0];
                                            console.log("Selected device:", selectedDevice);
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
                                        text: selectedBaudrate.toString()
                                        color: "#ffffff"
                                        font.pixelSize: 14
                                        font.family: "Monospace"
                                        anchors.verticalCenter: parent.verticalCenter
                                    }

                                    Item {
                                        Layout.fillWidth: true
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
                                        currentIndex = (currentIndex + 1) % availableBaudrates.length;
                                        selectedBaudrate = availableBaudrates[currentIndex];
                                        console.log("Selected baudrate:", selectedBaudrate);
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
                            color: "#4CAF50"

                            Text {
                                text: "Save Configuration"
                                anchors.centerIn: parent
                                color: "white"
                                font.pixelSize: 14
                                font.bold: true
                            }

                            MouseArea {
                                anchors.fill: parent
                                cursorShape: Qt.PointingHandCursor
                                onClicked: {
                                    // Save configuration to JSON file
                                    var config = {
                                        "device": selectedDevice,
                                        "baudrate": selectedBaudrate
                                    };
                                    var configStr = JSON.stringify(config, null, 2);
                                    console.log("Saving configuration:", configStr);

                                    // Connect to robot with new settings
                                    if (!isSimulationMode) {
                                        robotManager.connectRobot(selectedDevice, selectedBaudrate);
                                    }
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

                        // Separator
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredWidth: 500
                            Layout.alignment: Qt.AlignHCenter
                            Layout.topMargin: 20
                            height: 1
                            color: "#40ffffff"
                        }

                        // IK Gains Section
                        Text {
                            text: "IK Tuning"
                            color: "white"
                            font.pixelSize: 20
                            font.bold: true
                            font.family: "Segoe UI"
                            Layout.alignment: Qt.AlignHCenter
                            Layout.topMargin: 10
                        }

                        // Kp Slider
                        ColumnLayout {
                            Layout.fillWidth: true
                            Layout.preferredWidth: 500
                            Layout.alignment: Qt.AlignHCenter
                            spacing: 5

                            RowLayout {
                                Layout.fillWidth: true
                                Text {
                                    text: "Position Gain (Kp)"
                                    color: "#cccccc"
                                    font.pixelSize: 14
                                }
                                Item {
                                    Layout.fillWidth: true
                                }
                                Text {
                                    id: kpValueText
                                    text: kpSlider.value.toFixed(1)
                                    color: "#4CAF50"
                                    font.pixelSize: 14
                                    font.bold: true
                                    font.family: "Monospace"
                                }
                            }

                            Slider {
                                id: kpSlider
                                Layout.fillWidth: true
                                Layout.preferredHeight: 30
                                from: 0.5
                                to: 10.0
                                value: 5.0
                                stepSize: 0.5
                                live: true
                                onMoved: {
                                    if (robotNode) {
                                        robotNode.setSqpGains(value, kdSlider.value);
                                    }
                                }

                                background: Rectangle {
                                    x: kpSlider.leftPadding
                                    y: kpSlider.topPadding + kpSlider.availableHeight / 2 - height / 2
                                    width: kpSlider.availableWidth
                                    height: 6
                                    radius: 3
                                    color: "#1e1e1e"

                                    Rectangle {
                                        width: kpSlider.visualPosition * parent.width
                                        height: parent.height
                                        color: "#4CAF50"
                                        radius: 3
                                    }
                                }

                                handle: Rectangle {
                                    x: kpSlider.leftPadding + kpSlider.visualPosition * (kpSlider.availableWidth - width)
                                    y: kpSlider.topPadding + kpSlider.availableHeight / 2 - height / 2
                                    width: 20
                                    height: 20
                                    radius: 10
                                    color: kpSlider.pressed ? "#66BB6A" : "#4CAF50"
                                    border.color: "#ffffff"
                                    border.width: 2
                                }
                            }

                            Text {
                                text: "Higher = faster response (risk of overshoot)"
                                color: "#888888"
                                font.pixelSize: 10
                                font.italic: true
                            }
                        }

                        // Kd Slider
                        ColumnLayout {
                            Layout.fillWidth: true
                            Layout.preferredWidth: 500
                            Layout.alignment: Qt.AlignHCenter
                            spacing: 5

                            RowLayout {
                                Layout.fillWidth: true
                                Text {
                                    text: "Damping Gain (Kd)"
                                    color: "#cccccc"
                                    font.pixelSize: 14
                                }
                                Item {
                                    Layout.fillWidth: true
                                }
                                Text {
                                    id: kdValueText
                                    text: kdSlider.value.toFixed(1)
                                    color: "#2196F3"
                                    font.pixelSize: 14
                                    font.bold: true
                                    font.family: "Monospace"
                                }
                            }

                            Slider {
                                id: kdSlider
                                Layout.fillWidth: true
                                Layout.preferredHeight: 30
                                from: 0.5
                                to: 10.0
                                value: 7.0
                                stepSize: 0.5
                                live: true
                                onMoved: {
                                    if (robotNode) {
                                        robotNode.setSqpGains(kpSlider.value, value);
                                    }
                                }

                                background: Rectangle {
                                    x: kdSlider.leftPadding
                                    y: kdSlider.topPadding + kdSlider.availableHeight / 2 - height / 2
                                    width: kdSlider.availableWidth
                                    height: 6
                                    radius: 3
                                    color: "#1e1e1e"

                                    Rectangle {
                                        width: kdSlider.visualPosition * parent.width
                                        height: parent.height
                                        color: "#2196F3"
                                        radius: 3
                                    }
                                }

                                handle: Rectangle {
                                    x: kdSlider.leftPadding + kdSlider.visualPosition * (kdSlider.availableWidth - width)
                                    y: kdSlider.topPadding + kdSlider.availableHeight / 2 - height / 2
                                    width: 20
                                    height: 20
                                    radius: 10
                                    color: kdSlider.pressed ? "#42A5F5" : "#2196F3"
                                    border.color: "#ffffff"
                                    border.width: 2
                                }
                            }

                            Text {
                                text: "Higher = less overshoot (Kd ≥ 2√Kp for no overshoot)"
                                color: "#888888"
                                font.pixelSize: 10
                                font.italic: true
                            }

                            // Critical damping indicator
                            Text {
                                text: {
                                    var criticalKd = 2 * Math.sqrt(kpSlider.value);
                                    if (kdSlider.value >= criticalKd) {
                                        return "✓ Overdamped (no overshoot)";
                                    } else {
                                        return "⚠ Underdamped - may overshoot (need Kd ≥ " + criticalKd.toFixed(1) + ")";
                                    }
                                }
                                color: kdSlider.value >= 2 * Math.sqrt(kpSlider.value) ? "#4CAF50" : "#FF9800"
                                font.pixelSize: 11
                                font.bold: true
                                Layout.topMargin: 5
                            }
                        }

                        Item {
                            Layout.fillHeight: true
                        }
                    }
                }

                // Tab 4: Driver Settings
                Item {
                    anchors.fill: parent

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
                                model: currentLinks
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
                                color: "#FF5722" // Orange color for reset

                                Text {
                                    text: "Reset All Joints"
                                    anchors.centerIn: parent
                                    color: "white"
                                    font.pixelSize: 14
                                    font.bold: true
                                }

                                MouseArea {
                                    anchors.fill: parent
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: {
                                        console.log("Reset all joints to default position");

                                        if (isSimulationMode) {
                                            // Reset simulation positions
                                            simulationJointPositions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

                                            // Reset 3D model angles
                                            var angles = {};
                                            for (var i = 0; i < currentLinks.length; i++) {
                                                if (currentLinks[i].hasJoint) {
                                                    angles[currentLinks[i].jointName] = 0.0;
                                                }
                                            }
                                            jointAngles = angles;
                                        } else {
                                            // Send reset commands for all joints to MCU
                                            for (var i = 0; i < 7; i++) {
                                                robotManager.sendJointCommand(i, 0.0, 0.0);
                                            }

                                            // Reset 3D model angles
                                            var angles = {};
                                            for (var i = 0; i < currentLinks.length; i++) {
                                                if (currentLinks[i].hasJoint) {
                                                    angles[currentLinks[i].jointName] = 0.0;
                                                }
                                            }
                                            jointAngles = angles;

                                            // Reset gizmo to track EE position as robot moves
                                            transformGizmo.hasBeenDragged = false;
                                            robotGizmoHasBeenDragged = false;
                                            transformGizmo.updatePosition(true);
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
            }

            // =====================================================
            // RIGHT PANEL - Logs / Motion Planner (Switchable)
            // =====================================================
            Rectangle {
                id: logsPanelRight
                Layout.fillHeight: true
                Layout.preferredWidth: logsPanelExpanded ? 450 : 40
                color: "#1a1a1a"

                property bool logsPanelExpanded: true
                property int activeTab: 0  // 0 = Logs, 1 = Motion Planner
                property var logMessages: []

                function clearLogs() {
                    logMessages = [];
                }

                function addLog(level, message) {
                    var now = new Date();
                    var timestamp = now.toTimeString().substr(0, 8);
                    var epochMs = now.getTime();
                    var newLogs = logMessages.slice();
                    newLogs.push({
                        timestamp: timestamp,
                        epochMs: epochMs,
                        level: level,
                        message: message
                    });
                    if (newLogs.length > 400) {
                        newLogs = newLogs.slice(-400);
                    }
                    logMessages = newLogs;
                }

                // Left border
                Rectangle {
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    width: 1
                    color: "#30ffffff"
                }

                RowLayout {
                    anchors.fill: parent
                    spacing: 0

                    // Collapsed state
                    Rectangle {
                        Layout.fillHeight: true
                        Layout.preferredWidth: 40
                        color: "#252525"
                        visible: !logsPanelRight.logsPanelExpanded

                        Column {
                            anchors.centerIn: parent
                            spacing: 8

                            Rectangle {
                                width: 24
                                height: 24
                                radius: 4
                                color: expandBtnRightMA.containsMouse ? "#404040" : "transparent"
                                anchors.horizontalCenter: parent.horizontalCenter

                                Text {
                                    anchors.centerIn: parent
                                    text: "<"
                                    color: "#888888"
                                    font.pixelSize: 12
                                    font.bold: true
                                }

                                MouseArea {
                                    id: expandBtnRightMA
                                    anchors.fill: parent
                                    hoverEnabled: true
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: logsPanelRight.logsPanelExpanded = true
                                }
                            }

                            Text {
                                text: "L\nO\nG\nS"
                                color: "#888888"
                                font.pixelSize: 10
                                font.bold: true
                                horizontalAlignment: Text.AlignHCenter
                            }

                            Rectangle {
                                width: 24
                                height: 24
                                radius: 12
                                color: "#30ffffff"
                                visible: logsPanelRight.logMessages.length > 0
                                anchors.horizontalCenter: parent.horizontalCenter

                                Text {
                                    anchors.centerIn: parent
                                    text: logsPanelRight.logMessages.length > 99 ? "99+" : logsPanelRight.logMessages.length
                                    color: "#888888"
                                    font.pixelSize: 9
                                }
                            }
                        }
                    }

                    // Expanded state
                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: 0
                        visible: logsPanelRight.logsPanelExpanded

                        // Header with tab bar
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40
                            color: "#252525"

                            RowLayout {
                                anchors.fill: parent
                                anchors.leftMargin: 8
                                anchors.rightMargin: 8
                                spacing: 4

                                // Collapse button
                                Rectangle {
                                    width: 24
                                    height: 24
                                    radius: 4
                                    color: collapseBtnRightMA.containsMouse ? "#404040" : "transparent"

                                    Text {
                                        anchors.centerIn: parent
                                        text: ">"
                                        color: "#888888"
                                        font.pixelSize: 12
                                        font.bold: true
                                    }

                                    MouseArea {
                                        id: collapseBtnRightMA
                                        anchors.fill: parent
                                        hoverEnabled: true
                                        cursorShape: Qt.PointingHandCursor
                                        onClicked: logsPanelRight.logsPanelExpanded = false
                                    }
                                }

                                // Tab: Logs (takes half width)
                                Rectangle {
                                    Layout.fillWidth: true
                                    height: 32
                                    radius: 6
                                    color: logsPanelRight.activeTab === 0 ? "#404040" : (logsTabMA.containsMouse ? "#353535" : "#252525")
                                    border.color: logsPanelRight.activeTab === 0 ? "#42A5F5" : "transparent"
                                    border.width: 1

                                    Row {
                                        anchors.centerIn: parent
                                        spacing: 6

                                        Text {
                                            id: logsTabText
                                            text: "Logs"
                                            color: logsPanelRight.activeTab === 0 ? "#ffffff" : "#888888"
                                            font.pixelSize: 13
                                            font.bold: true
                                        }

                                        // Log count badge
                                        Rectangle {
                                            width: logBadgeText.width + 8
                                            height: 18
                                            radius: 9
                                            color: "#FF5252"
                                            visible: logsPanelRight.logMessages.length > 0

                                            Text {
                                                id: logBadgeText
                                                anchors.centerIn: parent
                                                text: logsPanelRight.logMessages.length > 99 ? "99+" : logsPanelRight.logMessages.length
                                                color: "white"
                                                font.pixelSize: 10
                                                font.bold: true
                                            }
                                        }
                                    }

                                    MouseArea {
                                        id: logsTabMA
                                        anchors.fill: parent
                                        hoverEnabled: true
                                        cursorShape: Qt.PointingHandCursor
                                        onClicked: logsPanelRight.activeTab = 0
                                    }
                                }

                                // Tab: Motion Planner (takes half width)
                                Rectangle {
                                    Layout.fillWidth: true
                                    height: 32
                                    radius: 6
                                    color: logsPanelRight.activeTab === 1 ? "#404040" : (motionTabMA.containsMouse ? "#353535" : "#252525")
                                    border.color: logsPanelRight.activeTab === 1 ? "#FF9800" : "transparent"
                                    border.width: 1

                                    Text {
                                        id: motionTabText
                                        anchors.centerIn: parent
                                        text: "Motion"
                                        color: logsPanelRight.activeTab === 1 ? "#ffffff" : "#888888"
                                        font.pixelSize: 13
                                        font.bold: true
                                    }

                                    MouseArea {
                                        id: motionTabMA
                                        anchors.fill: parent
                                        hoverEnabled: true
                                        cursorShape: Qt.PointingHandCursor
                                        onClicked: logsPanelRight.activeTab = 1
                                    }
                                }

                                // Clear button (only for Logs tab)
                                Rectangle {
                                    width: clearLogsBtnRightText.width + 16
                                    height: 24
                                    radius: 12
                                    color: clearLogsBtnRightMA.containsMouse ? "#FF5252" : "#404040"
                                    visible: logsPanelRight.activeTab === 0

                                    Text {
                                        id: clearLogsBtnRightText
                                        anchors.centerIn: parent
                                        text: "Clear"
                                        color: clearLogsBtnRightMA.containsMouse ? "white" : "#cccccc"
                                        font.pixelSize: 11
                                        font.family: "Segoe UI"
                                    }

                                    MouseArea {
                                        id: clearLogsBtnRightMA
                                        anchors.fill: parent
                                        hoverEnabled: true
                                        cursorShape: Qt.PointingHandCursor
                                        onClicked: logsPanelRight.clearLogs()
                                    }
                                }
                            }
                        }

                        // LOGS CONTENT (visible when activeTab === 0)
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: "#1a1a1a"
                            clip: true
                            visible: logsPanelRight.activeTab === 0

                            ListView {
                                id: logsListViewRight
                                anchors.fill: parent
                                anchors.margins: 8
                                model: logsPanelRight.logMessages
                                spacing: 2

                                onCountChanged: {
                                    Qt.callLater(function () {
                                        logsListViewRight.positionViewAtEnd();
                                    });
                                }

                                delegate: Column {
                                    width: logsListViewRight.width
                                    spacing: 2

                                    // Check if we should show header (timestamp + level)
                                    // Hide if previous log has same level and is within 10 seconds
                                    property bool showHeader: {
                                        if (index === 0)
                                            return true;
                                        var prevLog = logsPanelRight.logMessages[index - 1];
                                        if (!prevLog)
                                            return true;
                                        if (prevLog.level !== modelData.level)
                                            return true;
                                        var timeDiff = modelData.epochMs - prevLog.epochMs;
                                        return timeDiff > 10000; // 10 seconds
                                    }

                                    Row {
                                        spacing: 6
                                        visible: showHeader
                                        height: showHeader ? implicitHeight : 0

                                        Text {
                                            text: modelData.timestamp
                                            color: "#666666"
                                            font.pixelSize: 10
                                            font.family: "Monospace"
                                        }

                                        Rectangle {
                                            width: 40
                                            height: 14
                                            radius: 3
                                            color: modelData.level === "ERROR" ? "#802020" : modelData.level === "WARN" ? "#806020" : modelData.level === "DEBUG" ? "#205080" : "#206020"

                                            Text {
                                                anchors.centerIn: parent
                                                text: modelData.level
                                                color: modelData.level === "ERROR" ? "#FF6666" : modelData.level === "WARN" ? "#FFAA66" : modelData.level === "DEBUG" ? "#66AAFF" : "#66FF66"
                                                font.pixelSize: 8
                                                font.bold: true
                                            }
                                        }
                                    }

                                    Text {
                                        text: modelData.message
                                        color: "#bbbbbb"
                                        font.pixelSize: 11
                                        font.family: "Segoe UI"
                                        wrapMode: Text.Wrap
                                        width: logsListViewRight.width - 10
                                        leftPadding: showHeader ? 0 : 10
                                    }
                                }

                                Text {
                                    anchors.centerIn: parent
                                    text: "No logs"
                                    color: "#555555"
                                    font.pixelSize: 12
                                    font.italic: true
                                    visible: logsPanelRight.logMessages.length === 0
                                }
                            }
                        }

                        // MOTION PLANNER CONTENT (visible when activeTab === 1)
                        Item {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            visible: logsPanelRight.activeTab === 1

                            Panels.MotionPlannerPanel {
                                id: motionPlannerRight
                                anchors.fill: parent
                                waypointModel: window.waypointModel
                                getGizmoPoseURDF: window.getGizmoPoseURDF
                                motionController: motionController
                                robotNode: robotNode

                                onPreviewPath: {
                                    console.log("Preview path requested");
                                }

                                onExecutePath: {
                                    console.log("Execute path requested");
                                    var waypoints = [];
                                    for (var i = 0; i < window.waypointModel.count; i++) {
                                        var wp = window.waypointModel.get(i);
                                        waypoints.push({
                                            "name": wp.name,
                                            "posX": wp.posX,
                                            "posY": wp.posY,
                                            "posZ": wp.posZ,
                                            "quatW": wp.quatW,
                                            "quatX": wp.quatX,
                                            "quatY": wp.quatY,
                                            "quatZ": wp.quatZ,
                                            "motionType": wp.motionType,
                                            "speed": wp.speed
                                        });
                                    }
                                    motionController.executeWaypoints(waypoints);
                                }
                            }
                        }
                    }
                }

                Behavior on Layout.preferredWidth {
                    NumberAnimation {
                        duration: 150
                        easing.type: Easing.OutQuad
                    }
                }
            }
        }
    }

    // Helper function: Create quaternion from axis-angle rotation
    function quaternionFromAxisAngle(axis, angleRad) {
        var halfAngle = angleRad / 2.0;
        var s = Math.sin(halfAngle);
        return Qt.quaternion(Math.cos(halfAngle)  // w
        , axis.x * s            // x
        , axis.y * s            // y
        , axis.z * s             // z
        );
    }

    // Helper function: Multiply two quaternions
    function multiplyQuaternions(q1, q2) {
        return Qt.quaternion(q1.scalar * q2.scalar - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z, q1.scalar * q2.x + q1.x * q2.scalar + q1.y * q2.z - q1.z * q2.y, q1.scalar * q2.y - q1.x * q2.z + q1.y * q2.scalar + q1.z * q2.x, q1.scalar * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.scalar);
    }

    function findNode(root, name) {
        if (root.objectName === name)
            return root;
        for (var i = 0; i < root.children.length; i++) {
            var child = root.children[i];
            var found = findNode(child, name);
            if (found)
                return found;
        }
        return null;
    }

    Component.onCompleted: {
        console.log("Window created. RobotNode:", robotNode);
        if (appArgs.length > 1) {
            robotNode.loadURDF(appArgs[1]);
        } else {
            console.warn("No URDF file provided!");
        }
    }
}
