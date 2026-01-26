import QtQuick
import QtQuick3D

// WaypointMarkers - 3D visualization of motion planner waypoints
// Used as child of View3D scene
Repeater3D {
    id: root

    // Required property - the waypoint data from WaypointListModel
    required property var waypointData

    model: waypointData

    Component.onCompleted: console.log("WaypointMarkers instantiated")

    Node {
        id: waypointMarkerNode
        property bool isWaypoint: true
        required property int index  // Model injects this automatically
        required property bool isSelected  // Model role
        required property real posX
        required property real posY
        required property real posZ
        required property int motionType
        onIsSelectedChanged: console.log("Marker " + index + " isSelected changed to: " + isSelected)
        Component.onCompleted: console.log("Creating marker for:", posX, posY, posZ)

        // Convert URDF coordinates to scene coordinates
        // Scene has robotRoot with scale=100 and eulerRotation.x=-90
        // URDF (posX,posY,posZ) -> Scene (posX*100, posZ*100, -posY*100)
        position: Qt.vector3d(posX * 100, posZ * 100, -posY * 100)

        // Sphere marker (small dot)
        Model {
            source: "#Sphere"
            scale: Qt.vector3d(0.015, 0.015, 0.015)  // Scaled up 3x from 0.005 -> 0.015
            materials: DefaultMaterial {
                diffuseColor: {
                    if (isSelected)
                        return "#ffffff";
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
                lighting: DefaultMaterial.NoLighting
            }
            pickable: true  // Enable picking
            opacity: isSelected ? 1.0 : 0.8
        }

        // Axes for selected waypoint
        Node {
            visible: isSelected

            // X Axis (Red)
            Model {
                source: "#Cylinder"
                position: Qt.vector3d(5, 0, 0)
                eulerRotation.z: -90
                scale: Qt.vector3d(0.02, 0.1, 0.02)
                materials: DefaultMaterial {
                    diffuseColor: "#f44336"
                    lighting: DefaultMaterial.NoLighting
                }
            }

            // Y Axis (Green) - Up in scene frame
            Model {
                source: "#Cylinder"
                position: Qt.vector3d(0, 5, 0)
                scale: Qt.vector3d(0.02, 0.1, 0.02)
                materials: DefaultMaterial {
                    diffuseColor: "#4CAF50"
                    lighting: DefaultMaterial.NoLighting
                }
            }

            // Z Axis (Blue)
            Model {
                source: "#Cylinder"
                position: Qt.vector3d(0, 0, 5)
                eulerRotation.x: 90
                scale: Qt.vector3d(0.02, 0.1, 0.02)
                materials: DefaultMaterial {
                    diffuseColor: "#2196F3"
                    lighting: DefaultMaterial.NoLighting
                }
            }
        }
    }
}
