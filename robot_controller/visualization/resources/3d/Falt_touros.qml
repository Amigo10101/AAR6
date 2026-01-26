import QtQuick
import QtQuick3D

Node {
    // Materials
    DefaultMaterial {
        id: defaultMaterial_material
    }
    // end of Materials

    Node {
        id: _STL_BINARY_
        Model {
            source: "meshes/node.mesh"
            materials: [
                defaultMaterial_material
            ]
        }
    }
}
