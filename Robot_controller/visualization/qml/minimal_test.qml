import QtQuick
import QtQuick.Window

Window {
    width: 800
    height: 600
    visible: true
    title: "Minimum Test"
    color: "red"
    
    Rectangle {
        anchors.centerIn: parent
        width: 200
        height: 200
        color: "white"
        
        Text {
            anchors.centerIn: parent
            text: "TEST"
            font.pixelSize: 48
            color: "black"
        }
    }
    
    Component.onCompleted: {
        console.log("Window created and shown")
    }
}
