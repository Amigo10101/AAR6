import QtQuick
import QtQuick.Window

Window {
    width: 640
    height: 480
    visible: true
    title: "Test Window"
    color: "red"

    Component.onCompleted: {
        console.log("Test Window Created")
    }
}
