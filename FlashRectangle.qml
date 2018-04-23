import QtQuick 2.1


Rectangle {
    property string flashcolor : "green"
    property alias running : anim.running
    id : rect_
    color: "#00000000"
    border.width: 0
    border.color: "#000000"
    radius: 1
    anchors.fill: parent
    anchors.margins: -5
    SequentialAnimation {
                running : false
                id : anim
                PropertyAnimation {
                    target: rect_
                    property: "color"
                    to: flashcolor
                    duration: 200

                }
                PropertyAnimation {
                    target: rect_
                    property: "color"
                    to: "transparent"
                    duration: 10
                }
    }
}
