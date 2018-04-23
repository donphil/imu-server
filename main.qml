import QtQuick 2.1
import QtQuick.Window 2.2
import QtQuick.Controls 1.2
import QtQuick.Layouts 1.1
//import server 1.0
Window {
    id : wnd
    signal portSignal(string msg)

    visible: true
    width: 320
    height: 550
    title: "Data Server"

    //------------------------------------------------
    // Perform actions in QML invoked from C++ Signal
    //------------------------------------------------
    Connections{
        target: server;
        onPortChanged: {
            console.log("portResetFromCpp: " + port);
            tI_Port.text = port;
        }
    }
    Connections{
        target: server;
        onDelayChanged: tI_Delay.text = delay
    }
    Connections{
        target: server;
        onMsgSent: {
            //console.log("blink");
            blinkOnSend.running=true;
        }
    }
    //------------------------------------------------
    GridLayout {
        anchors.fill: parent
        anchors.leftMargin: 5
        anchors.rightMargin: 5
        columns: 2
        //rows : 4
        rowSpacing: 5
        columnSpacing: 5
    Item {
                    Layout.columnSpan: 2
                    Layout.fillWidth: true
                    implicitHeight: buttonReset.height

                    Button {
                        id: buttonResetserver
                        width: 100
                        height: 40
                        text:"Reset (Server)"
                        onClicked: {
                            server.resetServer();
                        }
                    }
                    Button {
                        id: buttonReset
                        anchors.left: buttonResetserver.right
                        width: 100
                        height: 40
                        text:"Reset (IMU)"
                        onClicked: {
                            server.resetIMU();
                        }
                    }
                    Button {
                        anchors.left: buttonReset.right
                        Layout.fillWidth: true
                        id: buttonClear
                        width: 100
                        height: 40
                        text:"Clear Text"
                        onClicked: {
                            textMessageOutput.text=""
                            textStatusOutput.text=""
                        }
                    }
        }

    Label {
        text: "Port:"
    }
    TextField{
        Layout.fillWidth: true
        id:tI_Port
        validator: IntValidator{bottom: 1; top: 10000;}
        focus: true
        text : "9001"
        FlashRectangle {
            id : portRectangle
        }
        onAccepted: {
            wnd.portSignal(tI_Port.text);
            portRectangle.running = true;
            console.log(('New port is set to: ' + tI_Port.text));
        }
    }
    Label {
        text: "Delay (ms):"
        Rectangle {
            id : blinkingRectangle
            x: 0
            y: 17
            width:67
            height:7
            color: "black"
            border.width: 0
            SequentialAnimation {
                        running : false
                        id : blinkOnSend
                        PropertyAnimation {
                            target: blinkingRectangle
                            property: "color"
                            to: "green"
                            duration: 100

                        }
                        PropertyAnimation {
                            target: blinkingRectangle
                            property: "color"
                            to: "black"
                            duration: 50
                        }
            }
        }
       }

    TextField{
        Layout.fillWidth: true
        id:tI_Delay
        validator: IntValidator{bottom: 1; top: 10000;}
        focus: false
        text : "1000"
        FlashRectangle {
            id : delayRectangle
        }
        onAccepted: {
            server.processTextMessage('SamplingInterval:'+tI_Delay.text);
            delayRectangle.running = true;
            console.log(('New Delay will be set"' + tI_Delay.text + '"'));
        }
    }
    Label {
        text: "Transm. length:"
    }
    TextField{
        Layout.fillWidth: true
        id:tI_transmissionlength
        validator: IntValidator{bottom: 1; top: 5000;}
        focus: true
        text : "10"
        FlashRectangle {
            id : transmissionlengthRectangle
        }
        onAccepted: {
            server.processTextMessage('ClearBuffer:');
            transmissionlengthRectangle.running = true;
            console.log(('At least ' + tI_transmissionlength.text + " measurements sent"));
        }
    }


    Label {
        text: "Clients:"
    }

    Text {
        id: textClientsOutput
        text: "0"
        Connections {
            target: server
            onNrClientsChanged: {
                textClientsOutput.text = nrClients
            }
        }
    }
    Label {
        text: "Msg:"
    }


    TextArea {
        id : textMessageOutput
        objectName: "name:message"
        text: "server.message"
        Connections {
            target: server
            onMsgChanged: {
                textMessageOutput.append(message)
            }
        }
    }
    Label {
        text: "Status"
    }

    TextArea {
        id: textStatusOutput
        text: "server.status"
        Connections {
            target: server
            onStatusChanged: {
                textStatusOutput.append(status)
            }
        }
    }
    }
}
