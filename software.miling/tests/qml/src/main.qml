import QtQuick 2.0
import Charts 1.0

Rectangle {
    width: 360
    height: 240

    ListModel {
        id: dataModel

        ListElement {
            color: "orange"
            texts: [
                ListElement { text: "one" },
                ListElement { text: "two" }
            ]
        }
        ListElement {
            color: "skyblue"
            texts: [
                ListElement { text: "three" },
                ListElement { text: "four" }
            ]
        }
    }

    ListView {
        id: view

        anchors.margins: 10
        anchors.fill: parent
        spacing: 10
        model: dataModel

        delegate: Rectangle {
            width: view.width
            height: 100
            color: model.color

            Row {
                anchors.margins: 10
                anchors.left: parent.left
                anchors.verticalCenter: parent.verticalCenter
                spacing: 10

                Repeater {
                    model: texts
                    delegate: Text {
                        verticalAlignment: Text.AlignVCenter
                        renderType: Text.NativeRendering
                        text: model.text
                    }
                }
            }
        }

        Item {
            width: 300; height: 200

            PieChart {
                id: aPieChart
                anchors.centerIn: parent
                width: 100; height: 100
                name: "A simple pie chart"
                color: "red"
            }

            Text {
                anchors { bottom: parent.bottom; horizontalCenter: parent.horizontalCenter; bottomMargin: 20 }
                text: aPieChart.name
            }
        }
    }


}
