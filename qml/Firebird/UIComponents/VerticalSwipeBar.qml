import QtQuick 2.0

Rectangle {
    property alias text: label.text
    signal clicked

    implicitWidth: label.contentHeight + 2 * 2
    color: "transparent"

    FBLabel {
        id: label
        rotation: -90
        anchors.centerIn: parent
        anchors.rightMargin: 2
        text: qsTr("Swipe here")
    }

    MouseArea {
        property point orig;
        anchors.fill: parent

        onPressed: {
            orig.x = mouseX;
            orig.y = mouseY;
        }

        onReleased: {
            if(Math.abs(orig.x - mouseX) < 5
               && Math.abs(orig.y - mouseY) < 5)
            parent.clicked();
        }
    }
}
