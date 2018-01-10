import QtQuick 2.0

Item {
    id: block

    property int type: 0

    Image {
        id: img

        anchors.fill: parent
        source: {
            if (type == 0) {
                return "pics/redStone.png"
            } else if (type == 1) {
                return "pics/blueStone.png"
            } else {
                return "pics/greenStone.png"
            }
        }
    }
}

