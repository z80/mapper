import QtQuick 2.0
Canvas {
    id: mycanvas
    width: 100
    height: 200
    onPaint: {
        var ctx = getContext("2d");
        ctx.fillStyle = Qt.rgba(1, 0, 0, 1);
        ctx.fillRect(0, 0, width, height);

        ctx.drawStyle = Qt.rgbs( 0, 0, 0, 0 );
        ctx.arc( width/2, height/2, width/3, 0, 360 );


    }
}
