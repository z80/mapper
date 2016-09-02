
#include <QtGui>
#include <QApplication>
#include <QtQuick>
#include "piechart.h"
//#include "voltamp_io.h"

int main( int argc, char * argv[] )
{
    QApplication app( argc, argv );

    Q_INIT_RESOURCE( images );

    qmlRegisterType<PieChart>("Charts", 1, 0, "PieChart");

    QQuickView view;
    view.setResizeMode(QQuickView::SizeRootObjectToView);
    view.setSource(QUrl("qrc:///main.qml"));
    view.show();


    int res = app.exec();
    
    return res;
}

