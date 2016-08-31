
#include <QtGui>
#include <QApplication>
#include <QtQuick>
//#include "voltamp_io.h"

int main( int argc, char * argv[] )
{
    QApplication app( argc, argv );

    Q_INIT_RESOURCE( images );

    QFile f( ":main.qml" );
    bool result = f.open( QIODevice::ReadOnly );
    Q_ASSERT( result );
    QByteArray a = f.readAll();

    QQmlEngine engine;
    QQmlComponent component( &engine );
    component.setData( a, QUrl());
    QQuickItem *item = qobject_cast<QQuickItem *>(component.create());

    QQuickView * view = new QQuickView( &engine, 0 );
    view->show();

    int res = app.exec();
    
    return res;
}

