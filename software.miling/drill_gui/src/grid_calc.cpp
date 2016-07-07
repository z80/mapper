
#include "grid_calc.h"
#include "simple_view.h"

GridCalc::GridCalc( QWidget * parent )
    :QWidget( parent )
{
    ui.setupUi( this );
    this->setWindowFlags( Qt::Tool );

    sv = 0;

    connect( ui.showGrid, SIGNAL(clicked()), this, SLOT(slotShowGrid()) );
}

GridCalc::~GridCalc()
{

}

void GridCalc::setSize( qreal size )
{
    ui.size->setValue( size );
}

qreal GridCalc::size() const
{
    return ui.size->value();
}

void GridCalc::setPrecission( qreal val )
{
    ui.precision->setValue( val );
}

qreal GridCalc::precission() const
{
    return ui.precision->value();
}

void GridCalc::setHeight( qreal val )
{
    ui.height->setValue( val );
}

qreal GridCalc::height() const
{
    return ui.height->value();
}

void GridCalc::slotShowGrid()
{
    if ( sv )
        sv->showGrid();
}



