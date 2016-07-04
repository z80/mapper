
#ifndef __GRID_CALC_H_
#define __GRID_CALC_H_

#include "ui_grid_calc.h"

class SimpleView;

class GridCalc: public QWidget
{
    Q_OBJECT
public:
    GridCalc( QWidget * parent = 0 );
    ~GridCalc();

    void setSize( qreal size );
    qreal size() const;

    void setPrecission( qreal val );
    qreal precission() const;

    void setHeight( qreal val );
    qreal height() const;

public SLOTS:
    void slotShowGrid();

private:
    Ui_GridCalc ui;
    SimpleView * sv;
};



#endif

