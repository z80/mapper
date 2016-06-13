
#ifndef __DRIVES_CTRL_H_
#define __DRIVES_CTRL_H_

#include "ui_drives_ctrl.h"
#include <QWidget>
#include <QDialog>

class DrivesCtrl: public QDialog
{
public:
    DrivesCtrl( QWidget * p = 0 );
    ~DrivesCtrl();


    Ui_DrivesCtrl ui;
};


#endif

