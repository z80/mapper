/*=========================================================================

  Program:   Visualization Toolkit
  Module:    SimpleView.h
  Language:  C++

  Copyright 2009 Sandia Corporation.
  Under the terms of Contract DE-AC04-94AL85000, there is a non-exclusive
  license for use of this work by or on behalf of the
  U.S. Government. Redistribution and use in source and binary forms, with
  or without modification, are permitted provided that this Notice and any
  statement of authorship are reproduced on all copies.

=========================================================================*/
#ifndef SimpleView_H
#define SimpleView_H

#include "vtkSmartPointer.h"    // Required for smart pointer internal ivars.
#include <QMainWindow>
#include <QTimer>
#include "positioner.h"
#include "camera.h"
#include "fov.h"
#include "visible_rects.h"
#include "known_rects.h"
#include "end_mill.h"


// Forward Qt class declarations
class Ui_SimpleView;

// Forward VTK class declarations
class vtkQtTableView;
class vtkSTLReader;
class vtkPoints;
class vtkPolyData;



class SimpleView : public QMainWindow
{
  Q_OBJECT

public:

  // Constructor/Destructor
  SimpleView();
  ~SimpleView();

public slots:

  virtual void slotOpenFile();
  virtual void slotExit();

  void slotEmCalibrate();
  void slotEmAppend();
  void slotEmChanged();
  void slotReadFrame();

protected:

protected slots:

private:
    void enableEndMillCtrls( bool en );

    cv::VideoCapture inputCapture;


    //vtkSmartPointer<vtkQtTableView> TableView;
    vtkSmartPointer<vtkSTLReader>   stlModel;
    vtkSmartPointer<vtkSTLReader>   stlSample;

    // Designer form
    Ui_SimpleView *ui;

    Positioner   positioner;
    Camera       camera;
    KnownRects   knownRects;
    VisibleRects visibleRects;
    EndMill      endMill;
    Fov    fov;


    // Update timer.
    QTimer * timer;

};

#endif // SimpleView_H
