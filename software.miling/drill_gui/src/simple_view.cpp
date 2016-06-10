/*
 * Copyright 2007 Sandia Corporation.
 * Under the terms of Contract DE-AC04-94AL85000, there is a non-exclusive
 * license for use of this work by or on behalf of the
 * U.S. Government. Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that this Notice and any
 * statement of authorship are reproduced on all copies.
 */


#include "ui_simple_view.h"
#include "simple_view.h"

#include <vtkDataObjectToTable.h>
#include <vtkElevationFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkQtTableView.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkVectorText.h>
#include <vtkSTLReader.h>

#include "vtkSmartPointer.h"

#define VTK_CREATE(type, name) \
  vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

// Constructor
SimpleView::SimpleView()
{
  this->ui = new Ui_SimpleView;
  this->ui->setupUi(this);

  timer = new QTimer( this );
  timer->setInterval( 10 );
  timer->setSingleShot( true );
  timer->start();
  connect( timer, SIGNAL(timeout()), this, SLOT(slotReadFrame()) );

  enableEndMillCtrls( false );

  // Qt Table View
  //this->TableView = vtkSmartPointer<vtkQtTableView>::New();

  // Place the table view in the designer form
  //this->ui->tableFrame->layout()->addWidget(this->TableView->GetWidget());

  // Geometry
  VTK_CREATE(vtkVectorText, text);
  text->SetText("VTK and Qt!");
  VTK_CREATE(vtkElevationFilter, elevation);
  elevation->SetInputConnection(text->GetOutputPort());
  elevation->SetLowPoint(0,0,0);
  elevation->SetHighPoint(10,0,0);

  // Mapper
  VTK_CREATE(vtkPolyDataMapper, mapper);
  mapper->ImmediateModeRenderingOn();
  mapper->SetInputConnection(elevation->GetOutputPort());

  // Actor in scene
  VTK_CREATE(vtkActor, actor);
  actor->SetMapper(mapper);

  // VTK Renderer
  //VTK_CREATE(vtkRenderer, ren);
  renderer = vtkRenderer::New();

  // Add Actor to renderer
  renderer->AddActor(actor);
  renderer->SetActiveCamera( camera.cam );

  // Inserting Camera and FieldOfView.
  renderer->AddActor( fov.actor );
  //camera.
  renderer->AddActor( visibleRects.actor );
  renderer->AddActor( knownRects.actor );
  renderer->AddActor( endMill.actor );

  // VTK/Qt wedded
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);

  // Just a bit of Qt interest: Culling off the
  // point data and handing it to a vtkQtTableView
  //VTK_CREATE(vtkDataObjectToTable, toTable);
  //toTable->SetInputConnection(elevation->GetOutputPort());
  //toTable->SetFieldType(vtkDataObjectToTable::POINT_DATA);

  // Here we take the end of the VTK pipeline and give it to a Qt View
  //this->TableView->SetRepresentationFromInputConnection(toTable->GetOutputPort());

  // Set up action signals and slots
  connect( this->ui->actionOpenFile, SIGNAL(triggered()), this, SLOT(slotOpenFile()) );
  connect( this->ui->actionExit,     SIGNAL(triggered()), this, SLOT(slotExit()) );



  connect( this->ui->emCalibrate, SIGNAL(clicked()),         this, SLOT(slotEmCalibrate()) );
  connect( this->ui->emAppend,    SIGNAL(clicked()),         this, SLOT(slotEmAppend()) );
  connect( this->ui->emDiameter,  SIGNAL(editingFinished()), this, SLOT(slotEmChanged()) );

  inputCapture.open( 0 );
};

SimpleView::~SimpleView()
{
  // The smart pointers should clean up for up

}

// Action to be taken upon file open
void SimpleView::slotOpenFile()
{

}

void SimpleView::slotExit() {
  qApp->exit();
}


void SimpleView::slotEmCalibrate()
{
    bool en = ui->emCalibrate->isChecked();
    enableEndMillCtrls( en );
    if ( en )
        positioner.startDrillPos();
    else
        positioner.endDrillPos();
}

void SimpleView::slotEmAppend()
{
    positioner.appendDrillPos();
}

void SimpleView::slotEmChanged()
{
    positioner.r = ui->emDiameter->value() / 2.0;
}

void SimpleView::enableEndMillCtrls( bool en )
{
    ui->emAppend->setEnabled( en );
    ui->emDiameter->setEnabled( en );
}

void SimpleView::enableAxesCtrls( bool en )
{
    ui->motoAppend->setEnabled( en );
}

void SimpleView::enableSampleCtrls( bool en )
{
    ui->sampleAppend->setEnabled( en );
    ui->rx->setEnabled( en );
    ui->ry->setEnabled( en );
    ui->nx->setEnabled( en );
    ui->ny->setEnabled( en );
}

void SimpleView::slotReadFrame()
{
    cv::Mat img;
    inputCapture >> img;

    positioner.frame( img );

    std::vector<double> xy;
    positioner.fieldOfView( xy );
    fov.updateFov( xy );
    positioner.visibleFeatures( xy );
    visibleRects.update( xy );
    positioner.knownFeatures( xy );
    knownRects.update( xy );

    double x, y;
    positioner.drillPos( x, y );
    endMill.update( x, y, 0.0 );

    timer->start();

    this->ui->qvtkWidget->GetRenderWindow()->Render();
}

void SimpleView::slotMotoCalibrate()
{
    bool en = ui->motoCalibrate->isChecked();
    enableAxesCtrls( en );

    if ( en )
        positioner.startAxesPos();
    else
        positioner.finishAxesPos();
}

void SimpleView::slotMotoAddPoint()
{
    positioner.appendAxesPos( 0, 0 );
}

void SimpleView::slotSampleCalibrate()
{
    bool en = ui->sampleAlign->isChecked();
    enableSampleCtrls( en );

    if ( en )
        positioner.startSamplePos();
    else
        positioner.endSamplePos();


}

void SimpleView::slotSampleAddPoint()
{
    double rx = ui->rx->value();
    double ry = ui->ry->value();
    double nx = ui->nx->value();
    double ny = ui->ny->value();

    positioner.appendSamplePos( cv::Point2d( rx, ry ), cv::Point2d( nx, ny ) );
}




