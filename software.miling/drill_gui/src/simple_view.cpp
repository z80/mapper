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
#include "drives_ctrl.h"

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <vtkDataObjectToTable.h>
#include <vtkElevationFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkQtTableView.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkVectorText.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

#include <QFileDialog>


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
  text->SetText("End mill");
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
  renderer->SetBackground( 0.85, 0.95, 0.85 );

  // Add Actor to renderer
  renderer->AddActor(actor);
  renderer->SetActiveCamera( camera.cam );

  // Inserting Camera and FieldOfView.
  renderer->AddActor( fov.actor );
  //camera.
  renderer->AddActor( visibleRects.actor );
  renderer->AddActor( knownRects.actor );
  renderer->AddActor( endMill.actor );
  grid = new Grid( renderer );
  model = new Model( renderer, this->ui->qvtkWidget->GetInteractor() );

  gridCalc = new GridCalc( 0 );

  // VTK/Qt wedded
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);


  vtkSmartPointer<vtkAxesActor> axes = 
    vtkSmartPointer<vtkAxesActor>::New();
 
  //vtkSmartPointer<vtkOrientationMarkerWidget> widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
  vtkOrientationMarkerWidget * widget = vtkOrientationMarkerWidget::New();
  widget->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
  widget->SetOrientationMarker( axes );
  QVTKInteractor * interactor = this->ui->qvtkWidget->GetInteractor();
  widget->SetInteractor( interactor );
  widget->SetViewport( 0.0, 0.0, 0.4, 0.4 );
  widget->SetEnabled( 1 );
  //widget->InteractiveOn();

  // Just a bit of Qt interest: Culling off the
  // point data and handing it to a vtkQtTableView
  //VTK_CREATE(vtkDataObjectToTable, toTable);
  //toTable->SetInputConnection(elevation->GetOutputPort());
  //toTable->SetFieldType(vtkDataObjectToTable::POINT_DATA);

  // Here we take the end of the VTK pipeline and give it to a Qt View
  //this->TableView->SetRepresentationFromInputConnection(toTable->GetOutputPort());

  // Set up action signals and slots
  connect( this->ui->actionOpenModelFile,  SIGNAL(triggered()), this, SLOT(slotOpenFileM()) );
  connect( this->ui->actionOpenSampleFile, SIGNAL(triggered()), this, SLOT(slotOpenFileS()) );
  connect( this->ui->actionExit,           SIGNAL(triggered()), this, SLOT(slotExit()) );

  connect( this->ui->actionDrive_movement, SIGNAL(triggered()), this, SLOT(slotDrivesCtrl()) );
  connect( this->ui->gridCalc, SIGNAL(triggered()), this, SLOT(slotGrid()) );

  connect( this->ui->emCalibrate, SIGNAL(clicked()),         this, SLOT(slotEmCalibrate()) );
  connect( this->ui->emAppend,    SIGNAL(clicked()),         this, SLOT(slotEmAppend()) );
  connect( this->ui->emDiameter,  SIGNAL(editingFinished()), this, SLOT(slotEmChanged()) );

  // Sample and model alingment.
  connect( this->ui->faceOnSample,   SIGNAL(clicked()), this, SLOT(slotFaceOnSample()) );
  connect( this->ui->faceOnModel,    SIGNAL(clicked()), this, SLOT(slotFaceOnModel()) );
  connect( this->ui->edgeOnSample,   SIGNAL(clicked()), this, SLOT(slotEdgeOnSample()) );
  connect( this->ui->edgeOnModel,    SIGNAL(clicked()), this, SLOT(slotEdgeOnModel()) );
  connect( this->ui->dropOnFace,     SIGNAL(clicked()), this, SLOT(slotDropOnFace()) );
  connect( this->ui->sampleAlign,    SIGNAL(clicked()), this, SLOT(slotSampleAlign()) );
  connect( this->ui->addEdgeContact, SIGNAL(clicked()), this, SLOT(slotAddEdgeContact()) );
  connect( this->ui->addEdgeFront,   SIGNAL(clicked()), this, SLOT(slotAddEdgeFront()) );


  inputCapture.open( 0 );
};

SimpleView::~SimpleView()
{
    // The smart pointers should clean up for up
    gridCalc->deleteLater();
    delete grid;
    delete model;
}

void SimpleView::showGrid()
{
    double d = gridCalc->size();
    double step = gridCalc->precission();
    double height = gridCalc->height();
    grid->setCutter( ui->emDiameter->value(), height + 100.0 );
    grid->setPrecision( step );
    grid->setZInterval( height, height-step/2.0 );
    double x, y;
    positioner.drillPos( x, y );
    grid->setPoints( cv::Point2d( x, y ), d );
    grid->setModel( &(model->model) );
    grid->run();
}

// Action to be taken upon file open
void SimpleView::slotOpenFileM()
{
    //QOpenFileDialog::exec
    QString fname = QFileDialog::getOpenFileName( this, "Select STL model file", "", "Model (*.stl)" );
    if ( fname.size() > 0 )
    {
        std::string stri = fname.toStdString();
        model->loadModel( stri );
    }
}

void SimpleView::slotOpenFileS()
{
    QString fname = QFileDialog::getOpenFileName( this, "Select STL sample file", "", "Sample (*.stl)" );
    if ( fname.size() > 0 )
    {
        std::string stri = fname.toStdString();
        model->loadSample( stri );
    }
}

void SimpleView::slotExit() {
  qApp->exit();
}

void SimpleView::slotDrivesCtrl()
{
    DrivesCtrl dc( this );
    dc.exec();
}

void SimpleView::slotGrid()
{
    gridCalc->sv = this;
    gridCalc->show();
}

void SimpleView::slotVideoAlign()
{
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
}

void SimpleView::slotReadFrame()
{
    if ( !ui->actionVideoAlign->isChecked() )
    {
        timer->start();
        return;
    }

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

void SimpleView::slotFaceOnSample()
{
    model->setModeSampleFace();
}

void SimpleView::slotFaceOnModel()
{
    model->setModeModelFace();
}

void SimpleView::slotEdgeOnSample()
{
    model->setModeSampleEdge();
}

void SimpleView::slotEdgeOnModel()
{
    model->setModeModelEdge();
}

void SimpleView::slotDropOnFace()
{
    model->dropOnFace();
}

void SimpleView::slotSampleAlign()
{
    if ( ui->sampleAlign->isChecked() )
        positioner.startSamplePos();
    else
        positioner.endSamplePos( ui->emDiameter->value()/10.0 ); // Divide by 10 to convert into mm.
}

void SimpleView::slotAddEdgeContact()
{
    ocl::Point & pa = model->edgeA;
    ocl::Point & pb = model->edgeB;
    ocl::Point n = model->edgeN;
    ocl::Point a = pb - pa;
    a.z = 0.0;
    n.z = 0.0;
    n = n.cross( a );
    n = a.cross( n );

    cv::Point2d da( pa.x, pa.y );
    cv::Point2d dn( n.x, n.y );
    positioner.appendSamplePos( da, dn );
}

void SimpleView::slotAddEdgeFront()
{
    ocl::Point & pa = model->edgeA;
    ocl::Point & pb = model->edgeB;
    ocl::Point n = model->edgeN;
    ocl::Point a = pb - pa;
    a.z = 0.0;
    n.z = 0.0;
    n = n.cross( a );
    n = a.cross( n );

    cv::Point2d da( pa.x, pa.y );
    cv::Point2d dn( n.x, n.y );
    positioner.appendSampleFrontPos( da, dn );
}





