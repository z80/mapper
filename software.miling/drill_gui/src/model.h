
#ifndef __MODEL_H_
#define __MODEL_H_

#include "vtkConeSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkCamera.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkProperty.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyleMultiTouchCamera.h"

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkRendererCollection.h>
#include <vtkDataSetMapper.h>
#include <vtkUnstructuredGrid.h>
#include <vtkIdTypeArray.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPlaneSource.h>
#include <vtkCellPicker.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h>
#include <vtkSelectionNode.h>
#include <vtkSelection.h>
#include <vtkExtractSelection.h>
#include <vtkObjectFactory.h>

#include <stlsurf.hpp>
#include <stlreader.hpp>
#include <triangle.hpp>

#include <string>
#include <climits>


class FaceSelectorStyle;
class EdgeSelectorStyle;


class Model
{
public:
    enum TSelectionMode { IDLE_MODE, FACE_SAMPLE, FACE_MODEL, EDGE_SAMPLE, EDGE_MODEL };

    Model( vtkRenderer * ren , vtkRenderWindowInteractor * iren );
    ~Model();

    void loadModel( const std::string & fname );
    void loadSample( const std::string & fname );

    void setModeSampleFace();
    void setModeSampleEdge();
    void setModeModelFace();
    void setModeModelEdge();

    void dropOnFace( const ocl::Triangle & t );
    void alignToEdge();

    // Helper functions for initializing visualization data.
    void resetMatrices();
    void convertPoint( double x1, double x2, double x3, double & y1, double & y2, double & y3 );
    void convertPoint( double * x, double * y );
    void prepareFaces( ocl::STLSurf & surf, vtkSmartPointer<vtkPoints> & pts, vtkSmartPointer<vtkPolyData> & polyData, vtkSmartPointer<vtkPolyDataMapper> & mapper );
    void prepareEdges( ocl::STLSurf & surf, vtkSmartPointer<vtkPoints> & pts, vtkSmartPointer<vtkPolyData> & polyData, vtkSmartPointer<vtkPolyDataMapper> & mapper );
    void convertSurf( const ocl::STLSurf & from, ocl::STLSurf & to );
    // Callbacks.
    void faceSelectedCallback( vtkIdType * inds );
    void edgeSelectedCallback( vtkIdType * inds );

    ocl::STLSurf modelOrig,  model;
    ocl::STLSurf sampleOrig, sample;
    // Flipping matrix.
    double A[3][4];

    // Orienting and positioning matrix.
    double B[3][4];

    // Selected face.
    ocl::Triangle selectedFace;
    // Selected edge.
    ocl::Point edgeA, edgeB, edgeN;

    // Needed pointers.
    vtkSmartPointer<vtkRenderer>               renderer;
    vtkSmartPointer<vtkRenderWindowInteractor> iren;

    // Visualization data.
    vtkSmartPointer<vtkPoints>         ptsM;
    vtkSmartPointer<vtkPolyData>       polyDataM;
    vtkSmartPointer<vtkPolyDataMapper> mapperM;
    vtkSmartPointer<vtkActor>          actorM;

    vtkSmartPointer<vtkPoints>         ptsS;
    vtkSmartPointer<vtkPolyData>       polyDataS;
    vtkSmartPointer<vtkPolyDataMapper> mapperS;
    vtkSmartPointer<vtkActor>          actorS;

    // Selected element visualization.
    vtkSmartPointer<vtkPoints>         ptsSel;
    vtkSmartPointer<vtkPolyData>       polyDataSel;
    vtkSmartPointer<vtkPolyDataMapper> mapperSel;
    vtkSmartPointer<vtkActor>          actorSel;

    // Selectors.
    vtkSmartPointer<FaceSelectorStyle> faceSelector;
    vtkSmartPointer<EdgeSelectorStyle> edgeSelector;

    TSelectionMode selectionMode;
};




#endif


