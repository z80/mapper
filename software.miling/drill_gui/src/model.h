
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



class Model
{
public:
    Model();
    ~Model();

    void loadModel( const std::string & fname );
    void loadSample( const std::string & fname );

    void setModeSampleFace();
    void setModeSampleEdge();
    void setModeModelFace();
    void setModeModelEdge();

    void dropOnFace();
    void alignToEdge();

    ocl::STLSurf modelOrig,  modelFlipped,  model;
    ocl::STLSurf sampleOrig, sampleFlipped, sample;
    // Flipping matrix.
    double A[3][4];

    // Orienting and positioning matrix.
    double B[3][4];

    // Visualization data.
    vtkSmartPointer<vtkPoints>         ptsM;
    vtkSmartPointer<vtkPolyData>       polyDataM;
    vtkSmartPointer<vtkPolyDataMapper> mapperM;
    vtkSmartPointer<vtkActor>          actorM;

    vtkSmartPointer<vtkPoints>         ptsS;
    vtkSmartPointer<vtkPolyData>       polyDataS;
    vtkSmartPointer<vtkPolyDataMapper> mapperS;
    vtkSmartPointer<vtkActor>          actorS;
};




#endif


