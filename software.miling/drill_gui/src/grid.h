
#ifndef __GRID_H_
#define __GRID_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <climits>

#include <stlsurf.hpp>
#include <stlreader.hpp>
#include <triangle.hpp>

#include "waterline.hpp"
#include "batchdropcutter.hpp"
#include "cylcutter.hpp"
#include "clpoint.hpp"



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

class Grid
{
public:
    Grid( vtkRenderer * ren );
    ~Grid();

    void setModel( ocl::STLSurf * surf );
    void setCutter( double d, double l );
    void setPrecision( double prec );
    void setZInterval( double zFrom, double zTo );
    void setPoints( const cv::Point2d & at, double d );

    void run();

    ocl::STLSurf surf;
    std::auto_ptr<ocl::CylCutter> cutter;
    double gridStep;
    double zFrom, zTo;

    ocl::BatchDropCutter bdc;

    // Needed pointers.
    vtkSmartPointer<vtkRenderer>               renderer;

    // Visualization data.
    vtkSmartPointer<vtkPoints>         pts;
    vtkSmartPointer<vtkPolyData>       polyData;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    vtkSmartPointer<vtkActor>          actor;

};



#endif


