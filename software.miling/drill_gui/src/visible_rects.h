
#ifndef __VISIBLE_RECTS_H_
#define __VISIBLE_RECTS_H_

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>

class VisibleRects
{
public:
    VisibleRects();
    ~VisibleRects();

    void update( std::vector<double> & xy );

    vtkSmartPointer<vtkPoints>         pts;
    vtkSmartPointer<vtkPolyData>       polyData;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    vtkSmartPointer<vtkActor>          actor;
};

#endif

