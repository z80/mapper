
#ifndef __KNOWN_RECTS_H_
#define __KNOWN_RECTS_H_

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>

class KnownRects
{
public:
    KnownRects();
    ~KnownRects();

    void update( std::vector<double> & xy );

    vtkSmartPointer<vtkPoints>         pts;
    vtkSmartPointer<vtkPolyData>       polyData;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    vtkSmartPointer<vtkActor>          actor;
};

#endif

