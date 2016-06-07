
#ifndef __FOV_H_
#define __FOV_H_

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>

class Fov
{
public:
    Fov();
    ~Fov();

    void updateFov( std::vector<double> & xy );

    vtkSmartPointer<vtkPoints>         pts;
    vtkSmartPointer<vtkPolyData>       polyData;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    vtkSmartPointer<vtkActor>          actor;
};

#endif

