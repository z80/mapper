
#ifndef __END_MILL_H_
#define __END_MILL_H_

#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>


class EndMill
{
public:
    EndMill();
    ~EndMill();

    void setDiameter( double d );
    void update( double x, double y, double z );

    vtkSmartPointer<vtkCylinderSource> cylinder;
    vtkSmartPointer<vtkPolyData>       polyData;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    vtkSmartPointer<vtkActor>          actor;
};



#endif


