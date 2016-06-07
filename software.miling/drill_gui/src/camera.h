
#ifndef __CAMERA_H_
#define __CAMERA_H_

#include <vtkCamera.h>
#include <vtkSmartPointer.h>

class Camera
{
public:
    Camera();
    ~Camera();

    vtkSmartPointer<vtkCamera> cam;
};

#endif


