
#include "camera.h"

Camera::Camera()
{
    cam = vtkCamera::New();
    cam->SetFocalPoint( 0.0, 0.0, 0.0 );
    cam->SetPosition( 100.0, 100.0, 100.0 );
}

Camera::~Camera()
{

}


