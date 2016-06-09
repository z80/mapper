
#include "camera.h"

Camera::Camera()
{
    cam = vtkCamera::New();
    cam->SetFocalPoint( 0.0, 0.0, 0.0 );
    cam->SetPosition( 0.0, 50.0, 50.0 );
}

Camera::~Camera()
{

}





