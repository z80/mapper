
// First include the required header files for the VTK classes we are using.
#include "vtkConeSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkCamera.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkProperty.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyleMultiTouchCamera.h"

#include <string>
#include <stlsurf.hpp>
#include <stlreader.hpp>
#include <triangle.hpp>
#include <climits>

int main()
{
    vtkRenderer *ren1= vtkRenderer::New();
    ren1->SetBackground( 0.1, 0.2, 0.4 );

    ocl::STLSurf surf;
    ocl::STLReader reader( L"./test.stl", surf );

    int sz = static_cast<int>( surf.tris.size() );
    for ( std::list<ocl::Triangle>::iterator i=surf.tris.begin(); i!=surf.tris.end(); i++ )
    {
        ocl::Triangle & t = *i;
        vtkSmartPointer<vtkPoints>   pts      = vtkPoints::New();
        vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::New();
        polyData->Allocate();
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkPolyDataMapper::New();
        mapper->SetInputData( polyData );
        vtkSmartPointer<vtkActor> actor = vtkActor::New();
        actor->SetMapper( mapper );
        actor->GetProperty()->SetColor( 0.75, 0.0, 0.0 );

        for ( int j=0; j<3; j++ )
            pts->InsertNextPoint( t.p[j].x, t.p[j].y, t.p[j].z );
        vtkIdType ids[2];
        ids[0] = 0;
        ids[1] = 1;
        polyData->InsertNextCell( VTK_LINE, 2, ids );
        ids[0] = 1;
        ids[1] = 2;
        polyData->InsertNextCell( VTK_LINE, 2, ids );
        ids[0] = 2;
        ids[1] = 0;
        polyData->InsertNextCell( VTK_LINE, 2, ids );

        polyData->SetPoints( pts );
        mapper->SetScalarRange( 0, sz-1 );

        ren1->AddActor( actor );
    }


    ocl::STLSurf surf2;
    ocl::STLReader reader2( L"./test.stl", surf2 );
    // Rotate using arbitrary face. For example, the very first one.
    ocl::Triangle t = *surf2.tris.begin();

    double A[3][3];
    ocl::Point n = t.n;
    // XY projection.
    if ( ( abs( n.x ) > std::numeric_limits<double>::epsilon() ) &&
         ( abs( n.y ) > std::numeric_limits<double>::epsilon() ) )
    {
        // Normalize projection and rotate to match it with Ox.
        double l = sqrt( n.x * n.x + n.y*n.y );
        double nx = n.x / l;
        double ny = n.y / l;
        // It is necessary to "unrotate". To do that it is neccessary
        // to invert rotation that caused current position.
        // For rotation inverted is equal to transposed.
        A[0][0] = nx;  A[0][1] = ny;  A[0][2] = 0.0;
        A[0][0] = -ny; A[0][1] = nx;  A[0][2] = 0.0;
        A[0][0] = 0.0; A[0][1] = 0.0; A[0][2] = 1.0;
    }
    else
    {
        // Just unit matrix.
        A[0][0] = 1.0; A[0][1] = 0.0;  A[0][2] = 0.0;
        A[0][0] = 0.0; A[0][1] = 1.0;  A[0][2] = 0.0;
        A[0][0] = 0.0; A[0][1] = 0.0;  A[0][2] = 1.0;

    }
    // And now rotate around Oy on minus cosine between n and (0, 0, -1).
    // Again inverted is equal to transposed.
    double c = -n.z;
    double s = sqrt( 1.0-c*c );
    double B[3][3];
    A[0][0] = c;   A[0][1] = 0.0;  A[0][2] = s;
    A[0][0] = 0.0; A[0][1] = 1.0;  A[0][2] = 0.0;
    A[0][0] = -s;  A[0][1] = 0.0;  A[0][2] = c;
    // And resultant preansformation is B*A.



    sz = static_cast<int>( surf2.tris.size() );
    for ( std::list<ocl::Triangle>::iterator i=surf2.tris.begin(); i!=surf2.tris.end(); i++ )
    {
        ocl::Triangle & t = *i;
        vtkSmartPointer<vtkPoints>   pts      = vtkPoints::New();
        vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::New();
        polyData->Allocate();
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkPolyDataMapper::New();
        mapper->SetInputData( polyData );
        vtkSmartPointer<vtkActor> actor = vtkActor::New();
        actor->SetMapper( mapper );
        actor->GetProperty()->SetColor( 0.0, 0.75, 0.0 );

        for ( int j=0; j<3; j++ )
            pts->InsertNextPoint( t.p[j].x, t.p[j].y, t.p[j].z );
        vtkIdType ids[2];
        ids[0] = 0;
        ids[1] = 1;
        polyData->InsertNextCell( VTK_LINE, 2, ids );
        ids[0] = 1;
        ids[1] = 2;
        polyData->InsertNextCell( VTK_LINE, 2, ids );
        ids[0] = 2;
        ids[1] = 0;
        polyData->InsertNextCell( VTK_LINE, 2, ids );

        polyData->SetPoints( pts );
        mapper->SetScalarRange( 0, sz-1 );

        ren1->AddActor( actor );
    }




    vtkRenderWindow *renWin = vtkRenderWindow::New();
    renWin->AddRenderer( ren1 );
    renWin->SetSize( 300, 300 );

    vtkSmartPointer<vtkCamera> cam = vtkCamera::New();
    cam->SetFocalPoint( 0.0, 0.0, 0.0 );
    cam->SetPosition( 0.0, 50.0, 50.0 );

    ren1->SetActiveCamera( cam );



    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
    iren->SetRenderWindow(renWin);
    vtkInteractorStyleMultiTouchCamera *style =
    vtkInteractorStyleMultiTouchCamera::New();
    iren->SetInteractorStyle(style);

    //Start the event loop
    iren->Initialize();
    iren->Start();

    return 0;
}


