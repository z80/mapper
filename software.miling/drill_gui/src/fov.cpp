
#include "fov.h"
#include <vtkIdList.h>

Fov::Fov()
{
    pts      = vtkPoints::New();
    polyData = vtkPolyData::New();
    polyData->Allocate();
    mapper = vtkPolyDataMapper::New();
    mapper->SetInputData( polyData );
    actor = vtkActor::New();
    actor->SetMapper( mapper );
}

Fov::~Fov()
{

}

void Fov::updateFov( std::vector<double> & xy )
{
    int sz = static_cast<int>( xy.size() );
    sz = sz/2;

    pts->Reset();
    for ( int i=0; i<sz; i++ )
    {
        int ind = i*2;
        double x = xy[ ind ];
        double y = xy[ ind+1 ];
        pts->InsertNextPoint( x, y, 0.0 );
    }

    polyData->Reset();
    for ( int i=0; i<sz; i++ )
    {
        int from = i;
        int to = (i+1)%sz;
        vtkIdType ids[2];
        ids[0] = from;
        ids[1] = to;
        polyData->InsertNextCell( VTK_LINE, 2, ids );
    }
    polyData->SetPoints( pts );
    mapper->SetScalarRange( 0, sz-1 );
}
