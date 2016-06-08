
#include "visible_rects.h"
#include <vtkProperty.h>

VisibleRects::VisibleRects()
{
    pts      = vtkPoints::New();
    polyData = vtkPolyData::New();
    polyData->Allocate();
    mapper = vtkPolyDataMapper::New();
    mapper->SetInputData( polyData );
    actor = vtkActor::New();
    actor->SetMapper( mapper );

    vtkProperty * p = actor->GetProperty();
    p->SetColor( 0.0, 0.0, 0.6 );
}

VisibleRects::~VisibleRects()
{
}

void VisibleRects::update( std::vector<double> & xy )
{
    int sz = static_cast<int>( xy.size() );
    sz = sz/8;

    pts->Reset();
    int ind = 0;
    for ( int i=0; i<sz; i++ )
    {
        for ( int j=0; j<4; j++ )
        {
            double x = xy[ ind ];
            double y = xy[ ind+1 ];
            pts->InsertNextPoint( x, y, 0.0 );
            ind += 2;
        }
    }

    polyData->Reset();
    ind = 0;
    for ( int i=0; i<sz; i++ )
    {
        for ( int j=0; j<4; j++ )
        {
            int v0 = i;
            int v1 = (i+1)%sz;
            int v2 = (i+2)%sz;
            int v3 = (i+3)%sz;
            vtkIdType ids[5];
            ids[0] = v0;
            ids[1] = v1;
            ids[2] = v2;
            ids[3] = v3;
            ids[4] = v0;
            polyData->InsertNextCell( VTK_LINE, 5, ids );
        }
        ind += 4;
    }
    polyData->SetPoints( pts );
    mapper->SetScalarRange( 0, sz-1 );
}



