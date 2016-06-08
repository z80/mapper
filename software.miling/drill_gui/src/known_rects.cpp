
#include "known_rects.h"
#include <vtkProperty.h>

KnownRects::KnownRects()
{
    pts      = vtkPoints::New();
    polyData = vtkPolyData::New();
    polyData->Allocate();
    mapper = vtkPolyDataMapper::New();
    mapper->SetInputData( polyData );
    actor = vtkActor::New();
    actor->SetMapper( mapper );

    vtkProperty * p = actor->GetProperty();
    p->SetColor( 0.0, 0.6, 0.0 );
}

KnownRects::~KnownRects()
{
}

void KnownRects::update( std::vector<double> & xy )
{
    int sz = static_cast<int>( xy.size() );
    sz = sz/8;
    //sz = ( sz < 2 ) ? sz : 1;

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
            int v0 = ind;
            int v1 = ind+1;
            int v2 = ind+2;
            int v3 = ind+3;
            vtkIdType ids[5];
            ids[0] = v0;
            ids[1] = v1;
            ids[2] = v2;
            ids[3] = v3;
            ids[4] = v0;
            polyData->InsertNextCell( VTK_LINE, 2, &(ids[0]) );
            polyData->InsertNextCell( VTK_LINE, 2, &(ids[1]) );
            polyData->InsertNextCell( VTK_LINE, 2, &(ids[2]) );
            polyData->InsertNextCell( VTK_LINE, 2, &(ids[3]) );
        }
        ind += 4;
    }
    polyData->SetPoints( pts );
    mapper->SetScalarRange( 0, sz-1 );
}



