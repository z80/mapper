
#include "end_mill.h"
#include <vtkProperty.h>

EndMill::EndMill()
{
    cylinder = vtkCylinderSource::New();
    cylinder->SetCenter( 0.0, 0.0, 0.0 );
    cylinder->SetRadius( 0.25 );
    cylinder->SetHeight( 5.0 );

    mapper = vtkPolyDataMapper::New();
    mapper->SetInputData( cylinder->GetOutput() );
    actor = vtkActor::New();
    actor->SetMapper( mapper );

    vtkProperty * p = actor->GetProperty();
    p->SetColor( 0.5, 0.6, 0.0 );

}

EndMill::~EndMill()
{

}

void EndMill::setDiameter( double d )
{
    cylinder->SetRadius( d );
}

void EndMill::update( double x, double y, double z )
{
    actor->SetPosition( x, y, z );
}


