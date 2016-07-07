
#include "grid.h"
#include "waterline.hpp"
#include "batchdropcutter.hpp"
#include "cylcutter.hpp"
#include "clpoint.hpp"
#include <algorithm>
#include <numeric>
#include <functional>

Grid::Grid( vtkRenderer * ren )
    : renderer( ren )
{
    pts      = vtkPoints::New();
    polyData = vtkPolyData::New();
    polyData->Allocate();
    mapper = vtkPolyDataMapper::New();
    mapper->SetInputData( polyData );
    actor = vtkActor::New();
    actor->SetMapper( mapper );
    actor->GetProperty()->SetColor( 0.75, 0.0, 0.75 );

    renderer->AddActor( actor );
}

Grid::~Grid()
{

}

void Grid::setModel( ocl::STLSurf * surf )
{
    this->surf = *surf;
    bdc.setSTL( *surf );
}

void Grid::setCutter( double d, double l )
{
    cutter = std::auto_ptr<ocl::CylCutter>( new ocl::CylCutter( d, l ) );
    bdc.setCutter( cutter.get() );
}

void Grid::setPrecision( double prec )
{
    gridStep = prec;
    bdc.setSampling( prec );
}

void Grid::setZInterval( double zFrom, double zTo )
{
    if ( zFrom < zTo )
    {
        double a = zFrom;
        zFrom = zTo;
        zTo = a;
    }
    this->zFrom = zFrom;
    this->zTo   = zTo;
}

void Grid::setPoints( const cv::Point2d & at, double d )
{
    bdc.clearCLPoints();
    // Bla-bla-bla.
    int n = static_cast<int>( d/gridStep );
    int nz = (zTo-zFrom)/static_cast<double>( gridStep );
    double k = d/static_cast<double>( n );
    for ( auto p=0; p<nz; p++ )
    {
        double z = zFrom + p*(zTo-zFrom)/static_cast<double>( nz );
        for ( auto i=0; i<n; i++ )
        {
            double x = at.x + k*static_cast<double>( i );
            for ( auto j=0; j<0; j++ )
            {
                double y = at.y + k*static_cast<double>( j );
                ocl::CLPoint pt( x, y, z );
                bdc.appendPoint( pt );
            }
        }
    }
}

void Grid::run()
{
    bdc.run();
    std::vector<ocl::CLPoint> data = bdc.getCLPoints();




    if ( pts )
        pts->Delete();
    if ( polyData )
        polyData->Delete();
    pts = vtkPoints::New();
    polyData = vtkPolyData::New();
    polyData->Allocate();

    mapper->SetInputData( polyData );


    for ( auto i=data.begin(); i!=data.end(); i++ )
    {
        const ocl::CLPoint & pt = *i;
        double x1 = pt.x;
        double x2 = pt.y;
        double x3 = pt.z;
        pts->InsertNextPoint( x1, x2, x3 );
    }

    for ( int i=1;i<data.size(); i++ )
    {
        vtkIdType ids[2];
        ids[0] = i-1;
        ids[1] = i+1;

        polyData->InsertNextCell( VTK_LINE, 2, ids );
    }
    polyData->SetPoints( pts );
    polyData->BuildCells();
    polyData->BuildLinks();

    renderer->GetRenderWindow()->Render();
}



