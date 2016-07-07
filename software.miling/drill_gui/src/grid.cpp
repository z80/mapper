
#include "grid.h"
#include "waterline.hpp"
#include "batchdropcutter.hpp"
#include "cylcutter.hpp"
#include "clpoint.hpp"
#include <algorithm>
#include <numeric>
#include <functional>

Grid::Grid( ocl::STLSurf * s )
    : surf( s )
{
    //ocl::Waterline w;

    std::vector<ocl::CLPoint> pts;
    pts.push_back( ocl::CLPoint( 0.0, 0.0, 0.0 ) );
    pts.push_back( ocl::CLPoint( 0.1, 0.0, 0.0 ) );
    pts.push_back( ocl::CLPoint( 0.2, 0.0, 0.0 ) );

    bdc.setSTL( *s );
    bdc.setCutter( &cutter );
    bdc.setSampling( 0.1 );
    std::for_each( pts.begin(), pts.end(),
                   std::bind( &ocl::BatchDropCutter::appendPoint, &bdc, std::placeholders::_1 ) );
    //bdc.setZ( 0.0 );

    bdc.run();
}

Grid::~Grid()
{

}

void Grid::setCutter( double d, double l )
{
    cutter = ocl::CylCutter( d, l );
    bdc.setCutter( &cutter );
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
                bdc.appendPoint( ocl::CLPoint( x, y, z ) );
            }
        }
    }
}

void Grid::run()
{
    bdc.run();
}



