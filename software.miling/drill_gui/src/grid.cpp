
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
    ocl::BatchDropCutter bdc;
    ocl::CylCutter cutter( 0.5, 150.0 );

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

}

void Grid::setPrecision( double prec )
{

}

void Grid::setZInterval( double zFrom, double zTo )
{

}

void Grid::setPoints( const cv::Point2d & at, double d )
{

}

void Grid::run()
{

}



