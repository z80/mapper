#include<stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio/videoio.hpp"

void fitLine( std::vector< std::vector<double> > & line, cv::Point2d & c, cv::Point2d & a );

int main()
{
    std::vector<double> pts;
    pts.push_back( 1.0 );
    pts.push_back( 1.0 );
    pts.push_back( 1.0 );
    pts.push_back( 0.0 );
    pts.push_back( 0.0 );
    pts.push_back( 1.0 );

    pts.push_back( 0.0 );
    pts.push_back( 2.0 );
    pts.push_back( 1.0 );
    pts.push_back( 0.0 );
    pts.push_back( 0.0 );
    pts.push_back( 1.0 );

    pts.push_back( 2.0 );
    pts.push_back( 1.0 );
    pts.push_back( 0.0 );
    pts.push_back( 1.0 );
    pts.push_back( 1.0 );
    pts.push_back( 0.0 );

    // Points number.
    auto sz = pts.size() / 6;
    // Sort points by lines.
    std::vector< std::vector<double> > p;
    for ( auto i=0; i<sz; )
    {
        std::vector<double> v( 6 );
        for ( auto j=0; j<6; j++ )
        {
            v[j] = pts[i];
            i++;
        }
        p.push_back( v );
    }
    std::vector< std::vector< std::vector<double> > > lines;
    while ( p.size() > 0 )
    {
        double rx = p[0][2];
        double ry = p[0][3];
        double nx = p[0][4];
        double ny = p[0][5];
        std::vector< std::vector<double> > to;
        std::remove_copy_if( p.begin(), p.end(), to.begin(), [=](const std::vector<double> & v) { return ( (v[2]==rx) && (v[3]==ry) && (v[4]==nx) && (v[5]==ny) ); } );
        lines.push_back( to );
    }
    std::sort( lines.begin(), lines.end(), [=]( std::vector< std::vector<double> > & a, std::vector< std::vector<double> > & b ) { return (a.size () < b.size()); } );
    // The very first line hast the most points.
    // Approximate it with line using eigenvalues.


   return 0;
}

void fitLine( std::vector< std::vector<double> > & line, cv::Point2d & c, cv::Point2d & a )
{
    auto sz = line.size();
    cv::Mat x( sz, 2, CV_64F );
    for ( auto i=0; i<sz; i++ )
    {
        x.at<> = line[i][0];
        x.at<> = line[i][1];
    }
}
