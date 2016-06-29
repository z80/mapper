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

class Line
{
public:
    Line() {}
    ~Line() {}
    Line( const Line & line )
    {
        *this = line;
    }

    const Line & operator=( const Line & line )
    {
        if ( this != &line )
        {
            ni   = line.ni;
            ri   = line.ri;

            pts = line.pts;
            r   = line.r;
            a   = line.a;
        }
        return *this;
    }

    void fitLine();

    cv::Point2d ni, ri; // Declared line parameters; "i" from "ideal" word.

    std::vector< cv::Point2d > pts; // Obtained points close to line.
    cv::Point2d r, a; // Derived line parameteres.
};

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
    sz = lines.size();
    std::vector<Line> linesS;
    for ( auto i=0; i<sz; i++ )
    {
        Line line;
        std::vector< std::vector<double> > & l = lines[i];
        auto lsz = l.size();
        for ( auto j=0; j<lsz; j++ )
        {
            std::vector<double> & pt = l[j];
            line.ri = cv::Point2d( pt[2], pt[3] );
            line.ni = cv::Point2d( pt[4], pt[5] );
            line.pts.push_back( cv::Point2d( pt[0], pt[1] ) );
        }
        line.fitLine();
        linesS.push_back( line );
    }


   return 0;
}

void Line::fitLine()
{
    auto sz = pts.size();
    if ( sz < 2 )
        return;

    cv::Mat x( sz, 2, CV_64F );
    for ( auto i=0; i<sz; i++ )
    {
        x.at<double>(i, 0) = pts[i].x;
        x.at<double>(i, 1) = pts[i].y;
    }
    
    cv::PCA pca_analysis( x, cv::Mat(), CV_PCA_DATA_AS_ROW );

    r = cv::Point2d( pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1) );

    double e0 = pca_analysis.eigenvalues.at<double>(0, 0);
    double e1 = pca_analysis.eigenvalues.at<double>(0, 1);

    int ind = ( fabs( e0 ) > fabs( e1 ) ) ? 0 : 1;
    a = cv::Point2d( pca_analysis.eigenvectors.at<double>(ind, 0), pca_analysis.eigenvectors.at<double>(ind, 1) );
}
