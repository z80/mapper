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
            n   = line.n;
        }
        return *this;
    }

    bool fitLine();
    bool fitLine( Line & line );
    bool adjustNormals( Line & line );
    bool intersectionPoint( Line & line, double d, cv::Point2d & ri, cv::Point2d & r0 );

    cv::Point2d ni, ri; // Declared line parameters; "i" from "ideal" word.

    std::vector< cv::Point2d > pts; // Obtained points close to line.
    cv::Point2d r, a, n; // Derived line parameteres.
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

bool Line::fitLine()
{
    auto sz = pts.size();
    if ( sz < 2 )
        return false;

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

    n = cv::Point2d( -a.y, a.x );

    return true;
}

bool Line::fitLine( Line & line )
{
    auto sz = pts.size();

    // Determine mean value.
    double rx = 0.0,
           ry = 0.0;
    for ( auto i=0; i<sz; i++ )
    {
        rx += pts[i].x;
        ry += pts[i].y;
    }
    rx /= static_cast<double>( sz );
    ry /= static_cast<double>( sz );

    // Cross with line and orient a away from intersection point.
    //cv::Point ai = cv::Point2d( -line.ni.y, line.ni.x );
    double ns = line.ni.x * ni.y - line.ni.y * ni.x;
    double nc = ni.dot( line.ni );

    double ax = line.a.x * nc - line.a.y * ns;
    double ay = line.a.x * ns + line.a.y * nc;

    a = cv::Point2d( ax, ay );
    r = cv::Point2d( rx, ry );
    n = cv::Point2d( -a.y, a.x );

    return true;
}

bool Line::adjustNormals( Line & line )
{
    // Determine cosine between normals. If it is too big, lines are almost parallel.
    double dot = ni.dot( line.ni );
    const double MAX_DOT = 0.8;
    if ( fabs( dot ) >= MAX_DOT )
        return false;

    // Define original dot product.
    bool res = fitLine();
    if ( !res )
        return false;
    res = line.fitLine();
    if ( !res )
        line.fitLine( *this );
    

    // Determine intersection point.
    cv::Mat N( 2, 2, CV_64F );
    N.at<double>( 0, 0 ) = n.x;
    N.at<double>( 0, 1 ) = n.y;
    N.at<double>( 1, 0 ) = line.n.x;
    N.at<double>( 1, 1 ) = line.n.y;

    cv::Mat RN( 2, 1, CV_64F );
    RN.at<double>( 0, 0 ) = n.dot( r );
    RN.at<double>( 0, 0 ) = line.n.dot( line.r );
    cv::Mat R = N.inv() * RN;

    cv::Point2d r0( R.at<double>( 0, 0 ), R.at<double>( 1, 0 ) ); // Common point.

    // Determine correct "a" vector directions for current line positions.
    // they are to be in the direction of mean points.
    cv::Point2d aa = r - r0;
    cv::Point2d ab = line.r - r0;

    // Determine "a" directions for original positions.
    cv::Point aai( -ni.y, ni.x );
    cv::Point abi( -line.ni.y, line.ni.x );
    // Dot profuct of "a" with another line "n" should be positive.
    if ( aai.dot( line.ni ) < 0.0 )
        aai = -aai;
    if ( abi.dot( ni ) < 0.0 )
        abi = -abi;

    // By comparing cross products adjust normals for current reference frame.
    double crossi = aai.x*ni.y - aai.y*ni.x;
    double cross  = aa.x*n.y   - aa.y*n.x;
    if ( crossi*cross < 0.0 )
        n = -n;

    crossi = abi.x*line.ni.y - abi.y*line.ni.x;
    cross  = ab.x*line.n.y   - ab.y*line.n.x;
    if ( crossi*cross < 0.0 )
        line.n = -line.n;
    return true;
}

bool Line::intersectionPoint( Line & line, double d, cv::Point2d & ri, cv::Point2d & r0 )
{
    bool res = adjustNormals( line );
    if ( !res )
        return false;

    // Determine intersection point.
    cv::Mat N( 2, 2, CV_64F );
    N.at<double>( 0, 0 ) = ni.x;
    N.at<double>( 0, 1 ) = ni.y;
    N.at<double>( 1, 0 ) = line.ni.x;
    N.at<double>( 1, 1 ) = line.ni.y;

    cv::Mat RN( 2, 1, CV_64F );
    RN.at<double>( 0, 0 ) = ni.dot( ri );
    RN.at<double>( 0, 0 ) = line.ni.dot( line.ri );
    cv::Mat R = N.inv() * RN;

    ri =  cv::Point2d( R.at<double>( 0, 0 ), R.at<double>( 1, 0 ) ); // Common point.

    // Determine the same point in corrent reference frame.
    N.at<double>( 0, 0 ) = n.x;
    N.at<double>( 0, 1 ) = n.y;
    N.at<double>( 1, 0 ) = line.n.x;
    N.at<double>( 1, 1 ) = line.n.y;

    RN.at<double>( 0, 0 ) = n.dot( r - n*(d/2.0) );
    RN.at<double>( 0, 0 ) = line.n.dot( line.r - line.n*(d/2.0) );
    R = N.inv() * RN;

    r0 = cv::Point2d( R.at<double>( 0, 0 ), R.at<double>( 1, 0 ) );
    return true;
}





