
#include "point_tracker.h"
#include "points_file.h"

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"


PointTracker::PointTracker()
{
    imageSz = cv::Size( 160, 120 );
}

PointTracker::~PointTracker()
{

}

void PointTracker::setCameraMatrix( const cv::Mat & projMatrix )
{
    this->projMatrix = projMatrix.clone();
}

void PointTracker::process( const cv::Mat & frame, const cv::Mat & worldM )
{
    worldHist.push_back( worldM.clone() );


    prepareImage( frame );
    calcOpticalFlow();
    if ( !flow.empty() )
        countOpticalFlow();
}

bool PointTracker::writePoints( const std::string & fname )
{
    PointsFile pf( fname );
    bool res = pf.write( points3d, camera3d );
    return res;
}

void PointTracker::clear()
{
    points3d.clear();
    camera3d.clear();
}

void PointTracker::prepareImage( const cv::Mat & frame )
{
    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::resize( gray, gray, imageSz );
}

void PointTracker::calcOpticalFlow()
{
    if( !grayPrev.empty() )
    {
        cv::calcOpticalFlowFarneback( grayPrev, gray, uflow, 0.5, 6, 30, 3, 5, 1.2, 0 );
        uflow.copyTo( flow );
    }
    grayPrev = gray.clone();
}

void PointTracker::countOpticalFlow()
{
    pointHistNew.clear();
    for ( int i=0; i<imageSz.height; i++ )
    {
        for ( int j=0; j<imageSz.width; j++ )
        {
            int index = i + imageSz.width * i;
            cv::Point2f dr = flow.at<cv::Point2f>(i, j);

            std::vector<cv::Point2f> & points = pointHistXy( j, i );
            // Determine if there was a movement.
            if ( (fabs(dr.x) < 1.0) && fabs(dr.y) < 1.0 )
            {
                // No movement. Triangulate if there are at least two points in history.
                // Otherwise it obviously doesn't make sense.
                if ( points.size() > 5 )
                {
                    cv::Point3f at, from;
                    calc3dPoint( points, at, from );
                    points3d.push_back( at );
                    camera3d.push_back( from );
                }
                // Clear history.
                points.clear();
                // Push current position.
                cv::Point2f at = cv::Point2f( static_cast<float>( j ), static_cast<float>( i ) );
                points.push_back( at );
                // Put it to a new array
                pushPointHistXy( j, i, points );
            }
            else
            {
                // Movement exists.
                // Push current position plus displacement.
                cv::Point2f at = cv::Point2f( static_cast<float>( j ) + dr.x, static_cast<float>( i ) + dr.y );
                points.push_back( at );
                // Put it to a new array
                int col = cvRound( at.x );
                int row = cvRound( at.y );

                pushPointHistXy( row, col, points );
            }
        }
    }

    // Trim world history.
    int ptSz = longestPointHist();
    int wSz = static_cast<int>( worldHist.size() );
    worldHistNew.clear();
    for ( int i=0; i<ptSz; i++ )
    {
        int ind = wSz - ptSz + i;
        cv::Mat m = worldHist[ ind ].clone();
        worldHistNew.push_back( m );
    }
    worldHist = worldHistNew;
}

void PointTracker::calc3dPoint( std::vector<cv::Point2f> & points,
                                cv::Point3f & at, cv::Point3f & from )
{
    // This is triangulation using all conserved point positions 
    // with all appropriate camera positions.
    // ..... to be implemented.
    double fx = projMatrix.at<double>( 0, 0 );
    double fy = projMatrix.at<double>( 1, 1 );
    double cx = projMatrix.at<double>( 0, 2 );
    double cy = projMatrix.at<double>( 1, 2 );

    int sz = static_cast<int>( points.size() );
    int worldSz = static_cast<int>( worldHist.size() );

    cv::Mat A( 3*sz, 3, CV_64FC1 );
    cv::Mat B( 3*sz, 1, CV_64FC1 );

    for ( int i=0; i<sz; i++ )
    {
        int worldIndex = worldSz - sz + i;
        const cv::Mat wrld = worldHist[ worldIndex ].clone();
        cv::Point2f pt = points[i];

        cv::Mat m( 4, 1, CV_64FC1 );
        double x = (static_cast<double>( pt.x ) - cx) / fx;
        double y = (static_cast<double>( pt.y ) - cy) / fy;
        double z = 1.0;
        double l = sqrt( x*x + y*y + z*z );
        x /= l;
        y /= l;
        z /= l;
        m.at<double>( 0, 0 ) = x;
        m.at<double>( 1, 0 ) = y;
        m.at<double>( 2, 0 ) = z;
        m.at<double>( 3, 0 ) = 0.0; // Yes, 0 to ignore translation part.

        // Camera position.
        double r0[3];
        r0[0] = wrld.at<double>( 0, 3 );
        r0[1] = wrld.at<double>( 1, 3 );
        r0[2] = wrld.at<double>( 2, 3 );

        // Remember the very first camera position to find appropriate
        // surface plane direction afterwards.
        if ( i == 0 )
        {
            from.x = r0[0];
            from.y = r0[1];
            from.z = r0[2];
        }

        // Convert to world ref. frame.
        m = wrld*m;
        double a[3];
        a[0] = m.at<double>( 0, 0 );
        a[1] = m.at<double>( 1, 0 );
        a[2] = m.at<double>( 2, 0 );

        A.at<double>( 3*i, 0 ) = 1.0 - a[0]*a[0];
        A.at<double>( 3*i, 1 ) = -a[0]*a[1];
        A.at<double>( 3*i, 2 ) = -a[0]*a[2];

        A.at<double>( 3*i+1, 0 ) = -a[0]*a[1];
        A.at<double>( 3*i+1, 1 ) = 1.0 - a[1]*a[1];
        A.at<double>( 3*i+1, 2 ) = -a[1]*a[2];

        A.at<double>( 3*i+2, 0 ) = -a[0]*a[2];
        A.at<double>( 3*i+2, 1 ) = -a[1]*a[2];
        A.at<double>( 3*i+2, 2 ) = 1.0 - a[2]*a[2];

        B.at<double>( 3*i, 0 )   = (1.0 - a[0]*a[0])*r0[0] - a[0]*a[1]*r0[1] - a[0]*a[2]*r0[2];
        B.at<double>( 3*i+1, 0 ) = -a[0]*a[1]*r0[0] + (1.0 - a[1]*a[1])*r0[1] - a[1]*a[2]*r0[2];
        B.at<double>( 3*i+2, 0 ) = -a[0]*a[2]*r0[0] - a[1]*a[2]*r0[1] + (1.0 - a[2]*a[2])*r0[2];
    }

    cv::Mat Atr = A.t();
    cv::Mat AtrA = Atr * A;
    cv::Mat invAtrA = AtrA.inv();
    //cv::Mat unity = invAtrA * AtrA;
    cv::Mat AtrB = Atr * B;
    cv::Mat R = invAtrA * AtrB;
    at.x = R.at<double>( 0, 0 );
    at.y = R.at<double>( 1, 0 );
    at.z = R.at<double>( 2, 0 );
}

std::vector<cv::Point2f> & PointTracker::pointHistXy( int row, int col )
{
    int index = imageSz.width * col + row;
    std::map< int, std::vector<cv::Point2f> >::iterator at = pointHist.find( index );
    if ( at != pointHist.end() )
        return at->second;

    //pointHist.insert( std::pair< int, std::vector<cv::Point2f> >( index, std::vector<cv::Point2f>() ) );
    pointHist[ index ] = std::vector<cv::Point2f>();
    std::vector<cv::Point2f> & points = pointHist[ index ];
    return points;
}

void PointTracker::pushPointHistXy( int row, int col, std::vector<cv::Point2f> & points )
{
    int index = imageSz.width * col + row;
    //pointHistNew.insert( std::pair< int, std::vector<cv::Point2f> >( index, points ) );
    pointHistNew[ index ] = points;
}

int PointTracker::longestPointHist() const
{
    int sz = 0;
    std::map< int, std::vector<cv::Point2f> >::const_iterator at;
    for ( at=pointHist.begin(); at!=pointHist.end(); at++ )
    {
        int length = static_cast<int>( at->second.size() );
        sz = ( sz < length ) ? length : sz;
    }
    return sz;
}






