#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"

#include <iostream>
#include <map>


void drawText( cv::Mat & image, const std::string & stri, int line=0 );
void initPoints( cv::Size sz );
void addPoint( cv::Size sz, int row, int col, cv::Point2f displacement );
void framePoints();
void mergeLastTracks();
static void on_mouse( int event, int x, int y, int flags, void* param );
void findAndDisplay( cv::Mat & img, int x, int y );
void findPoint( std::vector<cv::Point2f> & pts );

std::vector<cv::Mat> worldPts;
cv::Mat cameraMatrix;
cv::Mat distCoeffs;

int main()
{

    {
        cv::FileStorage fs( "./data/out_camera_data.xml", cv::FileStorage::READ); // Read the settings
        if (!fs.isOpened())
        {
              std::cout << "Could not open the configuration file" << std::endl;
              return -1;
        }

        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

        fs[ "camera_matrix" ] >> cameraMatrix;
        fs[ "distortion_coefficients" ] >> distCoeffs;
        fs.release();

    }

    cv::FileStorage fs( "./data/ss.xml", cv::FileStorage::READ ); // Read the settings
    if (!fs.isOpened())
    {
          std::cout << "Could not open the configuration file" << std::endl;
          return -1;
    }

    std::cout << "reading images\n";
    int cnt;
    fs[ "frames_cnt" ] >> cnt;

    bool first = true;
    cv::UMat gray,
             grayPrev,
             uflow;
    cv::Mat  flow;
    cv::Mat  flowAccum;

    cv::Mat  imgFirst,
             imgLast;

    for ( int i=0; i<cnt; i++ )
    {
        char frameName[ 256 ];

        std::string fname;
        sprintf( frameName, "img%d", i );
        fs[ frameName ] >> fname;
        std::cout << fname << std::endl;

        cv::Mat cameraPos = cv::Mat::zeros( 4, 4, CV_64F );
        sprintf( frameName, "pos%d", i );
        fs[ frameName ] >> cameraPos;
        //cout << (string)*it << "\n";
        worldPts.push_back( cameraPos );

        cv::Mat image = cv::imread( fname );

        //if ( first )
        cv::Mat undistorted; // = image.clone();
        cv::undistort( image, undistorted, cameraMatrix, distCoeffs );
        cv::flip( undistorted, undistorted, 1 );


        cvtColor( undistorted, gray, cv::COLOR_BGR2GRAY );
        if ( !grayPrev.empty() )
        {
            cv::calcOpticalFlowFarneback( grayPrev, gray, uflow, 0.5, 8, 25, 3, 5, 1.7, 0 );
            uflow.copyTo( flow );
            //cv::add( flowAccum, flow, flowAccum );
            for(int y = 0; y < imgFirst.rows; y++)
            {
                for(int x = 0; x < imgFirst.cols; x++)
                {
                    cv::Point2f displacement = flow.at<cv::Point2f>(y, x);
                    addPoint( cv::Size( imgFirst.cols, imgFirst.rows ),
                              y, x, displacement );
                }
            }
        }
        grayPrev = gray.clone();

        if ( first )
        {
            //cv::calcOpticalFlowFarneback( gray, gray, uflow, 0.5, 3, 15, 3, 5, 1.2, 0 );
            //uflow.copyTo( flow );
            //flowAccum = flow.clone();
            imgFirst = undistorted.clone();
            first = false;

            // Initialize tracks.
            initPoints( cv::Size( imgFirst.cols, imgFirst.rows ) );
        }
        else
        {
            if ( i==(cnt-1) )
                    imgLast = undistorted.clone();
        }

        framePoints();

        cv::imshow( "image", undistorted );
        cv::waitKey( 1 );
    }

    fs.release();


    mergeLastTracks();

/*
    int step = 64;
    cv::Scalar color = cv::Scalar(0, 255, 0);
    for(int y = 0; y < imgFirst.rows; y += step)
    {
        for(int x = 0; x < imgFirst.cols; x += step)
        {
            const cv::Point2f & fxy = flowAccum.at<cv::Point2f>(y, x);
            cv::line(imgFirst, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            cv::circle(imgFirst, cv::Point(x,y), 2, color, -1);
        }
    }
*/
    cv::imshow( "first", imgFirst );
    cv::imshow( "last",  imgLast );
    cv::setMouseCallback( "last", on_mouse, reinterpret_cast<void *>( &imgLast ) );

    cv::waitKey( 0 );
    return 0;
}

void drawText( cv::Mat & image, const std::string & stri, int line )
{
    putText( image, stri,
             cv::Point(20, (50 + line * 50)),
             cv::FONT_HERSHEY_COMPLEX, 1, // font face and scale
             cv::Scalar(255, 255, 255), // white
             1, cv::LINE_AA); // line thickness and type
}


std::map< int, std::vector<cv::Point2f> > frames;
std::map< int, std::vector<cv::Point2f> > framesDone;
std::map< int, std::vector<cv::Point2f> > framesNew;
void initPoints( cv::Size sz )
{
    for ( int col=0; col<sz.width; col++ )
    {
        for ( int row=0; row<sz.height; row++ )
        {
            cv::Point2f pt = cv::Point2f( static_cast<float>(col), static_cast<float>(row) );
            int index = row * sz.width + col;
            std::vector<cv::Point2f> pts;
            pts.push_back( pt );
            framesNew[index] = pts;
        }
    }
}


void addPoint( cv::Size sz, int row, int col, cv::Point2f displacement )
{
    int index = row * sz.width + col;
    double x = static_cast<double>(col) + displacement.x;
    double y = static_cast<double>(row) + displacement.y;
    int newIndex = cvRound( y ) * sz.width + cvRound( x );
    std::map< int, std::vector<cv::Point2f> >::iterator it = frames.find( index );
    cv::Point2f pt = cv::Point2f( static_cast<float>(col) + displacement.x, static_cast<float>(row) + displacement.y );
    if ( it != frames.end() )
    {
        std::vector<cv::Point2f> & pts = frames[index];
        if ( ( fabs( displacement.x ) >= 0.5 ) &&
             ( fabs( displacement.y ) >= 0.5 ) )
        {
            // If there is displacement place to further processing.
            pts.push_back( pt );
            framesNew[ newIndex ] = pts;
        }
        else
        {
            // Else put to completed chains.
            if ( pts.size() > 0 )
            {
                cv::Point2f pt = pts[ 0 ];
                int origIndex = sz.width * cvRound( pt.y ) + cvRound( pt.x );
                framesDone[ origIndex ] = pts;
            }
        }
    }
}

void framePoints()
{
    frames = framesNew;
}

void mergeLastTracks()
{
    std::map< int, std::vector<cv::Point2f> >::const_iterator it;
    for ( it=frames.begin(); it!=frames.end(); it++ )
    {
        framesDone[ it->first ] = it->second;
    }

    // Filter only meaningful tracks. E.i. with length > 1.
    frames.clear();
    for ( it=framesDone.begin(); it!=framesDone.end(); it++ )
    {
        if ( it->second.size() > 4 )
            frames[ it->first ] = it->second;
    }
}

static void on_mouse( int event, int x, int y, int flags, void* param )
{
    cv::Mat & img = *reinterpret_cast<cv::Mat *>( param );
    switch( event )
    {
    case cv::EVENT_LBUTTONDOWN: // set rect or GC_BGD(GC_FGD) labels
        findAndDisplay( img, x, y );
        break;
    default:
        break;
    }
}

void findAndDisplay( cv::Mat & img, int x, int y )
{
    cv::Mat painted = img.clone();
    // Find closest point.
    int index = -1;
    double dist = 23456788.0;

    int rows = painted.rows;
    int cols = painted.cols;
    std::map< int, std::vector<cv::Point2f> >::const_iterator it;
    for ( it=frames.begin();
          it!=frames.end(); it++ )
    {
        int i = it->first;
        int row = i/cols;
        int col = i - row*cols;
        double dx = static_cast<double>( col - x );
        double dy = static_cast<double>( row - y );
        double d = sqrt( dx*dx + dy*dy );
        if ( d < dist )
        {
            index = it->first;
            dist = d;
        }
    }

    // Draw track for this closest point.
    std::vector<cv::Point2f> & pts = framesDone[ index ];
    int sz = static_cast<int>( pts.size() );
    for ( int i=1; i<sz; i++ )
    {
        cv::Point2f from = pts[i-1];
        cv::Point2f to   = pts[i];
        cv::line( painted, from, to, cv::Scalar( 0, 255, 0 ) );
    }
    cv::imshow( "processed", painted );
    std::cout << "sz: " << sz << std::endl;

    findPoint( pts );
}

void findPoint( std::vector<cv::Point2f> & pts )
{
    int sz = static_cast<int>( pts.size() );

    cv::Mat A( 3*sz, 3, CV_64FC1 );
    cv::Mat B( 3*sz, 1, CV_64FC1 );

    double fx = cameraMatrix.at<double>( 0, 0 );
    double fy = cameraMatrix.at<double>( 1, 1 );
    double cx = cameraMatrix.at<double>( 0, 2 );
    double cy = cameraMatrix.at<double>( 1, 2 );

    for ( int i=0; i<sz; i++ )
    {
        cv::Point2f pt = pts[i];

        cv::Mat m( 4, 1, CV_64FC1 );
        double x = (pt.x - cx)/fx;
        double y = (pt.y - cy)/fy;
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
        r0[0] = worldPts[i].at<double>( 0, 3 );
        r0[1] = worldPts[i].at<double>( 1, 3 );
        r0[2] = worldPts[i].at<double>( 2, 3 );

        // Remember the very first camera position to find appropriate
        // surface plane direction afterwards.
        /*if ( i == 0 )
        {
            from.x = r0[0];
            from.y = r0[1];
            from.z = r0[2];
        }*/

        // Convert to world ref. frame.
        //m = wrld*m;
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

    cv::Point3d at;
    at.x = R.at<double>( 0, 0 );
    at.y = R.at<double>( 1, 0 );
    at.z = R.at<double>( 2, 0 );

    std::cout << "x: " << at.x << ", y: " << at.y << ", z: " << at.z << std::endl;
}




