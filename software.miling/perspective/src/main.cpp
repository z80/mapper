/*
 * matching_test.cpp
 *
 *  Created on: Oct 17, 2010
 *      Author: ethan
 */
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

void thresh_callback(int, void* );

Mat img, gray;

const double EDGE_SIZE = 1.0;
cv::Mat A, accumA, meanA;
int     accumCnt = 0;
bool    displayBoard = true;


void drawCorners( cv::Mat & img, std::vector<cv::Point2f> & pts );
void drawMatrix( cv::Mat & img );
void calcProj( std::vector<cv::Point2f> & pts );
void displayA( cv::Mat & img );

int main(int argc, const char ** argv)
{
    VideoCapture inputCapture;
    inputCapture.open( 0 );
    if ( !inputCapture.isOpened() )
    {
        cout << "Failed to open camera!";
        return 1;
    }

    // Locad calibrated camera parameters.
    FileStorage fs( "./data/out_camera_data.xml", FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
          cout << "Could not open the configuration file" << endl;
          return -1;
    }

    cv::Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs   = Mat::zeros(5, 1, CV_64F);

    fs[ "camera_matrix" ] >> cameraMatrix;
    fs[ "distortion_coefficients" ] >> distCoeffs;
    fs.release();

    accumA = cv::Mat::zeros(8, 1, CV_64F);
    accumCnt = 0;

    while ( true )
    {
        inputCapture >> img;
        Mat undistorted = img.clone();
        //undistort( undistorted, undistorted, cameraMatrix, distCoeffs );
        img = undistorted.clone();
        
        thresh_callback( 0, 0 );

        int res = 0;
        if ( res = waitKey( 200 ) )
        {
            if ( res == 'q' )
            {
                // Saving perspective transform into a file.
                FileStorage fs( "./perspective.xml", FileStorage::WRITE ); // Read the settings
                if (!fs.isOpened())
                {
                      cout << "Could not open the configuration file" << endl;
                      return -1;
                }
                fs << "perspective" << A;
                fs.release();
                // Exit loop.
                break;
            }
            else if ( res == 'd' )
            {
                displayBoard = !displayBoard;
            }
            else if ( res == 'r' )
            {
                accumA = cv::Mat::zeros(8, 1, CV_64F);
                accumCnt = 0;
            }
        }
    }
    inputCapture.release();

    return 0;
}

void thresh_callback(int, void* )
{
    cv::cvtColor( img, gray, CV_BGR2GRAY );
    cv::Size sz = cv::Size( 7, 7 );
    std::vector<cv::Point2f> corners2d;

    bool patternfound = cv::findChessboardCorners(  gray, sz, corners2d,
                                                    cv::CALIB_CB_ADAPTIVE_THRESH +
                                                    cv::CALIB_CB_NORMALIZE_IMAGE +
                                                    cv::CALIB_CB_FAST_CHECK );

    if ( patternfound )
    {
        try {

            cv::cornerSubPix( gray, corners2d, cv::Size(11, 11), cv::Size(-1, -1),
                              cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ) );
        }
        catch ( cv::Exception & e )
        {
            std::cout << e.what() << std::endl;
        }

        calcProj( corners2d );
        drawCorners( img, corners2d );
        displayA( img );
    }
    /// Show in a window
    //namedWindow( "Contours", CV_WINDOW_NORMAL /*CV_WINDOW_AUTOSIZE*/ );
    imshow( "img", img );
}

void drawCorners( cv::Mat & img, std::vector<cv::Point2f> & pts )
{
    bool first = true;
    cv::Point2f ptPrev;
    for ( std::vector<cv::Point2f>::const_iterator i=pts.begin(); i!=pts.end(); i++ )
    {
        const cv::Point2f & pt = *i;
        cv::circle( img, pt, 5, cv::Scalar( 200., 0., 0., 0.2 ) );
        if ( first )
            first = false;
        else
            cv::line( img, ptPrev, pt, cv::Scalar( 0., 0., 100., 0.2 ), 2  );
        ptPrev = pt;
    }
}

void drawMatrix( cv::Mat & img )
{

}

void calcProj( std::vector<cv::Point2f> & pts )
{
    cv::Size sz = cv::Size( 7, 7 );
    cv::Mat X = Mat::eye(2*pts.size(), 8, CV_64F);
    cv::Mat Y = Mat::eye(2*pts.size(), 1, CV_64F);
    for ( int i=0; i<pts.size(); i++ )
    {
        cv::Point2f pt = pts.at( i );
        Y.at<double>( 2*i,     0 ) = pt.x;
        Y.at<double>( 2*i + 1, 0 ) = pt.y;
        double x1, x2;
        int row = i / sz.width;
        int col = i - (sz.width * row);
        x1 = static_cast<double>( col ) - static_cast<double>( sz.width )/2.0;
        x1 *= EDGE_SIZE;
        x2 = static_cast<double>( sz.height-1 + row );
        x2 *= EDGE_SIZE;
        X.at<double>( 2*i, 0 ) = x1;
        X.at<double>( 2*i, 1 ) = x2;
        X.at<double>( 2*i, 2 ) = 1.0;
        X.at<double>( 2*i, 3 ) = 0.0;
        X.at<double>( 2*i, 4 ) = 0.0;
        X.at<double>( 2*i, 5 ) = 0.0;
        X.at<double>( 2*i, 6 ) = -x1*pt.x;
        X.at<double>( 2*i, 7 ) = -x2*pt.x;
        X.at<double>( 2*i+1, 0 ) = 0.0;
        X.at<double>( 2*i+1, 1 ) = 0.0;
        X.at<double>( 2*i+1, 2 ) = 0.0;
        X.at<double>( 2*i+1, 3 ) = x1;
        X.at<double>( 2*i+1, 4 ) = x2;
        X.at<double>( 2*i+1, 5 ) = 1.0;
        X.at<double>( 2*i+1, 6 ) = -x1*pt.y;
        X.at<double>( 2*i+1, 7 ) = -x2*pt.y;
    }
    cv::Mat trX = X.clone();
    trX = trX.t();
    cv::Mat XtX = trX * X;
    XtX = XtX.inv();
    cv::Mat XtY = trX * Y;
    cv::Mat A = XtX * XtY;
    ::A = A.clone();

    accumCnt += 1;
    accumA   += A;
    meanA    = accumA.mul( 1.0/static_cast<double>(accumCnt) );
}

void displayA( cv::Mat & img )
{
    //std::ostringstream os;
    char stri[128];
    const Scalar GREEN(0, 100, 0);
    cv::Mat & A = meanA;
    sprintf( stri, "%5.2f, %5.2f, %5.2f", A.at<double>(0, 0), A.at<double>(1, 0), A.at<double>(2, 0) );
    cv::putText( img, stri, cv::Point( 10, 10 ), 1, 1, GREEN );
    sprintf( stri, "%5.2f, %5.2f, %5.2f", A.at<double>(3, 0), A.at<double>(4, 0), A.at<double>(5, 0) );
    cv::putText( img, stri, cv::Point( 10, 30 ), 1, 1, GREEN );
    sprintf( stri, "%5.2f, %5.2f, 1.000", A.at<double>(6, 0), A.at<double>(7, 0) );
    cv::putText( img, stri, cv::Point( 10, 50 ), 1, 1, GREEN );
    sprintf( stri, "cnt = %5i", accumCnt );
    cv::putText( img, stri, cv::Point( 10, 70 ), 1, 1, GREEN );
}



