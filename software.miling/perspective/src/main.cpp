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
cv::Mat      A;

void drawCorners( cv::Mat & img, std::vector<cv::Point2f> & pts );
void drawMatrix( cv::Mat & img );
void calcProj( std::vector<cv::Point2f> & pts );

int main(int argc, const char ** argv)
{
    VideoCapture inputCapture;
    inputCapture.open( 1 );
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

    while ( true )
    {
        inputCapture >> img;
        Mat undistorted = img.clone();
        //undistort( undistorted, undistorted, cameraMatrix, distCoeffs );
        img = undistorted.clone();
        
        thresh_callback( 0, 0 );

        int res = 0;
        if ( waitKey( 200 ) == 'q' )
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

        drawCorners( img, corners2d );
        calcProj( corners2d );
        //cv::circle( img, pt, 5, cv::Scalar( 200., 0., 0., 0.2 ) );
        //cv::line( preview, cv::Point( pt.x-sz, pt.y ), cv::Point( pt.x+sz, pt.y ), cv::Scalar( 200., 0., 0., 0.2 ), 2  );


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
}



