#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"
#include <iostream>

#include "point_tracker.h"

void drawText( cv::Mat & image, const std::string & stri, int line=0 );

int main()
{
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

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

    PointTracker  pointTracker;
    pointTracker.setCameraMatrix( cameraMatrix );

    std::cout << "reading images\n";
    int cnt;
    fs[ "frames_cnt" ] >> cnt;
    //bool first = true;

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

        cv::Mat image = cv::imread( fname );

        //if ( first )
        cv::Mat undistorted; // = image.clone();
        cv::undistort( image, undistorted, cameraMatrix, distCoeffs );

        pointTracker.process( undistorted, cameraPos );

        cv::imshow( "image", undistorted );
        cv::waitKey( 1 );
    }

    pointTracker.finish();
    pointTracker.writePoints( "./data/points3d.txt" );

    fs.release();

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







