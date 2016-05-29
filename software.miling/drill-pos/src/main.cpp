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
#include "positioner.h"

using namespace cv;
using namespace std;

void thresh_callback(int, void* );

int thresh = 184;
int max_thresh = 255;
RNG rng(12345);

Mat img, gray;
Mat blurred;
Mat toProcess;
int thresholdValue = 192;
int cutoffTo   = 255;

int blurValue = 10;
int eps       = 5;
int threshholdWindowSz = 3;

const double EDGE_SIZE = 10.0;
cv::Mat      perspectiveCumulative;
int          perspectiveQty;

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
    FileStorage fs( "./data/out_camera_data.xml", FileStorage::READ ); // Read the settings
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


    FileStorage fsP( "./perspective.xml", FileStorage::READ ); // Read the settings
    if (!fsP.isOpened())
    {
          cout << "Could not open the configuration file" << endl;
          return -1;
    }
    cv::Mat perspective;
    fsP[ "perspective" ] >> perspective;
    fsP.release();


    Positioner positioner;
    while ( true )
    {
        inputCapture >> img;
        Mat undistorted;
        undistort( img, undistorted, cameraMatrix, distCoeffs );
        img = undistorted;
        

        positioner.frame( img );

        int res = waitKey( 20 );
        if ( res < 0 )
            continue;
        res &= 0xff;
        if ( res == 'q' )
            break;
    }
    inputCapture.release();

    return 0;
}





