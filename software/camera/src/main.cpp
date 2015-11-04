#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include "qr_extractor.h"
#include "stats.h"
#include "utils.h"

#include "cam_locator.h"
#include "feature_locator.h"
#include "point_tracker.h"

using namespace cv;
using namespace std;


const double akaze_thresh = 3e-4; // AKAZE detection threshold set to locate about 1000 keypoints
const double ransac_thresh = 2.5f; // RANSAC inlier threshold
const double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
const int bb_min_inliers = 100; // Minimal number of inliers to draw bounding box
const int stats_update_period = 10; // On-screen statistics are updated every 10 frames
















void drawText( Mat & image, const std::string & stri, int line=0 );

Mat cameraMatrix;
Mat distCoeffs;
Mat tMatrix, rMatrix;

enum Mode { LOOK_FOR_QR, TRACK_POINTS, PREDICT };

int main()
{
    CamLocator camLocator( 5, 7, 0.02 );


    cout << "Built with OpenCV " << CV_VERSION << endl;
    Mat image;
    VideoCapture capture;
    capture.open(0);

    FileStorage fs( "./data/out_camera_data.xml", FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
          cout << "Could not open the configuration file" << endl;
          return -1;
    }

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    distCoeffs = Mat::zeros(5, 1, CV_64F);

    fs[ "camera_matrix" ] >> cameraMatrix;
    fs[ "distortion_coefficients" ] >> distCoeffs;
    fs.release();

    camLocator.setCamera( cameraMatrix, distCoeffs );
    
    FeatureLocator featureLocator;
    featureLocator.setCameraMatrix( cameraMatrix, distCoeffs ); 
    PointTracker  pointTracker;
    pointTracker.setCameraMatrix( cameraMatrix, distCoeffs );

    if(capture.isOpened())
    {
        cout << "Capture is opened" << endl;
        int triangulatedCnt = 0;
        bool res;

        cv::Mat camToWorld4x4;
        for(;;)
        {
            capture >> image;
            if(image.empty())
                break;
            Mat undistorted = image.clone();
            undistort( image, undistorted, cameraMatrix, distCoeffs );

            uchar key = (uchar)waitKey( 33 );
            if( key == 'q' )
                break;
            if( key == 's' )
            {
                pointTracker.writePoints();
                pointTracker.clear();
            }

            if ( triangulatedCnt < 5 )
            {
                res = camLocator.findChessboard( undistorted, camToWorld4x4 );
                if ( !res )
                    continue;
            }
            else
                camToWorld4x4 = cv::Mat();

            if ( ( !camToWorld4x4.empty() ) || ( triangulatedCnt >= 5 ) )
                res = featureLocator.processFrame( undistorted, camToWorld4x4 );
            triangulatedCnt = featureLocator.triangulatedCnt();
            //if ( !camToWorld4x4.empty() )
            //    pointTracker.process( undistorted, camToWorld4x4 );
        }
    }
    return 0;
}

void drawText( Mat & image, const std::string & stri, int line )
{
    putText( image, stri,
             Point(20, (50 + line * 50)),
             FONT_HERSHEY_COMPLEX, 1, // font face and scale
             Scalar(255, 255, 255), // white
             1, LINE_AA); // line thickness and type
}






