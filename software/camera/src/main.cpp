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

using namespace cv;
using namespace std;


const double akaze_thresh = 3e-4; // AKAZE detection threshold set to locate about 1000 keypoints
const double ransac_thresh = 2.5f; // RANSAC inlier threshold
const double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
const int bb_min_inliers = 100; // Minimal number of inliers to draw bounding box
const int stats_update_period = 10; // On-screen statistics are updated every 10 frames

class Tracker
{
public:
    Tracker(Ptr<Feature2D> _detector, Ptr<DescriptorMatcher> _matcher) :
        detector(_detector),
        matcher(_matcher)
    {}

    void setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats);
    Mat process(const Mat frame, Stats& stats);
    Ptr<Feature2D> getDetector() {
        return detector;
    }
protected:
    Ptr<Feature2D> detector;
    Ptr<DescriptorMatcher> matcher;
    Mat first_frame, first_desc;
    vector<KeyPoint> first_kp;
    vector<Point2f> object_bb;
};

void Tracker::setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats)
{
    first_frame = frame.clone();
    detector->detectAndCompute(first_frame, noArray(), first_kp, first_desc);
    stats.keypoints = (int)first_kp.size();
    drawBoundingBox(first_frame, bb);
    putText(first_frame, title, Point(0, 60), FONT_HERSHEY_PLAIN, 5, Scalar::all(0), 4);
    object_bb = bb;
}

Mat Tracker::process(const Mat frame, Stats& stats)
{
    vector<KeyPoint> kp;
    Mat desc;
    detector->detectAndCompute(frame, noArray(), kp, desc);
    stats.keypoints = (int)kp.size();

    vector< vector<DMatch> > matches;
    vector<KeyPoint> matched1, matched2;
    matcher->knnMatch(first_desc, desc, matches, 2);
    for(unsigned i = 0; i < matches.size(); i++) {
        if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
            matched1.push_back(first_kp[matches[i][0].queryIdx]);
            matched2.push_back(      kp[matches[i][0].trainIdx]);
        }
    }
    stats.matches = (int)matched1.size();

    Mat inlier_mask, homography;
    vector<KeyPoint> inliers1, inliers2;
    vector<DMatch> inlier_matches;

    /*
    if(matched1.size() >= 4) {
        homography = findHomography(Points(matched1), Points(matched2),
                                    RANSAC, ransac_thresh, inlier_mask);
    }
    */

    if(matched1.size() < 4 || homography.empty()) {
        Mat res;
        hconcat(first_frame, frame, res);
        stats.inliers = 0;
        stats.ratio = 0;
        return res;
    }
    for(unsigned i = 0; i < matched1.size(); i++) {
        if(inlier_mask.at<uchar>(i)) {
            int new_i = static_cast<int>(inliers1.size());
            inliers1.push_back(matched1[i]);
            inliers2.push_back(matched2[i]);
            inlier_matches.push_back(DMatch(new_i, new_i, 0));
        }
    }
    stats.inliers = (int)inliers1.size();
    stats.ratio = stats.inliers * 1.0 / stats.matches;

    vector<Point2f> new_bb;

    perspectiveTransform(object_bb, new_bb, homography);
    Mat frame_with_bb = frame.clone();
    if(stats.inliers >= bb_min_inliers) {
        drawBoundingBox(frame_with_bb, new_bb);
    }
    Mat res;
    drawMatches(first_frame, inliers1, frame_with_bb, inliers2,
                inlier_matches, res,
                Scalar(255, 0, 0), Scalar(255, 0, 0));
    return res;
}
















void drawText( Mat & image, const std::string & stri, int line=0 );
void drawFeatures( Mat & img, const vector<KeyPoint> & pts );
bool solveCamPos();

QrExtractor qr;
vector< Point > qrPts2d;
vector< Point3d > qrPts3d;
bool qrFound;

Mat cameraMatrix;
Mat distCoeffs;
Mat tMatrix, rMatrix;

enum Mode { LOOK_FOR_QR, TRACK_POINTS, PREDICT };

int main()
{
    qrPts3d.resize( 3 );
    qrPts3d[0] = Point3d( 0.0, 0.0, 0.0 );
    qrPts3d[1] = Point3d( 1.0, 0.0, 0.0 );
    qrPts3d[2] = Point3d( 0.0, 1.0, 0.0 );

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
    
    vector<KeyPoint> kpts1, kpts2;
    Mat desc1, desc2;
    Mode mode = LOOK_FOR_QR;
    qrFound = false;

    Stats stats;
    Ptr<ORB> orb_detector = ORB::create();
    orb_detector->setMaxFeatures(stats.keypoints);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    Tracker tracker( orb_detector, matcher );


    if(capture.isOpened())
    {
        cout << "Capture is opened" << endl;
        for(;;)
        {
            capture >> image;
            if(image.empty())
                break;
            Mat undistorted = image.clone();
            undistort( image, undistorted, cameraMatrix, distCoeffs );
            //drawText(image);
            imshow( "Original", image );
            imshow( "Undistorted", undistorted );

            orb_detector->detect( undistorted, kpts1 );
            orb_detector->compute( undistorted, kpts1, desc1 );

            switch ( mode )
            {
            case LOOK_FOR_QR:
            case TRACK_POINTS:
                qrFound = qr.extract( undistorted );
                if ( qrFound )
                    qrPts2d = qr.points();
                drawText( image, qrFound ? "no QR A" : "A Detected!" );
                break;
            };

            ostringstream os;
            os << "mode = ";
            if ( mode == LOOK_FOR_QR )
                os << "LOOK_FOR_QR";
            else if ( mode == TRACK_POINTS )
                os << "TRACK_POINTS";
            else if ( mode == PREDICT )
                os << "PREDICT";
            drawText( image, os.str(), 1 );

            Mat imgWithFeatures = undistorted.clone();
            drawFeatures( imgWithFeatures, kpts1 );
            imshow( "Features", imgWithFeatures );
            //latch->compute( undistorted, kpts1, desc1 );

            int key = waitKey( 10 );
            if( key == 'q' )
                break;
            else if (key == 's' )
            {
                static int ind = 0;
                // Make screenshot.
                ostringstream os;
                os << "image" << ind++ << ".jpg";
                imwrite( os.str(), image );
            }
            else if ( key == 'l' )
            {
                if ( ( mode == LOOK_FOR_QR ) || ( mode == TRACK_POINTS ) )
                {

                    // If qr is found compute camera position.
                    bool res = solveCamPos();
                    if ( res )
                    {
                        mode = TRACK_POINTS;
                        if ( mode == LOOK_FOR_QR )
                            tracker.setFirstFrame( undistorted, kpts1, "aaa", stats );
                        else
                            tracker.process( undistorted, stats );
                    }
                }
            }
            else if ( key == 'p' )
            {
                // If both QRs are found start predicting.

            }
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

void drawFeatures( Mat & img, const vector<KeyPoint> & pts )
{
    const int sz = 9;
    for ( vector<KeyPoint>::const_iterator i=pts.begin(); i!=pts.end(); i++ )
    {
        const KeyPoint & pt = *i;
        line( img, Point( pt.pt.x-sz, pt.pt.y ), Point( pt.pt.x+sz, pt.pt.y ), Scalar( 200., 0., 0., 0.2 ), 2  );
        line( img, Point( pt.pt.x, pt.pt.y-sz ), Point( pt.pt.x, pt.pt.y+sz ), Scalar( 0., 200., 0., 0.2 ), 2  );
    }
}

bool solveCamPos()
{
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    //tvec[ 2 ] = 10.0;

    bool useExtrinsicGuess = true;

    // Pose estimation
    bool correspondence = cv::solvePnP( qrPts3d, qrPts2d, cameraMatrix, distCoeffs,
                                        rvec, tvec,
                                        useExtrinsicGuess, CV_ITERATIVE );

    // Transforms Rotation Vector to Matrix
    Rodrigues( rvec, rMatrix );
    tMatrix = tvec;

    return correspondence;
}






