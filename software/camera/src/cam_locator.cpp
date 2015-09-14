
#include "cam_locator.h"
#include "opencv2/calib3d.hpp"

class CamLocator::PD
{
public:
    PD() {}
    ~PD() {}

    int rows, cols;
    double step;
    std::vector<cv::Point2f> corners;
    cv::Mat gray;
    bool debug;
};


CamLocator::CamLocator( int rows, int cols, double step )
{
    pd = new PD();
    pd->rows = rows;
    pd->cols = cols;
    pd->step = step;
    pd->debug = true;
}

CamLocator::~CamLocator()
{
    delete pd;
}

bool CamLocator::findChessboard( const cv::Mat & mat, cv::Mat & vRot, cv::Mat & vTrans )
{

    cv::cvtColor( mat, pd->gray, CV_BGR2GRAY );
    cv::Size sz = cv::Size( pd->cols, pd->rows );


    std::vector<cv::Point2f> corners; //this will be filled by the detected corners
    bool patternfound = cv::findChessboardCorners(  pd->gray, sz, pd->corners,
                                                    cv::CALIB_CB_ADAPTIVE_THRESH + 
                                                    cv::CALIB_CB_NORMALIZE_IMAGE + 
                                                    cv::CALIB_CB_FAST_CHECK );

    if ( patternfound )
    {
        cv::cornerSubPix( pd->gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                      cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ) );

        // locate camera;
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
        //tvec[ 2 ] = 10.0;

        bool useExtrinsicGuess = true;

        // Pose estimation
        bool correspondence = cv::solvePnP( qrPts3d, qrPts2d, cameraMatrix, distCoeffs,
                                            rvec, tvec,
                                            useExtrinsicGuess, CV_ITERATIVE );

        if ( pd->debug )
        {
            if ( patternfound )
            {
                cv::Mat preview = mat;
                cv::drawChessboardCorners( preview, sz, cv::Mat(pd->corners), patternfound );
                cv::imshow( "Chessboard", pd->gray );
            }
        }

        if ( !correspondence )
            return false;
        // Transforms Rotation Vector to Matrix
        vRot  = rVec;
        vTran = tvec;

        return true;
    }

    return false;
}


/*
    std::vector<cv::Point2f> corners; //this will be filled by the detected corners

    //CALIB_CB_FAST_CHECK saves a lot of time on images
    //that do not contain any chessboard corners
    bool patternfound = findChessboardCorners(gray, patternsize, corners,
            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
            + CALIB_CB_FAST_CHECK);

    if ( patternfound )
    {
        cornerSubPix( gray, corners, Size(11, 11), Size(-1, -1),
                        TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }

    drawChessboardCorners(img, patternsize, Mat(corners), patternfound);
    */

