
#include "cam_locator.h"
#include "opencv2/calib3d.hpp"
#include <iostream>

class CamLocator::PD
{
public:
    PD() {}
    ~PD() {}

    int rows, cols;
    double step;
    std::vector<cv::Point2f> corners2d;
    std::vector<cv::Point3f> corners3d;
    cv::Mat gray;
    cv::Mat camTrans, camRot;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    bool debug;
};


CamLocator::CamLocator( int rows, int cols, double step )
{
    pd = new PD();
    pd->rows = rows;
    pd->cols = cols;
    pd->step = step;
    pd->debug = true;

    pd->corners3d.resize( rows * cols );
    int index = 0;
    for ( int iy=0; iy<rows; iy++ )
    {
        double y = static_cast<double>(cols-1-iy) * step;
        for ( int ix=0; ix<cols; ix++ )
        {
            double x = static_cast<double>(ix) * step;
            cv::Point3d pt( x, y, 0.0 ); 
            pd->corners3d[ index++ ] = pt;
        }
    }
    
}

CamLocator::~CamLocator()
{
    delete pd;
}

void CamLocator::setCamera( const cv::Mat & cameraMatrix, const cv::Mat & distCoefs )
{
    pd->cameraMatrix = cameraMatrix;
    pd->distCoeffs   = distCoefs;
}

bool CamLocator::findChessboard( const cv::Mat & mat, cv::Mat & vRot, cv::Mat & vTrans )
{

    cv::cvtColor( mat, pd->gray, CV_BGR2GRAY );
    cv::Size sz = cv::Size( pd->cols, pd->rows );

    cv::imshow( "Grey", pd->gray );


    bool patternfound = cv::findChessboardCorners(  pd->gray, sz, pd->corners2d,
                                                    cv::CALIB_CB_ADAPTIVE_THRESH + 
                                                    cv::CALIB_CB_NORMALIZE_IMAGE + 
                                                    cv::CALIB_CB_FAST_CHECK );

    if ( patternfound )
    {
        try {

            cv::cornerSubPix( pd->gray, pd->corners2d, cv::Size(11, 11), cv::Size(-1, -1),
                              cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ) );
        }
        catch ( cv::Exception & e )
        {
            std::cout << e.what() << std::endl;
        }

        // locate camera;
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
        tvec.at<double>( 2, 0 ) = 10.0;

        const bool useExtrinsicGuess = true;

        // Pose estimation
        bool correspondence;
        try {
            correspondence = cv::solvePnP( pd->corners3d, pd->corners2d, pd->cameraMatrix, pd->distCoeffs,
                                            rvec, tvec,
                                            useExtrinsicGuess, CV_ITERATIVE );
        }
        catch ( cv::Exception & e )
        {
            correspondence = false;
            std::cout << e.what() << std::endl;
        }

        if ( pd->debug )
        {
            if ( patternfound )
            {
                cv::Mat preview = mat;
                try {
                    cv::drawChessboardCorners( preview, sz, cv::Mat(pd->corners2d), patternfound );
                }
                catch ( cv::Exception & e )
                {
                    correspondence = false;
                    std::cout << e.what() << std::endl;
                }
                cv::imshow( "Chessboard", pd->gray );
            }
            if ( correspondence )
            {
                std::cout << "x: " << tvec.at<double>( 0, 0 ) << " ";
                std::cout << "y: " << tvec.at<double>( 1, 0 ) << " ";
                std::cout << "z: " << tvec.at<double>( 2, 0 ) << std::endl;
            }
        }

        if ( !correspondence )
            return false;
        // Transforms Rotation Vector to Matrix
        vRot   = rvec;
        vTrans = tvec;

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

