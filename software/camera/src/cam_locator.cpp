
#include "cam_locator.h"
#include "opencv2/calib3d.hpp"
#include <iostream>
#include "text_drawer.h"

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
        double y = static_cast<double>(rows-1-iy) * step;
        for ( int ix=0; ix<cols; ix++ )
        {
            double x = static_cast<double>(ix) * step;
            cv::Point3f pt( x, y, 0.0 );
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

bool CamLocator::findChessboard( const cv::Mat & mat, cv::Mat & camToWorld4x4 )
{

    cv::cvtColor( mat, pd->gray, CV_BGR2GRAY );
    cv::Size sz = cv::Size( pd->cols, pd->rows );

    //cv::imshow( "Grey", pd->gray );


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
        bool useExtrinsicGuess = false;
        if ( !camToWorld4x4.empty() )
        {
            cv::Mat rot = camToWorld4x4( cv::Rect( 0, 0, 3, 3 ) );
            cv::Rodrigues( rot, rvec );
            tvec = camToWorld4x4( cv::Rect( 3, 0, 1, 3 ) ); // (!!!) should checke if dimentions are in their places.
            useExtrinsicGuess = true;
        }
        else
            tvec.at<double>( 2, 0 ) = 1.0;


        // Pose estimation
        bool correspondence;
        try {
            /*
            correspondence = cv::solvePnP( cv::Mat(pd->corners3d), cv::Mat(pd->corners2d), pd->cameraMatrix, pd->distCoeffs,
                                            rvec, tvec,
                                            useExtrinsicGuess, CV_ITERATIVE );
            */
            correspondence = cv::solvePnPRansac( cv::Mat(pd->corners3d), cv::Mat(pd->corners2d), pd->cameraMatrix, pd->distCoeffs,
                                            rvec, tvec,
                                            useExtrinsicGuess, CV_ITERATIVE );
        }
        catch ( cv::Exception & e )
        {
            correspondence = false;
            std::cout << e.what() << std::endl;
        }


        if ( !correspondence )
            return false;
        // Transforms Rotation Vector to Matrix

        cv::Mat rot;
        cv::Rodrigues( rvec, rot );
        cv::Mat objToCam = cv::Mat::zeros( 4, 4, CV_64FC1);
        for ( int iy=0; iy<3; iy++ )
        {
            for ( int ix=0; ix<3; ix++ )
            {
                objToCam.at<double>( iy, ix ) = rot.at<double>( iy, ix );
            }
            objToCam.at<double>( iy, 3 ) = tvec.at<double>( iy, 0 );
        }
        objToCam.at<double>( 3, 3 ) = 1.0;
        camToWorld4x4 = objToCam.inv();

        if ( pd->debug )
        {
            if ( patternfound )
            {
                cv::Mat preview = mat.clone();
                try {
                    int sz = 9;
                    //cv::drawChessboardCorners( preview, sz, cv::Mat(pd->corners2d), patternfound );
                    cv::Point2f pt = pd->corners2d[0];
                    cv::line( preview, cv::Point( pt.x-sz, pt.y ), cv::Point( pt.x+sz, pt.y ), cv::Scalar( 200., 0., 0., 0.2 ), 2  );
                    cv::line( preview, cv::Point( pt.x, pt.y-sz ), cv::Point( pt.x, pt.y+sz ), cv::Scalar( 0., 200., 0., 0.2 ), 2  );
                    pt = pd->corners2d[pd->corners2d.size() - pd->cols ];
                    cv::line( preview, cv::Point( pt.x-sz, pt.y ), cv::Point( pt.x+sz, pt.y ), cv::Scalar( 200., 0., 0., 0.2 ), 2  );
                    cv::line( preview, cv::Point( pt.x, pt.y-sz ), cv::Point( pt.x, pt.y+sz ), cv::Scalar( 0., 200., 0., 0.2 ), 2  );
                    pt = pd->corners2d[pd->corners2d.size() - 1 ];
                    cv::line( preview, cv::Point( pt.x-sz, pt.y ), cv::Point( pt.x+sz, pt.y ), cv::Scalar( 200., 0., 0., 0.2 ), 2  );
                    cv::line( preview, cv::Point( pt.x, pt.y-sz ), cv::Point( pt.x, pt.y+sz ), cv::Scalar( 0., 200., 0., 0.2 ), 2  );
                    cv::imshow( "Chessboard", pd->gray );
                }
                catch ( cv::Exception & e )
                {
                    correspondence = false;
                    std::cout << e.what() << std::endl;
                }
            }
            if ( correspondence )
            {
                std::cout << "x: " << camToWorld4x4.at<double>( 0, 3 ) << " ";
                std::cout << "y: " << camToWorld4x4.at<double>( 1, 3 ) << " ";
                std::cout << "z: " << camToWorld4x4.at<double>( 2, 3 ) << std::endl;
            }
        }

        return true;
    }
    if ( pd->debug )
        cv::imshow( "Chessboard", pd->gray );

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

