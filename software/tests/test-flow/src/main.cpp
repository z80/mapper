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
void appPoint( int row, int col, cv::Point2f displacement );
void framePoints();

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

        cv::Mat image = cv::imread( fname );

        //if ( first )
        cv::Mat undistorted; // = image.clone();
        cv::undistort( image, undistorted, cameraMatrix, distCoeffs );



        cvtColor( undistorted, gray, cv::COLOR_BGR2GRAY );
        if ( !grayPrev.empty() )
        {
            cv::calcOpticalFlowFarneback( grayPrev, gray, uflow, 0.5, 8, 15, 3, 5, 1.2, 0 );
            uflow.copyTo( flow );
            cv::add( flowAccum, flow, flowAccum );
        }
        grayPrev = gray.clone();

        if ( first )
        {
            cv::calcOpticalFlowFarneback( gray, gray, uflow, 0.5, 3, 15, 3, 5, 1.2, 0 );
            uflow.copyTo( flow );
            flowAccum = flow.clone();
            imgFirst = undistorted.clone();
            first = false;
        }
        else
        {
            if ( i==(cnt-1) )
                    imgLast = undistorted.clone();
        }

        cv::imshow( "image", undistorted );
        cv::waitKey( 1 );
    }

    fs.release();

    int step = 64;
    cv::Scalar color = cv::Scalar(0, 255, 0);
    for(int y = 0; y < imgFirst.rows; y += step)
        for(int x = 0; x < imgFirst.cols; x += step)
        {
            const cv::Point2f & fxy = flowAccum.at<cv::Point2f>(y, x);
            cv::line(imgFirst, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            cv::circle(imgFirst, cv::Point(x,y), 2, color, -1);
        }


    cv::imshow( "first", imgFirst );
    cv::imshow( "last",  imgLast );

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
            frames[index] = pts;
        }
    }
}


void appPoint( int row, int col, cv::Point2f displacement )
{
}

void framePoints()
{
}






