#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"
#include <iostream>

// Linear camera motion.
const double stepX = 1.0;

void drawText( cv::Mat & image, const std::string & stri, int line=0 );
bool saveImage( cv::FileStorage & fs, const cv::Mat & image, int & index );

int main()
{
    std::cout << "Built with OpenCV " << CV_VERSION << std::endl;

    int ssIndex = 0;

    cv::Mat image;
    cv::VideoCapture capture;
    capture.open(0);

    cv::FileStorage fs( "./data/ss.xml", cv::FileStorage::WRITE ); // Read the settings
    if (!fs.isOpened())
    {
          std::cout << "Could not open the configuration file" << std::endl;
          return -1;
    }

    if(capture.isOpened())
    {
        std::cout << "Capture is opened" << std::endl;

        for(;;)
        {
            capture >> image;
            if(image.empty())
                break;

            cv::imshow( "camera", image );

            uchar key = (uchar)cv::waitKey( 33 );
            if( key == 'q' )
                break;
            if( key == 's' )
            {
                saveImage( fs, image, ssIndex );
            }

        }
    }

    fs << "frames_cnt" << ssIndex;
    fs.release();
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

bool saveImage( cv::FileStorage & fs, const cv::Mat & image, int & index )
{
    char imageName[256];

    std::vector<int> compression_params;
    compression_params.push_back( cv::IMWRITE_PNG_COMPRESSION );
    compression_params.push_back( 9 );

    sprintf( imageName, "./data/image%d.png", index );
    bool res = cv::imwrite( imageName, image, compression_params );

    if ( res )
    {
        char frameName[ 256 ];

        double x = static_cast<double>( index ) * stepX;

        cv::Mat cameraPos = cv::Mat::eye( 4, 4, CV_64FC1 );
        cameraPos.at<double>( 0, 3 ) = x;

        sprintf( frameName, "img%d", index );
        fs << frameName << imageName;
        sprintf( frameName, "pos%d", index );
        fs << frameName << cameraPos;
    }
    index += 1;

    return res;
}






