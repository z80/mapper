
#include "point_tracker.h"


PointTracker::PointTracker()
{
    imageSz = cv::Size( 160, 120 );
}

PointTracker::~PointTracker()
{

}

void PointTracker::setCameraMatrix( const cv::Mat & projMatrix, const cv::Mat & distCoefs )
{
    this->projMatrix = projMatrix.clone();
    this->distCoefs  = distCoefs.clone();
}

void PointTracker::process( const cv::Mat & frame, const cv::Mat & worldM )
{

}

void PointTracker::finish()
{

}

void PointTracker::prepareImage( const cv::Mat & frame )
{
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    cv::resize( gray, gray, imageSz );
}

void PointTracker::calcOpticalFlow()
{
    if( !grayPrev.empty() )
        calcOpticalFlowFarneback( grayPrev, gray, uflow, 0.5, 6, 30, 3, 5, 1.2, 0);
    grayPrev = gray.clone();
}

void PointTracker::countOpticalFlow()
{
    for ( int i=0; i<imageSz.height; i++ )
    {
        for ( int j=0; j<imageSz.width; j++ )
        {
            int index = i + imageSz.width * i;
            cv::Point2f dr = uflow.at<cv::Point2f>(i, j);
        }
    }
}

void PointTracker::calc3dPoints()
{

}

void PointTracker::calc3dPoint()
{

}



