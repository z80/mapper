
#include "point_tracker.h"


PointTracker::PointTracker()
{

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

