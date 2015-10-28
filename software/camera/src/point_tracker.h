
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"

#include <list>
#include <vector>
#include <map>

#ifndef __POINT_TRACKER_H_
#define __POINT_TRACKER_H_

class PointTracker
{
public:
    PointTracker();
    ~PointTracker();

    void setCameraMatrix( const cv::Mat & projMatrix, const cv::Mat & distCoefs );
    void process( const cv::Mat & frame, const cv::Mat & worldM );
    void finish(); // Stop tracking all points and calc resulting 3D positions.

private:
    // To just hold current values.
    cv::Mat projMatrix;
    cv::Mat distCoefs;

    cv::UMat gray,
             grayPrev,
             flow;

    cv::Size imageSz;

    std::map< int, std::vector<cv::Point2f> > pointHist,
                                              pointHistNew;
    std::vector< cv::Mat > worldHist;
};

#endif


