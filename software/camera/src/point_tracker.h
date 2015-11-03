
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
    bool writePoints( const std::string & fname = "./points.dat" );
    void clear(); // Stop tracking all points and calc resulting 3D positions.

private:
    void prepareImage( const cv::Mat & frame );
    void calcOpticalFlow();
    void countOpticalFlow();
    void calc3dPoint( std::vector<cv::Point2f> & points, cv::Point3f & at, cv::Point3f & from );

    std::vector<cv::Point2f> & pointHistXy( int row, int col );
    void pushPointHistXy( int row, int col, std::vector<cv::Point2f> & points );
    int longestPointHist() const;
    // How to find points which don't move any more. Just if
    // there is no displacement at a particular frame add new
    // empty array with just current point. And use previous
    // point history for triangulation.

    // To just hold current values.
    cv::Mat projMatrix;
    cv::Mat distCoefs;

    cv::UMat gray,
             grayPrev,
             uflow;
    cv::Mat  flow;

    cv::Size imageSz;

    std::map< int, std::vector<cv::Point2f> > pointHist,
                                              pointHistNew;
    std::vector< cv::Mat > worldHist, worldHistNew;

    // Derived points in space and appropriate camera positions.
    std::vector<cv::Point3f> points3d,
                             camera3d;
};

#endif


