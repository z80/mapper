
#ifndef __NEWTON_CAM_H_
#define __NEWTON_CAM_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <vector>
#include <iostream>

class NewtonCam
{
public:
    NewtonCam();
    ~NewtonCam();

    bool matchPoints( std::vector<cv::Point2d> & knownPts, std::vector<cv::Point2d> & foundPts, cv::Mat & cam2Floor );
    double fi( double * a );
    void  gradFi( double * a, double * dfi );
    void  J( double * a, double * j );

    double a[6], lambda[3];

    cv::Mat XtX, XtY;

    std::vector<cv::Point2d> knownPts;
    std::vector<cv::Point2d> foundPts;

    static const double ALPHA;
    static const double MIN_STEP;
    static const double EPS;
    static const int ITER_MAX;
};




#endif








