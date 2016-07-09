
#ifndef __NEWTON_SAM_H_
#define __NEWTON_SAM_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <vector>
#include <iostream>

class NewtonSam
{
public:
    NewtonSam();
    ~NewtonSam();

    bool matchPoints( std::vector<double> & pts, std::vector<double> & ptsOff, double d, cv::Mat & floor2Sample );
    double fi( double * a );
    void  gradFi( double * a, double * dfi );
    void  J( double * a, double * j );

    double a[6], lambda[3];

    cv::Mat XtX, XtY;

    std::vector<double> pts;

    static const double ALPHA;
    static const double MIN_STEP;
    static const double EPS;
    static const int ITER_MAX;
};




#endif








