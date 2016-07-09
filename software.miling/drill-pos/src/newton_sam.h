
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

    std::vector<double> pts;
};




#endif








