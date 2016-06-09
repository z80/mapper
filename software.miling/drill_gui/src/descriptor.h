
#ifndef __DESCRIPTOR_H_
#define __DESCRIPTOR_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <vector>
#include <iostream>

class Descriptor
{
    Descriptor( double eps=0.5 );
    Descriptor( const Descriptor & inst );
    ~Descriptor();

    Descriptor & operator=( const Descriptor & inst );
    bool operator==( const Descriptor & inst );
    void build( std::vector<std::vector<cv::Point2d>> & rects, int rectInd, int ptInd, double range = 10.0 );

    std::vector<double> dists;
    double dotProd, L1, L2;
    double eps;
};


#endif

