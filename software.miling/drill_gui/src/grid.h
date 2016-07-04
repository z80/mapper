
#ifndef __GRID_H_
#define __GRID_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <climits>

#include <stlsurf.hpp>
#include <stlreader.hpp>
#include <triangle.hpp>


class Grid
{
public:
    Grid( ocl::STLSurf * surf );
    ~Grid();

    void setCutter( double d, double l );
    void setPrecision( double prec );
    void setZInterval( double zFrom, double zTo );
    void setPoints( const cv::Point2d & at, double d );

    void run();

    ocl::STLSurf * surf;
};



#endif


