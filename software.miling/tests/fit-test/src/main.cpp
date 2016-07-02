#include <stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <limits>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio/videoio.hpp"

#include "newton_sam.h"

int main()
{
    std::vector<double> pts;
    pts.push_back( 1.0 ); // r
    pts.push_back( 1.0 );
    pts.push_back( 1.0 ); // ri
    pts.push_back( 0.0 );
    pts.push_back( 0.0 ); // ni
    pts.push_back( 1.0 );

    pts.push_back( 2.0 );
    pts.push_back( 1.0 );
    pts.push_back( 1.0 );
    pts.push_back( 0.0 );
    pts.push_back( 0.0 );
    pts.push_back( 1.0 );

    pts.push_back( 3.0 );
    pts.push_back( 1.2 );
    pts.push_back( 1.0 );
    pts.push_back( 0.0 );
    pts.push_back( 0.0 );
    pts.push_back( 1.0 );

    pts.push_back( 1.0 );
    pts.push_back( 2.0 );
    pts.push_back( 0.0 );
    pts.push_back( 1.0 );
    pts.push_back( 1.0 );
    pts.push_back( 0.0 );

    NewtonSam sam;
    cv::Mat A;
    sam.matchPoints( pts, 2.0, A );

    return 0;
}







