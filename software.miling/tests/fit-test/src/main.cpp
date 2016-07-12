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

    pts.push_back( 3.1 );
    pts.push_back( 1.1 );
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


    std::vector<double> ptsFront;
    ptsFront.push_back( 5.0 ); // r
    ptsFront.push_back( 5.0 );
    ptsFront.push_back( 1.0 ); // ri
    ptsFront.push_back( 0.0 );
    ptsFront.push_back( 0.0 ); // ni
    ptsFront.push_back( 1.0 );

    ptsFront.push_back( 8.0 );
    ptsFront.push_back( 8.0 );
    ptsFront.push_back( 0.0 );
    ptsFront.push_back( 1.0 );
    ptsFront.push_back( 1.0 );
    ptsFront.push_back( 0.0 );


    NewtonSam sam;
    cv::Mat A;
    sam.matchPoints( pts, ptsFront, 2.0, A );
    double x = 1.0;
    double y = 1.0;
    double xo = A.at<double>( 0, 0 )*x + A.at<double>( 0, 1 )*y + A.at<double>( 0, 2 );
    double yo = A.at<double>( 1, 0 )*x + A.at<double>( 1, 1 )*y + A.at<double>( 1, 2 );

    x = 2.0;
    y = 1.0;
    xo = A.at<double>( 0, 0 )*x + A.at<double>( 0, 1 )*y + A.at<double>( 0, 2 );
    yo = A.at<double>( 1, 0 )*x + A.at<double>( 1, 1 )*y + A.at<double>( 1, 2 );

    x = 3.0;
    y = 1.2;
    xo = A.at<double>( 0, 0 )*x + A.at<double>( 0, 1 )*y + A.at<double>( 0, 2 );
    yo = A.at<double>( 1, 0 )*x + A.at<double>( 1, 1 )*y + A.at<double>( 1, 2 );

    x = 1.0;
    y = 2.0;
    xo = A.at<double>( 0, 0 )*x + A.at<double>( 0, 1 )*y + A.at<double>( 0, 2 );
    yo = A.at<double>( 1, 0 )*x + A.at<double>( 1, 1 )*y + A.at<double>( 1, 2 );


    return 0;
}







