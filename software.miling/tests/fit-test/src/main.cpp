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
    pts.push_back( -1.4677 ); // r
    pts.push_back( 25.464 );
    pts.push_back( 6.8 ); // ri
    pts.push_back( 6.8 );
    pts.push_back( 0.0 ); // ni
    pts.push_back( 92.48 );

    pts.push_back( -1.694 ); // r
    pts.push_back( 27.44 );
    pts.push_back( 6.8 ); // ri
    pts.push_back( 6.8 );
    pts.push_back( 0.0 ); // ni
    pts.push_back( 92.48 );

    pts.push_back( -0.87209 ); // r
    pts.push_back( 28.044 );
    pts.push_back( 6.8 ); // ri
    pts.push_back( 6.8 );
    pts.push_back( 0.0 ); // ni
    pts.push_back( 92.48 );

    pts.push_back( -0.31734 );
    pts.push_back( 24.74 );
    pts.push_back( 6.8 );
    pts.push_back( 0.0 );
    pts.push_back( 92.48 );
    pts.push_back( 0.0 );

    pts.push_back( 2.1256 );
    pts.push_back( 25.977 );
    pts.push_back( 6.8 );
    pts.push_back( 0.0 );
    pts.push_back( 92.48 );
    pts.push_back( 0.0 );

    pts.push_back( 4.3727 );
    pts.push_back( 25.29 );
    pts.push_back( 6.8 );
    pts.push_back( 0.0 );
    pts.push_back( 92.48 );
    pts.push_back( 0.0 );


    std::vector<double> ptsFront;
    ptsFront.push_back( -2.901 ); // r
    ptsFront.push_back( 26.823 );
    ptsFront.push_back( 6.8 ); // ri
    ptsFront.push_back( 6.8 );
    ptsFront.push_back( 0.0 ); // ni
    ptsFront.push_back( 92.48 );

    ptsFront.push_back( 3.2432 );
    ptsFront.push_back( 23.5 );
    ptsFront.push_back( 6.8 );
    ptsFront.push_back( 0.0 );
    ptsFront.push_back( 92.48 );
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







