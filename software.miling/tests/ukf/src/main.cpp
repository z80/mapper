#include <stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <limits>
#include <functional>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio/videoio.hpp"


#include "ukf.h"

void pr( double * x, double * xn );
void sen( double * x, double * z );

int main()
{
    Ukf<double, 1, 1 > ukf;
    ukf.init();
    ukf.Rx[0][0] = 2.0; // cm;
    ukf.Rz[0][0] = 2.0; // cm;


    double x[1];
    x[0] = 0.0;
    double z[1];
    z[0] = 0.0;
    srand( 0 );

    for ( auto i=0; i<100; i++ )
    {
        x[0] = static_cast<double>( ((rand() % 128) - 64 ) ) / 64.0;
        z[0] = static_cast<double>( ((rand() % 128) - 64 ) ) / 64.0;
        ukf.predict( x, std::bind( &pr, std::placeholders::_1, std::placeholders::_2 ) );
        ukf.correct( z, sen );
    }

    return 0;
}

void pr( double * x, double * xn )
{
    xn[0] = x[0];
}

void sen( double * x, double * z )
{
    z[0] = x[0];
}








