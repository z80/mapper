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
#include "ukfm.h"

void pr( double * x, double * xn );
void sen01( double * x, double * z );
void sen02( double * x, double * z );

int main()
{
    UkfP<double, 3> ukfp;
    ukfp.Rx[0][0] = 2.0; // cm;
    ukfp.Rx[1][1] = 2.0; // cm;
    ukfp.Rx[2][2] = 2.0; // cm;

    UkfC<double, 3, 1> ukfc01;
    UkfC<double, 3, 1> ukfc02;

    double x[3];
    x[0] = 0.0;
    x[1] = 0.0;
    x[2] = 0.0;
    double z01[1];
    z01[0] = 0.0;
    double z02[1];
    z02[0] = 0.0;

    srand( 0 );
    double P[3][3];

    for ( auto i=0; i<100; i++ )
    {
        for ( auto j=0; j<10; j++ )
        {
            z01[0] = static_cast<double>( ((rand() % 128) - 64 ) ) / 64.0;
            z02[0] = static_cast<double>( ((rand() % 128) - 64 ) ) / 64.0; // + static_cast<double>( i );
            ukfp.predict( x, x, std::bind( &pr, std::placeholders::_1, std::placeholders::_2 ) );
            ukfc01.correct( ukfp, z01, x, sen01 );
            //ukfp.predict( x, x, std::bind( &pr, std::placeholders::_1, std::placeholders::_2 ) );
            //ukfc02.correct( ukfp, z02, x, sen02 );
            ukfp.stateNoiseCov( P );
        }
        ukfp.predict( x, x, std::bind( &pr, std::placeholders::_1, std::placeholders::_2 ) );
        z02[0] = static_cast<double>( ((rand() % 128) - 64 ) ) / 64.0; // + static_cast<double>( i );
        ukfc02.correct( ukfp, z02, x, sen02 );
        ukfp.stateNoiseCov( P );
    }

    return 0;
}

const double dt = 1.0;
void pr( double * x, double * xn )
{

    xn[0] = x[0] + x[1] * dt;
    xn[1] = x[1] + x[2] * dt;
    xn[2] = x[2];
}

void sen01( double * x, double * z )
{
    z[0] = x[2];
}

void sen02( double * x, double * z )
{
    z[0] = x[0];
}








