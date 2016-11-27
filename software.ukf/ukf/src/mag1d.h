
#ifndef __MAG1D_H_
#define __MAG1D_H_

#include "mag.h"
#include "matrix2.hpp"
#include "vector2.hpp"
#include "ukfm.h"

#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <limits>
#include <functional>







//B = mu/(4pi)( 3r(m,r)/r^5 - m/r^3 )


class Mag1D
{
public:
    Mag1D();
    ~Mag1D();

    void process();
    void fillState( double  * x, double * z );
    void updateState( double * x, double * z );

    // Magnet state.
    void   initSystem();
    void   magnetTimeStep();
    double magnetAng; // Magnet rotation angle.
    void realB( double * x, double * B );

    // Generate X.
    void generateSensorReadings();

    // State variables.
    double senX;
    double senV;
    double senA;

    double senB;

    // Sensor readings.
    double sa;
    double sB;

    // Predicted sensor readings and belief.
    double pB; // Field value.

    // Beief.
    double x;
    double v;
    double a;

    UkfP<double, 3>    ukfP;
    UkfC<double, 3, 1> ukfC;


    // Prediction step.
    void predict( double * x, double * y );
    void correct( double * x, double * z );

    // System params.
    //static int stepsPerRev; // Steps per revolution.
    static double m;        // Magnetic momentum.
    static double r0;       // To exclude infinite field value.
    static double magnetW;  // Angular velocity.
    static double sensorN;  // Measure frequency.
    // Sensor precision.
    static double sigmaA;
    static double sigmaW;
    static double sigmaB;
};


#endif


