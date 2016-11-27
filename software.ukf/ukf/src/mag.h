
#ifndef __MAG_H_
#define __MAG_H_

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



template<typename FLOAT> const Math::Vector<4, FLOAT> m2q( const Math::Matrix<3, FLOAT> & m )
{
    Math::Vector<4, FLOAT> q;
    FLOAT trace = m[0][0] + m[1][1] + m[2][2];
    if( trace > 0.0 )
    {
        FLOAT s = 0.5 / sqrt( trace + 1.0);
        q[0] = 0.25 / s;
        q[1] = ( m[2][1] - m[1][2] ) * s;
        q[2] = ( m[0][2] - m[2][0] ) * s;
        q[3] = ( m[1][0] - m[0][1] ) * s;
    }
    else
    {
        if ( m[0][0] > m[1][1] && m[0][0] > m[2][2] )
        {
            FLOAT s = 2.0 * sqrtf( 1.0 + m[0][0] - m[1][1] - m[2][2]);
            q[0] = (m[2][1] - m[1][2] ) / s;
            q[1] = 0.25 * s;
            q[2] = (m[0][1] + m[1][0] ) / s;
            q[3] = (m[0][2] + m[2][0] ) / s;
        }
        else if (m[1][1] > m[2][2])
        {
            FLOAT s = 2.0 * sqrtf( 1.0 + m[1][1] - m[0][0] - m[2][2]);
            q[0] = (m[0][2] - m[2][0] ) / s;
            q[1] = (m[0][1] + m[1][0] ) / s;
            q[2] = 0.25 * s;
            q[3] = (m[1][2] + m[2][1] ) / s;
        }
        else
        {
            FLOAT s = 2.0 * sqrtf( 1.0 + m[2][2] - m[0][0] - m[1][1] );
            q[0] = (m[1][0] - m[0][1] ) / s;
            q[1] = (m[0][2] + m[2][0] ) / s;
            q[2] = (m[1][2] + m[2][1] ) / s;
            q[3] = 0.25 * s;
        }
    }
    return q;
}

template<typename FLOAT> const Math::Matrix<3, FLOAT> q2m( const Math::Vector<4, FLOAT> & q )
{
    Math::Matrix<3, FLOAT> m;

    FLOAT qw = q[0];
    FLOAT qx = q[1];
    FLOAT qy = q[2];
    FLOAT qz = q[3];

    m[0][0] = 1.0 - 2.0*(qy*qy - qz*qz);
    m[0][1] = 2.0*(qx*qy - qz*qw);
    m[0][2] = 2.0*(qx*qz + qy*qw);
    m[1][0] = 2.0*(qx*qy + qz*qw);
    m[1][1] = 1.0 - 2.0*(qx*qx - qz*qz);
    m[1][2] = 2.0*(qy*qz - qx*qw);
    m[2][0] = 2.0*(qx*qz - qy*qw);
    m[2][1] = 2.0*(qy*qz + qx*qw);
    m[2][2] = 1.0 - 2.0*(qx*qx - qy*qy);

    return m;
}

template<typename FLOAT> const Math::Vector<4, FLOAT> a2q( const Math::Vector<3, FLOAT> & a )
{
    FLOAT ang = sqrt( a[0]*a[0] + a[1]*a[1] + a[2]*a[2] );
    Math::Vector<4, FLOAT> q;
    if ( ang <= 0.0001 )
    {
        q[0] = 1.0;
        q[1] = q[2] = q[3] = 0.0;
    }
    else
    {
        FLOAT n[3];
        n[0] = a[0]/ang;
        n[1] = a[1]/ang;
        n[2] = a[2]/ang;
        FLOAT c = cos( ang );
        FLOAT s = sin( ang );
        q[0] = c;
        q[1] = s*n[0];
        q[2] = s*n[1];
        q[3] = s*n[2];
    }
    return q;
}

template<typename FLOAT> const Math::Vector<3, FLOAT> q2a( const Math::Vector<4, FLOAT> & q )
{
    FLOAT c = q[0];
    FLOAT s = sqrt( q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
    FLOAT ang = atan2( s, c );
    Math::Vector<3, FLOAT> a;
    if ( abs( ang ) < 0.0001 )
    {
        a[0] = 0.0;
        a[1] = 0.0;
        a[2] = 0.0;
    }
    else
    {
        a[0] = q[1]/s*ang;
        a[1] = q[2]/s*ang;
        a[2] = q[3]/s*ang;
    }
    return a;
}




//B = mu/(4pi)( 3r(m,r)/r^5 - m/r^3 )


class Mag
{
public:
    Mag();
    ~Mag();

    // Magnet state.
    void   initSystem();
    void   magnetTimeStep();
    double magnetAng; // Magnet rotation angle.
    void realB( double * x, double * B );

    // Generate X.
    void generateSensorReadings();

    // Predict.
    void predict();

    // State variables.
    double senX[3];
    double senV[3];
    double senA[3];

    double senAng[3];
    double senW[3];

    double senB[3];

    // Sensor readings.
    double sa[3];
    double sw[3];
    double sB[3];

    // Predicted sensor readings and belief.
    double pB[3]; // Field value.

    // Beief.
    double x[3];
    double v[3];
    double a[3];

    double ang[3];
    double angW[3];

    UkfP<double, 18>    ukfP;
    UkfC<double, 18, 9> ukfC;


    // Prediction step.
    void predict( double * x, double * y );
    void estimate( double * x, double * z );
    // Auxilary function to get world to local RF transformation.
    Math::Matrix<3, double> toWorldA( double * x );

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


