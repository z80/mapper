
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

// Quaternion normalization.
template<typename FLOAT> void quatNorm( Math::Vector<4, FLOAT> & a )
{
    FLOAT l = 1.0/sqrt( a[0]*a[0] + a[1]*a[1] + a[2]*a[2] + a[3]*a[3] );
    a[0] *= l;
    a[1] *= l;
    a[2] *= l;
    a[3] *= l;
}

// Quaternion multiplication.
template<typename FLOAT> const Math::Vector<4, FLOAT> quatMult( const Math::Vector<4, FLOAT> & a,
                                                                const Math::Vector<4, FLOAT> & b  )
{
    FLOAT w, x, y ,z;
    //x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
    x   =  a[1] * b[0] + a[2] * b[3] - a[3] * b[2] + a[0] * b[1];
    //y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
    y   = -a[1] * b[3] + a[2] * b[0] + a[3] * b[1] + a[0] * b[2];
    //z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    z   =  a[1] * b[2] - a[2] * b[1] + a[3] * b[0] + a[0] * b[3];
    //w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
    w   = -a[1] * b[1] - a[2] * b[2] - a[3] * b[3] + a[0] * b[0];
    Math::Vector<4, FLOAT> ab;
    ab[0] = w;
    ab[1] = x;
    ab[2] = y;
    ab[3] = z;
    quatNorm( ab );
    return ab;
}

// Quaternion conjugate.
template<typename FLOAT> Math::Vector<4, FLOAT> quatConj( const Math::Vector<4, FLOAT> & a )
{
    Math::Vector<4, FLOAT> b;
    b[0] = a[0];
    b[1] = -a[1];
    b[2] = -a[2];
    b[3] = -a[3];
    return b;
}

// Quaternion vector transform.
template<typename FLOAT> Math::Vector<3, FLOAT> quatNorm( const Math::Vector<3, FLOAT> & v,
                                                          const Math::Vector<4, FLOAT> & q )
{
    FLOAT A[3][3];
    // q.w²+q.x²-q.y²-q.z²
    A[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    // 2*q.x*q.y - 2*q.w*q.z
    A[0][1] = 2.0 * (q[1]*q[2] - q[0]*q[3]);
    // 2*q.x*q.z + 2*q.w*q.y
    A[0][2] = 2.0 * (q[1]*q[3] + q[0]*q[2]);

    // 2*q.x*q.y + 2*q.w*q.z
    A[1][0] = 2.0 * (q[1]*q[2] + q[0]*q[3]);
    // q.w²-q.x² + q.y²-q.z²
    A[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    // 2*q.y*q.z - 2*q.w*q.x
    A[1][2] = 2.0 * (q[2]*q[3] - q[0]*q[1]);

    // 2*q.x*q.z - 2*q.w*q.y
    A[2][0] = 2.0 * (q[1]*q[3] - q[0]*q[2]);
    // 2*q.y*q.z + 2*q.w*q.x
    A[2][1] = 2.0 * (q[2]*q[3] + q[0]*q[1]);
    // q.w²-q.x²-q.y²+q.z²
    A[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    // Transforming vector.
    Math::Vector<3, FLOAT> r;
    r[0] = A[0][0]*v[0] + A[0][1]*v[1] + A[0][2]*v[2];
    r[1] = A[1][0]*v[0] + A[1][1]*v[1] + A[1][2]*v[2];
    r[2] = A[2][0]*v[0] + A[2][1]*v[1] + A[2][2]*v[2];

    // Returning result.
    return r;
}





//B = mu/(4pi)( 3r(m,r)/r^5 - m/r^3 )


class Mag
{
public:
    Mag();
    ~Mag();

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

    // x3, v3, a3, ang4, w3, [B3]
    UkfP<double, 16>    ukfP;
    UkfC<double, 16, 9> ukfC;


    // Prediction step.
    void predict( double * x, double * y );
    void correct( double * x, double * z );
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


