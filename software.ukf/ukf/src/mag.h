
#ifndef __MAG_H_
#define __MAG_H_

#include "matrix2.hpp"
#include "vector2.hpp"

template<int DIM, typename FLOAT> const Math::Vector<DIM, FLOAT> & m2a( const Math::Matrix<DIM, FLOAT> & m )
{
    Math::Vector<DIM, FLOAT> q;
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

template<int DIM, typename FLOAT> const Math::Matrix<DIM, FLOAT> & q2m( const Math::Vector<DIM, FLOAT> & q )
{
    Math::Matrix<DIM, FLOAT> m;

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



//B = mu/(4pi)( 3r(m,r)/r^5 - m/r^3 )


class Mag
{
public:
    Mag();
    ~Mag();

    // Magnet state.
    void   magnetTimeStep();
    double magnetAng; // Magnet rotation angle.
    void realB( double * x, double * B );

    // State variables.
    double senX[3];
    double senV[3];
    double senA[3];

    double senAng[3];
    double senW[3];

    double Be[3];

    // Sensor readings.
    double sa[3];
    double sw[3];
    double sB[3];

    // Predicted sensor readings.
    double pa[3];
    double pw[3];
    double pB[3];

    // Prediction step.
    void predictionX();
    void predictionG();
    void predictionB();

    // System params.
    //static int stepsPerRev; // Steps per revolution.
    static double m;        // Magnetic momentum.
    static double r0;       // To exclude infinite field value.
    static double magnetW;  // Angular velocity.
    static double sensorN;  // Measure frequency.
};


#endif


