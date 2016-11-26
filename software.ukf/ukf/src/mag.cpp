
#include "mag.h"
#include <math.h>

double Mag::m = 1.0; // Magnetic momentum.
double Mag::r0 = 0.01; // To exclude infinite field value.
double Mag::magnetW = 0.5 * 2.0*3.1415926535; // Angular velocity.
double Mag::sensorN = 1.0; // Measure frequency.


void Mag::magnetTimeStep()
{
    magnetAng += magnetW/sensorN;
}

void Mag::realB( double * x, double * B )
{
    double m[3];
    m[0] = cos(magnetAng) * Mag::m;
    m[1] = sin(magnetAng) * Mag::m;
    m[2] = 0.0;
    double r = sqrt( x[0]*x[0] + x[1]*x[1] + x[2]*x[2] );
    double mr = (m[0]*x[0] + m[1]*x[1] + m[2]*x[2])/(r*r);
    B[0] = (3.0*x[0]*mr - m[0])/(r*r*r);
    B[1] = (3.0*x[1]*mr - m[1])/(r*r*r);
    B[2] = (3.0*x[2]*mr - m[2])/(r*r*r);
}



