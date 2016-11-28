
#include "mag1d.h"
#include <functional>


double Mag1D::m = 1.0; // Magnetic momentum.
double Mag1D::r0 = 0.01; // To exclude infinite field value.
double Mag1D::magnetW = 1.0 * 2.0*3.1415926535; // Angular velocity.
double Mag1D::sensorN = 20.0; // Measure frequency.

double Mag1D::sigmaA = 0.1;
double Mag1D::sigmaW = 0.1;
double Mag1D::sigmaB = 0.1;

Mag1D::Mag1D()
{

}

Mag1D::~Mag1D()
{

}

void Mag1D::process()
{
    initSystem();

    double x[4];
    double z[2];
    for ( int i=0; i<5; i++ )
    {
        generateSensorReadings();
        fillState( x, z );
        ukfP.predict( x, x, std::bind( &Mag1D::predict, this, std::placeholders::_1, std::placeholders::_2 ) );
        ukfC.correct( ukfP, z, x, std::bind( &Mag1D::correct, this, std::placeholders::_1, std::placeholders::_2 ) );
        updateState( x, z );
    }
    this->x = 1.0;
    this->v = 0.0;
    this->a = 0.0;

    for ( int i=0; i<100; i++ )
    {
        magnetTimeStep();
        generateSensorReadings();
        fillState( x, z );
        ukfP.predict( x, x, std::bind( &Mag1D::predict, this, std::placeholders::_1, std::placeholders::_2 ) );
        ukfC.correct( ukfP, z, x, std::bind( &Mag1D::correct, this, std::placeholders::_1, std::placeholders::_2 ) );
        updateState( x, z );
    }

    this->senX += 1.0;
    for ( int i=0; i<100; i++ )
    {
        magnetTimeStep();
        generateSensorReadings();
        fillState( x, z );
        ukfP.predict( x, x, std::bind( &Mag1D::predict, this, std::placeholders::_1, std::placeholders::_2 ) );
        ukfC.correct( ukfP, z, x, std::bind( &Mag1D::correct, this, std::placeholders::_1, std::placeholders::_2 ) );
        updateState( x, z );
    }

    for ( int i=0; i<100; i++ )
    {
        magnetTimeStep();
        generateSensorReadings();
        fillState( x, z );
        ukfP.predict( x, x, std::bind( &Mag1D::predict, this, std::placeholders::_1, std::placeholders::_2 ) );
        ukfC.correct( ukfP, z, x, std::bind( &Mag1D::correct, this, std::placeholders::_1, std::placeholders::_2 ) );
        updateState( x, z );
    }
}

void Mag1D::fillState( double  * x, double * z )
{
    x[0] = this->x;
    x[1] = this->v;
    x[2] = this->a;
    x[3] = this->pB;

    //z[0] = this->sa;
    //z[1] = this->sB;
    z[0] = this->sB;
    z[1] = this->sa;
}

void Mag1D::updateState( double * x, double * z )
{
    this->x  = x[0];
    this->v  = x[1];
    this->a  = x[2];
    this->pB = x[3];

    //this->sa = z[0];
    //this->sB = z[1];
    this->sB = z[0];
    this->sa = z[1];
}

void Mag1D::initSystem()
{
    // Real position.
    senX = 1.0;
    senV = 0.0;
    senA = 0.0;

    // Belief.
    x  = 1.0;
    v  = 0.0;
    a  = 0.0;
    pB = 0.0;

    // Initialize ukfp.
    // Position
    ukfP.Rx[0][0] = 0.1; // = 1m/s * dt
    // Speed
    ukfP.Rx[1][1] = 0.1;
    // Acceleration
    ukfP.Rx[2][2] = 0.1;
    // External magnetic field.
    ukfP.Rx[3][3] = 0.1;

    ukfC.Rz[0][0] = 0.0000001;
    //ukfC.Rz[1][1] = 0.01;
}

void Mag1D::magnetTimeStep()
{
    magnetAng += magnetW/sensorN;
}

void Mag1D::realB( double * x, double * B )
{
    double m[3];
    m[0] = cos(magnetAng) * Mag1D::m;
    m[1] = sin(magnetAng) * Mag1D::m;
    m[2] = 0.0;
    double r = sqrt( x[0]*x[0] );
    double mr = (m[0]*x[0])/(r*r);
    //B[0] = (3.0*x[0]*mr - m[0])/(r*r*r);
    //B[1] = (3.0*x[1]*mr - m[1])/(r*r*r);
    //B[2] = (3.0*x[2]*mr - m[2])/(r*r*r);
    B[0] = (3.0*x[0]*mr - m[0])/(r*r*r);
}

void Mag1D::generateSensorReadings()
{
    sa = senA + sigmaA * static_cast<double>( rand() % 128 - 64 ) / 64.0;
    realB( &senX, &senB );
    sB = 1.0 + senB + sigmaB * static_cast<double>( rand() % 128 - 64 ) / 64.0;
}


void Mag1D::predict( double * x, double * y )
{
    // All numbers are in global RF.
    double dt = 1.0/sensorN;
    y[0] = x[0] + x[1]*dt;
    y[1] = x[1] + x[2]*dt;
    y[2] = x[2];
    y[3] = x[3];
}

void Mag1D::correct( double * x, double * z )
{
    // xx, xy, xz, vx, vy, vz, ax, ay, az, angx, angy, angz, wx, wy, wz, bx, by, bz
    // Calculate B in external RF.
    // Convert all needed readings into local sensor RF.
    double a;
    double B;
    realB( x, &B );
    //z[0] = x[2];
    //z[1] = B;
    z[0] = B + x[3];
    z[1] = x[2];
}





