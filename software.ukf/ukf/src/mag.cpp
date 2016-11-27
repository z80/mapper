
#include "mag.h"
#include <functional>


double Mag::m = 1.0; // Magnetic momentum.
double Mag::r0 = 0.01; // To exclude infinite field value.
double Mag::magnetW = 0.5 * 2.0*3.1415926535; // Angular velocity.
double Mag::sensorN = 1.0; // Measure frequency.

double Mag::sigmaA = 0.05;
double Mag::sigmaW = 0.1;
double Mag::sigmaB = 0.1;

Mag::Mag()
{

}

Mag::~Mag()
{

}

void Mag::process()
{
    initSystem();

    double x[15];
    double z[9];
    generateSensorReadings();
    fillState( x, z );
    ukfP.predict( x, x, std::bind( &Mag::predict, this, std::placeholders::_1, std::placeholders::_2 ) );
    ukfC.correct( ukfP, z, x, std::bind( &Mag::correct, this, std::placeholders::_1, std::placeholders::_2 ) );
    updateState( x, z );

    for ( int i=0; i<10; i++ )
    {
        magnetTimeStep();
        generateSensorReadings();
        fillState( x, z );
        ukfP.predict( x, x, std::bind( &Mag::predict, this, std::placeholders::_1, std::placeholders::_2 ) );
        ukfC.correct( ukfP, z, x, std::bind( &Mag::correct, this, std::placeholders::_1, std::placeholders::_2 ) );
        updateState( x, z );
    }
}

void Mag::fillState( double  * x, double * z )
{
    for ( int i=0; i<3; i++ )
    {
        x[i]   = this->x[i];
        x[i+3] = this->v[i];
        x[i+6] = this->a[i];
        x[i+9] = this->ang[i];
        x[i+12] = this->angW[i];

        z[i]   = this->sa[i];
        z[i+3] = this->sw[i];
        z[i+6] = this->sB[i];
    }
}

void Mag::updateState( double * x, double * z )
{
    for ( int i=0; i<3; i++ )
    {
        this->x[i]    = x[i];
        this->v[i]    = x[i+3];
        this->a[i]    = x[i+6];
        this->ang[i]  = x[i+9];
        this->angW[i] = x[i+12];

        this->sa[i] = z[i];
        this->sw[i] = z[i+3];
        this->sB[i] = z[i+6];
    }
}

void Mag::initSystem()
{
    // Real position.
    senX[0] = 1.0;
    senX[1] = 0.0;
    senX[2] = 0.0;

    senV[0] = 0.0;
    senV[1] = 0.0;
    senV[2] = 0.0;

    senA[0] = 0.0;
    senA[1] = 0.0;
    senA[2] = 0.0;

    senAng[0] = 0.0;
    senAng[1] = 0.0;
    senAng[2] = 0.0;

    senW[0] = 0.0;
    senW[1] = 0.0;
    senW[2] = 0.0;

    // Belief.
    x[0] = 1.0;
    x[1] = 0.0;
    x[2] = 0.0;

    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 0.0;

    a[0] = 0.0;
    a[1] = 0.0;
    a[2] = 0.0;

    ang[0] = 0.0;
    ang[1] = 0.0;
    ang[2] = 0.0;

    angW[0] = 0.0;
    angW[1] = 0.0;
    angW[2] = 0.0;

    // Initialize ukfp.
    // Position
    ukfP.Rx[0][0] = 1.0 / sensorN; // = 1m/s * dt
    ukfP.Rx[1][1] = 1.0 / sensorN;
    ukfP.Rx[2][2] = 1.0 / sensorN;
    // Speed
    ukfP.Rx[3][3] = 1.0 / sensorN;
    ukfP.Rx[4][4] = 1.0 / sensorN;
    ukfP.Rx[5][5] = 1.0 / sensorN;
    // Acceleration
    ukfP.Rx[6][6] = 0.01;
    ukfP.Rx[7][7] = 0.01;
    ukfP.Rx[8][8] = 0.01;
    // Angle
    ukfP.Rx[9][9]   = 0.01;
    ukfP.Rx[10][10] = 0.01;
    ukfP.Rx[11][11] = 0.01;
    // Angular velocity
    ukfP.Rx[12][12] = 0.01;
    ukfP.Rx[13][13] = 0.01;
    ukfP.Rx[14][14] = 0.01;
}

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

void Mag::generateSensorReadings()
{
    // It is also necessary to convert readings to local sensor RF.
    Math::Matrix<3, double> A;
    Math::Vector<4, double> q;
    q = a2q<double>( senAng );
    A = q2m<double>( q );


    for ( int i=0; i<3; i++ )
    {
        sa[i] = 0.0;
        for ( int j=0; j<3; j++ )
        {
            double a = (j==2) ? (senA[j] + 9.81) : senA[j]; // For Z acceleration first add gravity.
            sa[i] += a * A[j][i]; // Transposed A to get value in local RF.
        }
    }
    sa[0] = sa[0] + sigmaA * static_cast<double>( rand() % 128 - 64 ) / 64.0;
    sa[1] = sa[1] + sigmaA * static_cast<double>( rand() % 128 - 64 ) / 64.0;
    sa[2] = sa[2] + sigmaA * static_cast<double>( rand() % 128 - 64 ) / 64.0;
    //

    for ( int i=0; i<3; i++ )
    {
        sw[i] = 0.0;
        for ( int j=0; j<3; j++ )
            sw[i] += senW[j] * A[j][i]; // Transposed A to get value in local RF.
    }
    sw[0] = sw[0] + sigmaW * static_cast<double>( rand() % 128 - 64 ) / 64.0;
    sw[1] = sw[1] + sigmaW * static_cast<double>( rand() % 128 - 64 ) / 64.0;
    sw[2] = sw[2] + sigmaW * static_cast<double>( rand() % 128 - 64 ) / 64.0;

    realB( senX, senB );

    for ( int i=0; i<3; i++ )
    {
        sB[i] = 0.0;
        for ( int j=0; j<3; j++ )
            sB[i] += senB[j] * A[j][i]; // Transposed A to get value in local RF.
    }
    sB[0] = sB[0] + sigmaB * static_cast<double>( rand() % 128 - 64 ) / 64.0;
    sB[1] = sB[1] + sigmaB * static_cast<double>( rand() % 128 - 64 ) / 64.0;
    sB[2] = sB[2] + sigmaB * static_cast<double>( rand() % 128 - 64 ) / 64.0;
}


void Mag::predict( double * x, double * y )
{
    // All numbers are in global RF.
    double dt = 1.0/sensorN;
    for ( int i=0; i<3; i++ )
    {
        y[i]   = x[i] + x[i+3]*dt;
        y[i+3] = x[i+3] + x[i+6]*dt;
        y[i+6] = x[i+6];
        //y[i+9] = x[i+9] + x[i+12]*dt; // Not sure if I can add angles (may be for small angles???).
    }
    Math::Vector<3, double> a;
    a[0] = x[9];
    a[1] = x[10];
    a[2] = x[11];
    Math::Vector<4, double> q = a2q<double>( a );
    Math::Matrix<3, double> A = q2m<double>( q );
    a[0] = x[12] * dt;
    a[1] = x[13] * dt;
    a[2] = x[14] * dt;
    q = a2q( a );
    Math::Matrix<3, double> B = q2m<double>( q );
    A = A * B;
    q = m2q<double>( A );
    a = q2a<double>( q );
    for ( int i=0; i<3; i++ )
    {
        y[i+9]  = a[i];
        y[i+12] = x[i+12];
    }
}

void Mag::correct( double * x, double * z )
{
    // xx, xy, xz, vx, vy, vz, ax, ay, az, angx, angy, angz, wx, wy, wz, bx, by, bz
    // Calculate B in external RF.
    // Convert all needed readings into local sensor RF.
    double a[3];
    double w[3];
    double B[3];
    realB( x, B );
    for ( int i=0; i<3; i++ )
    {
        a[i] = x[i+6];
        w[i] = x[i+12];
    }
    // Convert to local RF.
    Math::Matrix<3, double> A = toWorldA( x );
    // Concerning acceleration first add gravity to it.
    a[0] += 9.81;
    for ( int i=0; i<3; i++ )
    {
        z[i]   = 0.0;
        z[i+3] = 0.0;
        z[i+6] = 0.0;
        for ( int j=0; j<3; j++ )
        {
            double aa = A[j][i];
            z[i]   += aa * a[j]; // A is transposed, because of that indices are flipped.
            z[i+3] += aa * w[i];
            z[i+6] += aa * B[i];
        }
    }
}

Math::Matrix<3, double> Mag::toWorldA( double * x )
{
    Math::Vector<3, double> senAng;
    senAng[0] = x[9];
    senAng[1] = x[10];
    senAng[2] = x[11];
    Math::Vector<4, double> q;
    q = a2q<double>( senAng );
    Math::Matrix<3, double> aA;
    aA = q2m<double>( q );
    return aA;
}




