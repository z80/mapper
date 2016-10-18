
#ifndef __UKF_H_
#define __UKF_H_

#include <vector>
#include <algorithm>
#include <numeric>
#include <functional>
#include "matrix2.hpp"



template <typename Float, int Nst, int Nsen >
class Ukf
{
public:
    typedef std::function<void (Float * args, Float * res)> FPredict;
    typedef std::function<void (Float * x, Float * z)>      FSens;

    Ukf();
    ~Ukf();

    void init();
    void predict( Float * x, Float * xNext, FPredict pr );
    void correct( Float * z, Float * xNext, FSens sens );
    void cholesky();

    // Sigma point parameters.
    Float alpha; // Scaling parameters. 1<=alpha<=1+1e-4.
    Float beta;  // For Gaussian distribution beta = 2 is optimal.
    Float k;     // k is usually set to either 0 or 3-L.
    // Lambda = alpha^2 * (L + k) - L.

    // Process description "Rx", "Rz" process noise and sensor noise covariance matrices.
    Float Rx[Nst][Nst];
    Float Rz[Nsen][Nsen];

    Float x[Nst];     // State at previous step.
    Float P[Nst][Nst]; // State noise covariance.
    Float L[Nst][Nst]; // Square root of covariance, e.i. P = L * tr(L);
    Float sigma[2*Nst + 1][Nst]; // Sigma points.
    Float y[2*Nst + 1][Nst]; // Sigma points moved through predition function.
    Float Ym[Nst];           // Mean value.
    Float Py[Nst][Nst];      // Covariance matrix calculated from sigma points.

    // Sensor readings.
    Float zy[2*Nst+1][Nsen];
    Float Zm[Nsen];
    Float Pz[Nsen][Nsen];
    Float invPz[Nsen][Nsen];

    // Combined covariance.
    Float Pyz[Nst][Nsen];
};





template <typename Float, int Nst, int Nsen>
Ukf<Float, Nst, Nsen>::Ukf()
{
}

template <typename Float, int Nst, int Nsen>
Ukf<Float, Nst, Nsen>::~Ukf()
{
}

template <typename Float, int Nst, int Nsen>
void Ukf<Float, Nst, Nsen>::init()
{
    alpha = 1.0; // Scaling parameters. 1<=alpha<=1+1e-4.
    beta = 2.0;  // For Gaussian distribution beta = 2 is optimal.
    k = 0.0;     // k is usually set to either 0 or 3-L.


    for ( auto i=0; i<Nst; i++ )
        x[i] = 0.0;
    for ( auto i=0; i<Nst; i++ )
    {
        for ( auto j=0; j<Nst; j++ )
        {
            P[i][j] = (i==j) ? 1.0 : 0.0;
            Rx[i][j] = (i==j) ? 1.0 : 0.0;
        }
    }
    for ( auto i=0; i<Nsen; i++ )
    {
        for ( auto j=0; j<Nsen; j++ )
        {
            Rz[i][j] = (i==j) ? 1.0 : 0.0;
        }
    }
}

template <typename Float, int Nst, int Nsen >
void Ukf<Float, Nst, Nsen>::predict( Float * x, Float * xNext, FPredict pr )
{
    // Obtain Cholesky square root of covariance.
    cholesky();

    // Initialize weights.
    const Float lambda = alpha*alpha*(static_cast<Float>(Nst) + k) - static_cast<Float>(Nst);
    const Float lambda_2 = sqrt( lambda + static_cast<Float>(Nst) );
    const Float Wm0 = lambda / ( lambda + static_cast<Float>(Nst) ) + (1.0 - alpha*alpha + beta);
    const Float Wc0 = Wm0;// + (1.0 - alpha*alpha + beta);
    const Float Wmci = 0.5/( lambda + static_cast<Float>(Nst) );
    const Float Sm = 1.0/( Wm0 + static_cast<Float>(Nst*2)*Wmci );
    const Float Sc = 1.0/( Wc0 + static_cast<Float>(Nst*2)*Wmci );

    // Generate sigma points.
    // 0-th point.
    for ( auto i=0; i<Nst; i++ )
        sigma[0][i] = x[i];
    // Predict 0-th sigma point.
    pr( sigma[0], y[0] );
    // i-th and i+Nst-th point pairs.
    for ( auto i=0; i<Nst; i++ )
    {
        for ( auto j=0; j<Nst; j++ )
        {
            Float d = lambda_2 * L[j][i];
            sigma[i+1][j]     = x[j] + d;
            sigma[i+1+Nst][j] = x[j] - d;
        }
        // Predict (i+1)-th and (i+1+Nst)-th sigma points.
        pr( sigma[i+1],     y[i+1] );
        pr( sigma[i+1+Nst], y[i+1+Nst] );
    }

    // Calculate mean "Ym".
    for ( auto i=0; i<Nst; i++ )
    {
        Ym[i] = 0.0;
        const int N = 2*Nst+1;
        for ( auto j=0; j<N; j++ )
        {
            const Float W = Sm * ( (j==0) ? Wm0 : Wmci );
            Ym[i] += W*y[j][i];
        }
    }
    // Initialize covatiance Py.
    for ( auto i=0; i<Nst; i++ )
    {
        for ( auto j=0; j<Nst; j++ )
        {
            Py[i][j] = Rx[i][j];
        }
    }
    // Calculate covariance "Py" using points "y[i]" and mean value "Ym".
    const int N = 2*Nst+1;
    for ( auto k=0; k<N; k++ )
    {
        Float W = Sc * ( (k==0) ? Wc0 : Wmci );
        for ( auto i=0; i<Nst; i++ )
        {
            for ( auto j=0; j<Nst; j++ )
                Py[i][j] += W * ( y[k][i] - Ym[i] ) * ( y[k][j] - Ym[j] );
        }
    }

    if ( xNext )
    {
        for ( auto i=0; i<Nst; i++ )
            xNext[i] = Ym[i];
    }
}

template <typename Float, int Nst, int Nsen >
void Ukf<Float, Nst, Nsen>::correct( Float * z, Float * xNext, FSens    sens )
{
    // Predict sensor readings.
    const int N = 2*Nst+1;
    for ( auto i=0; i<N; i++ )
        sens( y[i], zy[i] );

    const Float lambda = alpha*alpha*(static_cast<Float>(Nst) + k) - static_cast<Float>(Nst);
    const Float lambda_2 = sqrt( lambda + static_cast<Float>(Nst) );
    const Float Wm0 = lambda / ( lambda + static_cast<Float>(Nst) );
    const Float Wc0 = Wm0 + (1.0 - alpha*alpha + beta);
    const Float Wmci = 0.5/( lambda + static_cast<Float>(Nst) );
    const Float Sm = 1.0/( Wm0 + static_cast<Float>(Nst*2)*Wmci );
    const Float Sc = 1.0/( Wc0 + static_cast<Float>(Nst*2)*Wmci );

    // Calculate mean "Zm".
    for ( auto i=0; i<Nsen; i++ )
    {
        Zm[i] = 0.0;
        for ( auto j=0; j<N; j++ )
        {
            Float W = Sm * ( (j==0) ? Wm0 : Wmci );
            Zm[i] += W*zy[j][i];
        }
    }

    // Calculate sensor noise covariance.
    // Initialize covatiance Pz.
    for ( auto i=0; i<Nsen; i++ )
    {
        for ( auto j=0; j<Nsen; j++ )
        {
            Pz[i][j] = Rz[i][j];
        }
    }
    // Calculate covariance "Pz" using points "z[i]" and mean value "Zm".
    for ( auto k=0; k<N; k++ )
    {
        Float W = Sc * ( (k==0) ? Wc0 : Wmci );
        for ( auto i=0; i<Nsen; i++ )
        {
            for ( auto j=0; j<Nsen; j++ )
            {
                Pz[i][j] += W * ( zy[k][i] - Zm[i] ) * ( zy[k][j] - Zm[j] );
            }
        }
    }
    // Calculate cross covariance Pxz.
    for ( auto i=0; i<Nst; i++ )
    {
        for ( auto j=0; j<Nsen; j++ )
        {
            Pyz[i][j] = 0.0;
        }
    }
    for ( auto k=0; k<N; k++ )
    {
        Float W = Sc * ( (k==0) ? Wc0 : Wmci );
        for ( auto i=0; i<Nst; i++ )
        {
            for ( auto j=0; j<Nsen; j++ )
            {
                Pyz[i][j] += W * ( y[k][i] - Ym[i] ) * ( zy[k][j] - Zm[j] );
            }
        }
    }

    // Calculate inversion invPzz.
    Math::Matrix<Nsen, Float> A;
    for ( auto i=0; i<Nsen; i++ )
    {
        for ( auto j=0; j<Nsen; j++ )
        {
            A[i][j] = Pz[i][j];
        }
    }
    Math::Matrix<Nsen, Float> invA = A.inv();
    for ( auto i=0; i<Nsen; i++ )
    {
        for ( auto j=0; j<Nsen; j++ )
        {
            invPz[i][j] = invA[i][j];
        }
    }

    // K = Pyz * invPz;
    Float K[Nst][Nsen];
    for ( auto i=0; i<Nst; i++ )
    {
        for ( auto j=0; j<Nsen; j++ )
        {
            K[i][j] = 0.0;
            for ( auto k=0; k<Nsen; k++ )
                K[i][j] += Pyz[i][k] * invPz[k][j];
        }
    }
    // Y = Y + K*( z - z(Y) )
    Float zzy[Nsen];
    for ( auto i=0; i<Nsen; i++ )
        zzy[i] = z[i] - Zm[i];
    for ( auto i=0; i<Nst; i++ )
    {
        this->x[i] = Ym[i];
        for ( auto j=0; j<Nsen; j++ )
            this->x[i] += K[i][j] * zzy[j];
    }

    // P = Py - K * Pz * tr(K);
    Float PzTrK[Nsen][Nst];
    for ( auto i=0; i<Nsen; i++ )
    {
        for ( auto j=0; j<Nst; j++ )
        {
            PzTrK[i][j] = 0.0;
            for ( auto k=0; k<Nsen; k++ )
                PzTrK[i][j] += Pz[i][k] * Pyz[j][k]; // Second index because it is transposed.
        }
    }
    for ( auto i=0; i<Nst; i++ )
    {
        for ( auto j=0; j<Nst; j++ )
        {
            P[i][j] = Py[i][j];
            for ( auto k=0; k<Nsen; k++ )
            {
                P[i][j] -= K[i][k] * PzTrK[k][j];
            }
        }
    }

    if ( xNext )
    {
        for ( auto i=0; i<Nst; i++ )
            xNext[i] = this->x[i];
    }
}

template <typename Float, int Nst, int Nsen >
void Ukf<Float, Nst, Nsen>::cholesky()
{
    // Init elements with zeros.
    for ( auto i=0; i<Nst; i++ )
    {
        for ( auto j=0; j<Nst; j++ )
        {
            L[i][j] = 0.0;
        }
    }

    // Elements to the left of diagonal.
    for ( auto i=0; i<Nst; i++ )
    {
        Float acc;
        for ( auto j=0; j<i; j++ )
        {
            acc = 0.0;
            for ( auto k=0; k<j; k++ )
                acc += L[i][k] * L[j][k];
            L[i][j] = ( L[i][j] - acc ) / L[j][j];
        }

        // Diagonal element.
        acc = P[i][i];
        for ( auto k=0; k<i; k++ )
            acc -= L[i][k] * L[i][k];
        L[i][i] = sqrt( acc );
    }
}



#endif



