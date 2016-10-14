
#include "ukf.h"

template <typename Float, int Nst, int Nsen >
Ukf<Float, Nst, Nsen>::Ukf()
{
}

template <typename Float, int Nst, int Nsen >
Ukf<Float, Nst, Nsen>::~Ukf()
{
}

template <typename Float, int Nst, int Nsen >
void Ukf<Float, Nst, Nsen>::predict( Float * x, FPredict & pr )
{
    // Obtain Cholesky square root of covariance.
    cholesky( P, L );

    // Initialize weights.
    Float lambda = alpha*alpha*(static_cast<Float>(Nst) + k) - static_cast<Float>(Nst);
    Float lambda_2 = sqrt( lambda + static_cast<Float>(Nst) );
    Wm0 = lambda / ( lambda + static_cast<Float>(Nst) );
    Wc0 = Wm[0] + (1.0 - alpha*alpha + beta);
    Wmci = 0.5/( lambda + static_cast<Float>(Nst) );

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
            sigma[i+1][j]     = x[j] + lambda_2 * L[Nst*j + i];
            sigma[i+1+Nst][j] = x[j] - lambda_2 * L[Nst*j + i];
        }
        // Predict (i+1)-th and (i+1+Nst)-th sigma points.
        pr( sigma[i+1],     y[i+1] );
        pr( sigma[i+1+Nst], y[i+1+Nst] );
    }

    // Calculate mean "Ym".
    for ( auto i=0; i<Nst; i++ )
    {
        Float W = (i==0) ? Wm0 : Wmci;
        Ym[i] = 0.0;
        const int N = 2*Nst+1;
        for ( auto j=0; j<N; j++ )
            Ym[i] += W*y[j][i];
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
        Float W = (k==0) ? Wc0 : Wmci;
        for ( auto i=0; i<Nst; i++ )
        {
            for ( auto j=0; j<Nst; j++ )
            {
                Py[i][j] += W * ( y[k][i] - Ym[i] ) * ( y[k][j] - Ym[j] );
            }
        }
    }
}

template <typename Float, int Nst, int Nsen >
void Ukf<Float, Nst, Nsen>::correct( Float * z, FSens    & sens )
{
    // Predict sensor readings.
    const int N = 2*Nst+1;
    for ( auto i=0; i<N; i++ )
        pr( y[i], z[i] );

    // Calculate mean "Zm".
    for ( auto i=0; i<Nsen; i++ )
    {
        Float W = (i==0) ? Wm0 : Wmci;
        Zm[i] = 0.0;
        for ( auto j=0; j<N; j++ )
            Zm[i] += W*z[j][i];
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
        Float W = (k==0) ? Wc0 : Wmci;
        for ( auto i=0; i<Nsen; i++ )
        {
            for ( auto j=0; j<Nsen; j++ )
            {
                Pz[i][j] += W * ( z[k][i] - Zm[i] ) * ( z[k][j] - Zm[j] );
            }
        }
    }
    // Calculate cross covariance Pxz.
    for ( auto i=0; i<Nst; i++ )
    {
        for ( auto j=0; j<Nsen; j++ )
        {
            Pxz[i][j] = 0.0;
        }
    }
    for ( auto k=0; k<N; k++ )
    {
        Float W = (k==0) ? Wc0 : Wmci;
        for ( auto i=0; i<Nst; i++ )
        {
            for ( auto j=0; j<Nsen; j++ )
            {
                Pxz[i][j] += W * ( y[k][i] - Ym[i] ) * ( z[k][j] - Zm[j] );
            }
        }
    }
}

template <typename Float, int Nst, int Nsen >
void Ukf<Float, Nst, Nsen>::cholesky( Float * P, Float * L )
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
            L[i][j] = ( A[i][j] - acc ) / L[j][j];
        }

        // Diagonal element.
        acc = P[i][i];
        for ( auto k=0; k<i; k++ )
            acc -= L[k + Nst*i] * L[k + Nst*i];
        L[i][i] = sqrt( acc );
    }
}



