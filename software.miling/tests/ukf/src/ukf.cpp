
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

    // Generate sigma points.
    // 0-th point.
    for ( auto i=0; i<Nst; i++ )
        sigma[0][i] = x[i];
    // i-th and i+Nst-th point pairs.
    Float gamma = alpha*alpha*(static_cast<Float>(Nst) + k) - static_cast<Float>(Nst);
    Float gamma_2 = sqrt( gamma );
    for ( auto i=0; i<Nst; i++ )
    {
        for ( auto j=0; j<Nst; j++ )
            sigma[i][j] = x[i];

        for ( auto j=0; j<Nst; j++ )
            sigma[i+Nst][j] = x[i];
    }

    for ( auto i=0; i<Nst; i++ )
    {
        
    }
    // Make time step.
    pr( this->x, x );
}

template <typename Float, int Nst, int Nsen >
void Ukf<Float, Nst, Nsen>::correct( Float * z, FSens    & sens )
{
}

template <typename Float, int Nst, int Nsen >
void Ukf<Float, Nst, Nsen>::cholesky( Float * P, Float * L )
{
    // Init elements with zeros.
    for ( auto i=0; i<Nst; i++ )
    {
        for ( auto j=0; j<Nst; j++ )
        {
            L[j + Nst*i] = 0.0;
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
                acc += L[k + Nst*i] * L[k + Nst*j];
            L[j + Nst*i] = ( A[j + Nst*i] - acc ) / L[j + Nst*j];
        }

        // Diagonal element.
        acc = P[i][i];
        for ( auto k=0; k<i; k++ )
            acc -= L[k + Nst*i] * L[k + Nst*i];
        L[i][i] = sqrt( acc );
    }
}



