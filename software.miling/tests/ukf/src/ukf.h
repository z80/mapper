
#ifndef __UKF_H_
#define __UKF_H_

#include <vector>
#include <algorithm>
#include <numeric>
#include <functional>

template <typename Float, int Nst, int Nsen >
class Ukf
{
public:
    typedef std::function<void (Float * args, Float * res)> FPredict;
    typedef std::function<void (Float * x, Float * z)>      FSens;

    Ukf();
    ~Ukf();

    void predict( Float * x, FPredict & pr );
    void correct( Float * z, FSens    & sens );
    void cholesky( Float * P, Float * L );

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
    Float Wm0; // Sigma point weights.
    Float Wc0;
    Float Wmci;
    Float y[2*Nst + 1][Nst]; // Sigma points moved through predition function.
    Float Ym[Nst];           // Mean value.
    Float Py[Nst][Nst];       // Covariance matrix calculated from sigma points.

    // Sensor readings.
    Float z[2*Nst+1][Nsen];
    Float Zm[Nsen];
    Float Pz[Nsen][Nsen];
    Float invPz[Nsen][Nsen];

    // Combined covariance.
    Float Pyz[Nst][Nsen];
};


#endif



