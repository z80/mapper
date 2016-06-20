

#include "newton_cam.h"

const double NewtonCam::ALPHA    = 0.5;
const double NewtonCam::MIN_STEP = 1.0e-6;
const double NewtonCam::EPS      = 1.0e-6;


NewtonCam::NewtonCam()
{
}

NewtonCam::~NewtonCam()
{
}

bool NewtonCam::matchPoints( std::vector<cv::Point2d> & knownPts, std::vector<cv::Point2d> & foundPts )
{
    this->knownPts = knownPts;
    this->foundPts = foundPts;

    auto xSz = knownPts.size();
    // First approach is just pseudoinverse matrix.
    cv::Mat X = cv::Mat::zeros( xSz, 3, CV_64F );
    cv::Mat Y = cv::Mat::zeros( xSz, 2, CV_64F );
    for ( auto i=0; i<xSz; i++ )
    {
        const cv::Point2d & foundPt = foundPts[i];
        const cv::Point2d & knownPt = knownPts[i];
        X.at<double>( i, 0 ) = static_cast<double>( foundPt.x );
        X.at<double>( i, 1 ) = static_cast<double>( foundPt.y );
        X.at<double>( i, 2 ) = 1.0;
        Y.at<double>( i, 0 ) = knownPt.x;
        Y.at<double>( i, 1 ) = knownPt.y;
    }
    cv::Mat Xt = X.t();
    cv::Mat XtX = Xt * X;
    XtX = XtX.inv();
    cv::Mat XtY = Xt * Y;
    cv::Mat A = (XtX * XtY).t();

    // Construct matrices for calculating minimizing function.
    cv::Mat x = cv::Mat::zeros( 2*xSz, 6, CV_64F );
    cv::Mat y = cv::Mat::zeros( 2*xSz, 1, CV_64F );
    for ( auto i=0; i<xSz; i+=2 )
    {
        const cv::Point2d & foundPt = foundPts[i];
        const cv::Point2d & knownPt = knownPts[i];
        x.at<double>( i, 0 ) = static_cast<double>( foundPt.x );
        x.at<double>( i, 1 ) = static_cast<double>( foundPt.y );
        x.at<double>( i, 2 ) = 1.0;
        x.at<double>( i, 3 ) = 0.0;
        x.at<double>( i, 4 ) = 0.0;
        x.at<double>( i, 5 ) = 0.0;

        int j = i+1;
        x.at<double>( j, 0 ) = 0.0;
        x.at<double>( j, 1 ) = 0.0;
        x.at<double>( j, 2 ) = 0.0;
        x.at<double>( j, 3 ) = static_cast<double>( foundPt.x );;
        x.at<double>( j, 4 ) = static_cast<double>( foundPt.y );
        x.at<double>( j, 5 ) = 1.0;

        y.at<double>( i, 0 ) = knownPt.x;
        y.at<double>( i, 1 ) = knownPt.y;
    }

    // Initialize matrices needed for iterative calculations.
    this->XtX = x.t() * x;
    this->XtY = x.t() * y;
    // Copy A to a.
    int ind = 0;
    for ( int i=0; i<2; i++ )
    {
        for ( int j=0; j<3; j++ )
        {
            a[ ind++ ] = A.at<double>( i, j );
        }
    }
    lambda[0] = lambda[1] = lambda[2] = 0.0;

    double a[9];
    for ( int i=0; i<6; i++ )
        a[i] = this->a[i];
    for ( int i=0; i<3; i++ )
        a[i+6] = this->lambda[i];
    while ( true )
    {
        double f = fi( a );
        double g[9];
        double alpha = 1.0;
        gradFi( a, g );
        double newF;
        do {
            double newA[9];
            for ( int i=0; i<9; i++ )
                newA[i] = a[i] - alpha*f/g[i];
            newF = fi( newA );
            if ( newF < f )
            {
                // If it has become better assign newA to a and exit from step decreasing loop.
                for ( int i=0; i<9; i++ )
                    a[i] = newA[i];
                break;
            }
            alpha *= ALPHA;
        } while (alpha >= MIN_STEP);
        if ( fabs( f - newF ) < EPS )
            break;
    }
    return true;
}

double NewtonCam::fi( double * a )
{
    double f = 0.0;
    auto xSz = knownPts.size();
    // Points part.
    for ( auto i=0; i<xSz; i++ )
    {
        const cv::Point2d & fPt = foundPts[i];
        const cv::Point2d & kPt = knownPts[i];
        double x = a[0] * fPt.x + a[1] * fPt.y + a[2];
        double y = a[3] * fPt.x + a[4] * fPt.y + a[5];
        double d = (x - kPt.x) * (x - kPt.x) + (y - kPt.y) * (y - kPt.y);
        f += d;
    }
    // Lambda part.
    double d = a[6] * ( a[0] * a[0] + a[3] * a[3] - 1.0 );
    f += d;
    d = a[7] * ( a[1] * a[1] + a[4] * a[4] - 1.0 );
    f += d;
    d = a[8] * ( a[0] * a[1] + a[3] * a[4] );
    f += d;
    return f;
}

void  NewtonCam::gradFi( double * a, double * dfi )
{
    cv::Mat A( 6, 1, CV_64F, a );
    cv::Mat grad( 1, 6, CV_64F, dfi );
    grad = (XtX * A - XtY) * 2.0;
    dfi[0] += 2.0*a[6]*a[0] + a[8]*a[1];
    dfi[1] += 2.0*a[7]*a[1] + a[8]*a[0];
    dfi[3] += 2.0*a[6]*a[3] + a[8]*a[4];
    dfi[4] += 2.0*a[7]*a[4] + a[8]*a[3];

    dfi[6] = a[0]*a[0] + a[3]*a[3] - 1.0;
    dfi[7] = a[1]*a[1] + a[4]*a[4] - 1.0;
    dfi[8] = a[0]*a[1] + a[3]*a[4];
}










