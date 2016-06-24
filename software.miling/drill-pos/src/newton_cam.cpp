

#include "newton_cam.h"

const double NewtonCam::ALPHA        = 0.2;
const double NewtonCam::MIN_STEP     = 1.0e-6;
const double NewtonCam::EPS          = 1.0e-6;
const int    NewtonCam::ITER_MAX     = 32;


NewtonCam::NewtonCam()
{
}

NewtonCam::~NewtonCam()
{
}

bool NewtonCam::matchPoints( std::vector<cv::Point2d> & knownPts, std::vector<cv::Point2d> & foundPts, cv::Mat & cam2Floor )
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

    /*
    cv::Mat R( 2, 2, CV_64F );
    for ( int i=0; i<2; i++ )
    {
        for ( int j=0; j<2; j++ )
        {
            R.at<double>(i, j) = A.at<double>(i, j);
        }
    }

    cv::Mat W, U, Vt;
    cv::SVD::compute( R, W, U, Vt );
    R = U * Vt;
    for ( int i=0; i<2; i++ )
    {
        for ( int j=0; j<2; j++ )
        {
            A.at<double>(i, j) = R.at<double>(i, j);
        }
    }


    std::cout << "W: " << std::endl;
    for ( auto i=0; i<W.rows; i++ )
    {
        for ( auto j=0; j<W.cols; j++ )
        {
            std::cout << W.at<double>( i, j ) << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "U: " << std::endl;
    for ( auto i=0; i<U.rows; i++ )
    {
        for ( auto j=0; j<U.cols; j++ )
        {
            std::cout << U.at<double>( i, j ) << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "Vt: " << std::endl;
    for ( auto i=0; i<Vt.rows; i++ )
    {
        for ( auto j=0; j<Vt.cols; j++ )
        {
            std::cout << Vt.at<double>( i, j ) << " ";
        }
        std::cout << std::endl;
    }
    */
    //cam2Floor = A;
    //return true;



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
        x.at<double>( j, 3 ) = static_cast<double>( foundPt.x );
        x.at<double>( j, 4 ) = static_cast<double>( foundPt.y );
        x.at<double>( j, 5 ) = 1.0;

        y.at<double>( i, 0 ) = knownPt.x;
        y.at<double>( j, 0 ) = knownPt.y;
    }

    // Initialize matrices needed for iterative calculations.
    this->XtX = x.t() * x;
    this->XtY = x.t() * y;

    /*
    std::cout << "XtX: " << std::endl;
    for ( auto i=0; i<6; i++ )
    {
        for ( auto j=0; j<6; j++ )
        {
            std::cout << this->XtX.at<double>( i, j ) << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "XtY: " << std::endl;
    for ( auto i=0; i<6; i++ )
    {
        std::cout << this->XtY.at<double>( i, 0 ) << std::endl;
    }
    */


    // Copy A to a.
    int ind = 0;
    double a[10];
    for ( int i=0; i<2; i++ )
    {
        for ( int j=0; j<3; j++ )
        {
            a[ ind++ ] = A.at<double>( i, j );
        }
    }
    for ( int i=0; i<4; i++ )
        a[i+6] = 1.0;

    while ( true )
    {
        int improvementsCnt = 0;
        for ( auto dir=0; dir<9; dir++ )
        {
            double f = fi( a );
            double g[10];
            double alpha = 1.0;
            gradFi( a, g );
            double newA[10];
            for ( int i=0; i<10; i++ )
                newA[i] = a[i];
            if ( fabs( g[dir] ) > std::numeric_limits<double>::epsilon() )
            {
                for ( int tries=0; tries<ITER_MAX; tries++ )
                {
                    newA[dir] = a[dir] - alpha*f/g[dir];
                    double newF = fi( newA );
                    if ( fabs(newF) < fabs( f ) )
                    {
                        // Indicate that situation was improved.
                        improvementsCnt++;
                        // Apply changes.
                        for ( auto i=0;i<10; i++ )
                            a[i]=newA[i];
                        f = newF;
                        // Terminate cutrrent loop.
                        break;
                    }
                    else
                        alpha *= ALPHA;
                }
            }
        }
        if ( improvementsCnt < 1 )
            break;
        improvementsCnt = 0;

    }
    cam2Floor = cv::Mat::zeros( 2, 3, CV_64F );
    ind = 0;
    for ( int i=0; i<2; i++ )
    {
        for ( int j=0; j<3; j++ )
        {
            cam2Floor.at<double>( i, j ) = a[ind++];
        }
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
    d = a[9] * ( a[0] * a[4] - a[1] * a[3] - 1.0 );
    f += d;
    return f;
}

void  NewtonCam::gradFi( double * a, double * dfi )
{
    cv::Mat A( 6, 1, CV_64F );
    cv::Mat grad( 6, 1, CV_64F );
    for ( auto i=0; i<6; i++ )
        A.at<double>( i, 0 ) = a[i];
    grad = (XtX * A - XtY) * 2.0;
    for ( auto i=0; i<6; i++ )
        dfi[i] = grad.at<double>( i, 0 );
    dfi[0] += 2.0*a[6]*a[0] + a[8]*a[1] + a[9]*a[4];
    dfi[1] += 2.0*a[7]*a[1] + a[8]*a[0] - a[9]*a[3];
    dfi[3] += 2.0*a[6]*a[3] + a[8]*a[4] - a[9]*a[1];
    dfi[4] += 2.0*a[7]*a[4] + a[8]*a[3] + a[9]*a[0];

    dfi[6] = a[0]*a[0] + a[3]*a[3] - 1.0;
    dfi[7] = a[1]*a[1] + a[4]*a[4] - 1.0;
    dfi[8] = a[0]*a[1] + a[3]*a[4];
    dfi[9] = a[0]*a[4] - a[1]*a[3] - 1.0;
}










