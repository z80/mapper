

#include "newton_sam.h"

const double NewtonSam::ALPHA        = 0.2;
const double NewtonSam::MIN_STEP     = 1.0e-6;
const double NewtonSam::EPS          = 1.0e-6;
const int    NewtonSam::ITER_MAX     = 32;


NewtonSam::NewtonSam()
{
}

NewtonSam::~NewtonSam()
{
}

bool NewtonSam::matchPoints( std::vector<double> & pts, double d, cv::Mat & floor2Sample )
{
    auto sz = pts.size() / 6;
    this->pts.clear();
    cv::Mat   X( sz, 6, CV_64F );
    cv::Mat   Y( sz, 1, CV_64F );

    // First approach is just pseudoinverse matrix.
    int ind = 0;
    for ( auto i=0; i<sz; i++ )
    {
        double x = pts[ ind ];
        double y = pts[ ind+1 ];
        double rx = pts[ ind+2 ];
        double ry = pts[ ind+3 ];
        double nx = pts[ ind+4 ];
        double ny = pts[ ind+5 ];

        double l = sqrt( nx*nx + ny*ny );
        nx /= l;
        ny /= l;

        this->pts.push_back( x );
        this->pts.push_back( y );
        this->pts.push_back( rx );
        this->pts.push_back( ry );
        this->pts.push_back( nx );
        this->pts.push_back( ny );

        X.at<double>( i, 0 ) = x*nx;
        X.at<double>( i, 1 ) = y*nx;
        X.at<double>( i, 2 ) = nx;
        X.at<double>( i, 3 ) = x*ny;
        X.at<double>( i, 4 ) = y*ny;
        X.at<double>( i, 5 ) = ny;

        Y.at<double>( i, 0 ) = rx*nx + ry*ny + d/2.0;

        ind += 6;
    }
    cv::Mat Xt = X.t();
    cv::Mat XtX = Xt * X;
    this->XtX = XtX;
    XtX = XtX.inv();
    cv::Mat XtY = Xt * Y;
    this->XtY = XtY;
    cv::Mat S = XtX * XtY; // Initial guess.

    std::cout << "X: " << std::endl;
    for ( auto i=0; i<X.rows; i++ )
    {
        for ( auto j=0; j<X.cols; j++ )
        {
            std::cout << X.at<double>( i, j ) << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "Y: " << std::endl;
    for ( auto i=0; i<Y.rows; i++ )
    {
        for ( auto j=0; j<Y.cols; j++ )
        {
            std::cout << Y.at<double>( i, j ) << " ";
        }
        std::cout << std::endl;
    }

    



    // Construct matrices for calculating minimizing function.



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


    // Copy S to a.
    double a[9];
    for ( int i=0; i<6; i++ )
        a[i] = S.at<double>( i, 0 );
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

    floor2Sample = cv::Mat::zeros( 2, 3, CV_64F );
    ind = 0;
    for ( int i=0; i<2; i++ )
    {
        for ( int j=0; j<3; j++ )
        {
            floor2Sample.at<double>( i, j ) = a[ind++];
        }
    }
    return true;
}

double NewtonSam::fi( double * a )
{
    double f = 0.0;
    auto sz = pts.size() / 6;
    // Points part.
    for ( auto i=0; i<sz; i++ )
    {
        int ind = i*6;

        double xc = pts[ ind ];
        double yc = pts[ ind+1 ];
        double xs = a[0]*xc + a[1]*yc + a[2];
        double ys = a[3]*xc + a[4]*yc + a[5];
        double rx = pts[ ind+2 ];
        double ry = pts[ ind+3 ];
        double nx = pts[ ind+4 ];
        double ny = pts[ ind+5 ];
        double d = (xs - rx) * nx + (ys - ny) * ny;
        f += d*d;
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

void  NewtonSam::gradFi( double * a, double * dfi )
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










