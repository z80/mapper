

#include "newton_cam.h"

const double NewtonCam::ALPHA        = 0.2;
const double NewtonCam::MIN_STEP     = 1.0e-6;
const double NewtonCam::EPS          = 1.0e-6;
const int    NewtonCam::ITER_MAX     = 8;


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
    double a[9];
    for ( int i=0; i<2; i++ )
    {
        for ( int j=0; j<3; j++ )
        {
            a[ ind++ ] = A.at<double>( i, j );
        }
    }
    for ( int i=0; i<3; i++ )
        a[i+6] = 1.0;

    double f = fi( a );

    double alpha = 1.0;
    bool improved = true;
    for ( int tries=0; tries<ITER_MAX; tries++ )
    {
        int improvementsCnt;

        double newA[9];

        do {
            double jac[81];
            double g[9];
            bool perCoordinate;
            if ( improved )
            {
                improved = false;

                J( a, jac );
                cv::Mat jacobian( 9, 9, CV_64F );
                ind = 0;
                for ( int i=0; i<9; i++ )
                {
                    for ( int j=0; j<9; j++ )
                    {
                        jacobian.at<double>(i, j) = jac[ind++];
                    }
                }

                double det = cv::determinant( jacobian );
                if ( fabs( det ) < 1000.0*std::numeric_limits<double>::epsilon() )
                    // If determinant is 0 matrix is already
                    // very ortogonal.
                    perCoordinate = true;
                else
                {
                    perCoordinate = false;

                    jacobian = jacobian.inv();
                    ind = 0;
                    for ( int i=0; i<9; i++ )
                    {
                        for ( int j=0; j<9; j++ )
                        {
                            jac[ind++] = jacobian.at<double>(i, j);
                        }
                    }
                }

                gradFi( a, g );
            }

            improvementsCnt = 0;
            if ( !perCoordinate )
            {
                ind = 0;
                for ( int i=0; i<9; i++ )
                {
                    newA[i] = a[i];
                    for ( int j=0; j<9; j++ )
                        newA[i] -= alpha*jac[ind++]*g[j];
                }
                double newF = fi( newA );
                if ( fabs(newF) < fabs( f ) )
                {
                    // Indicate that situation was improved.
                    improvementsCnt++;
                    improved = true;
                    // Apply changes.
                    for ( auto i=0;i<9; i++ )
                        a[i]=newA[i];
                    f = newF;
                }
            }
            else
            {
                ind = 0;
                for ( auto i=0; i<9; i++ )
                {
                    newA[i] = a[i];
                    double den = 0.0;
                    for ( int j=0; j<9; j++ )
                        den += jac[ind++]*g[j];
                    if ( fabs( den ) > ( std::numeric_limits<double>::epsilon() * 1000.0 ) )
                    {
                        newA[i] -= alpha*f/den;
                        double newF = fi( newA );
                        if ( fabs(newF) < fabs(f) )
                        {
                            improvementsCnt += 1;
                            a[i] = newA[i];
                        }
                    }
                }
            }
        } while ( improvementsCnt > 0 );
        alpha *= ALPHA;
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

bool NewtonCam::removeOutlayers( std::vector<cv::Point2d> & knownPts,
                                 std::vector<cv::Point2d> & foundPts,
                                 cv::Mat & cam2Floor,
                                 double percent )
{
    //using Pair = std::pair<cv::Point2d, cv::Point2d>;
    typedef std::pair<cv::Point2d, cv::Point2d> Pair;

    int sz = static_cast<int>( knownPts.size() );
    if ( sz < 4 ) // At least one point is supposed to be extra point.
        return false;
    sz = static_cast<int>( static_cast<double>( sz ) * percent + 0.5 );
    sz = ( sz >= 3 ) ? sz : 3;

    // Derive very first approach.
    bool res = matchPoints( knownPts, foundPts, cam2Floor );
    if ( !res )
        return false;

    // Trim data based on that first approach.
    std::vector< Pair > pairs;
    auto n = knownPts.size();
    for ( auto i=0; i<n; i++ )
        pairs.push_back( Pair( knownPts[i], foundPts[i] ) );
    std::sort( pairs.begin(), pairs.end(), [&]( const Pair & a, const Pair & b )
        {
            double x = a.first.x - a.second.x * cam2Floor.at<double>( 0, 0 ) + a.second.y * cam2Floor.at<double>( 0, 1 ) + cam2Floor.at<double>( 0, 2 );
            double y = a.first.y - a.second.x * cam2Floor.at<double>( 1, 0 ) + a.second.y * cam2Floor.at<double>( 1, 1 ) + cam2Floor.at<double>( 1, 2 );
            double la = sqrt( x*x + y*y );

            x = b.first.x - b.second.x * cam2Floor.at<double>( 0, 0 ) + b.second.y * cam2Floor.at<double>( 0, 1 ) + cam2Floor.at<double>( 0, 2 );
            y = b.first.y - b.second.x * cam2Floor.at<double>( 1, 0 ) + b.second.y * cam2Floor.at<double>( 1, 1 ) + cam2Floor.at<double>( 1, 2 );
            double lb = sqrt( x*x + y*y );

            return (la <= lb);
        }
    );

    // Generate new data arrays.
    std::vector<cv::Point2d> knPts( sz ), fnPts( sz );
    for ( auto i=0; i<sz; i++ )
    {
        knPts[i] = pairs[i].first;
        fnPts[i] = pairs[i].second;
    }
    // Generate improved transformation with only points which fit the best.
    res = matchPoints( knPts, fnPts, cam2Floor );
    if ( !res )
        return false;

    return true;
}

double NewtonCam::fi( double * a )
{
    double f = 0.0;
    double g[10];
    gradFi( a, g );
    for ( auto i=0; i<9; i++ )
    {
        double v = g[i]*g[i];
        f += v;
    }
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

    double * L = &a[6];

    dfi[0] += 2.0*L[0]*a[0] + L[2]*a[1];
    dfi[1] += 2.0*L[1]*a[1] + L[2]*a[0];
    dfi[3] += 2.0*L[0]*a[3] + L[2]*a[4];
    dfi[4] += 2.0*L[1]*a[4] - L[2]*a[3];

    dfi[6] = a[0]*a[0] + a[3]*a[3] - 1.0;
    dfi[7] = a[1]*a[1] + a[4]*a[4] - 1.0;
    dfi[8] = a[0]*a[1] + a[3]*a[4];
}

void NewtonCam::J( double * a, double * j )
{
    double * L = &a[6];
    int ind = 0;

    // Row 0;
    for ( auto i=0; i<6; i++ )
        j[ind++] = XtX.at<double>( 0, i );
    j[0] += 2.0*L[0];
    j[1] += L[2];
    j[ind++] = 2.0*a[0];
    j[ind++] = 0.0;
    j[ind++] = a[1];
    
    // Row 1;
    for ( auto i=0; i<6; i++ )
        j[ind++] = XtX.at<double>( 1, i );
    j[10] += L[2];
    j[11] += 2.0*L[1];
    j[ind++] = 0.0;
    j[ind++] = 2.0*a[1];
    j[ind++] = a[0];

    // Row 2;
    for ( auto i=0; i<6; i++ )
        j[ind++] = XtX.at<double>( 2, i );
    j[ind++] = 0.0;
    j[ind++] = 0.0;
    j[ind++] = 0.0;

    // Row 3;
    for ( auto i=0; i<6; i++ )
        j[ind++] = XtX.at<double>( 3, i );
    j[33] += 2.0*L[0];
    j[34] += -L[2];
    j[ind++] = 2.0*a[3];
    j[ind++] = 0.0;
    j[ind++] = -a[4];

    // Row 4;
    for ( auto i=0; i<6; i++ )
        j[ind++] = XtX.at<double>( 4, i );
    j[43] += -L[2];
    j[44] += 2.0*L[1];
    j[ind++] = 0.0;
    j[ind++] = 2.0*a[4];
    j[ind++] = -a[3];

    // Row 5;
    for ( auto i=0; i<6; i++ )
        j[ind++] = XtX.at<double>( 5, i );
    j[ind++] = 0.0;
    j[ind++] = 0.0;
    j[ind++] = 0.0;

    // Row 6;
    j[ind++] = 2.0*a[0];
    j[ind++] = 0.0;
    j[ind++] = 0.0;
    j[ind++] = 2.0*a[3];
    j[ind++] = 0.0;
    j[ind++] = 0.0;
    j[ind++] = 0.0;
    j[ind++] = 0.0;
    j[ind++] = 0.0;

    // Row 7;
    j[ind++] = 0.0;
    j[ind++] = 2.0*a[1];
    j[ind++] = 0.0;
    j[ind++] = 0.0;
    j[ind++] = 2.0*a[4];
    j[ind++] = 0.0;
    j[ind++] = 0.0;
    j[ind++] = 0.0;
    j[ind++] = 0.0;

    // Row 8;
    j[ind++] = a[1];
    j[ind++] = a[0];
    j[ind++] = 0.0;
    j[ind++] = -a[4];
    j[ind++] = -a[3];
    j[ind++] = 0.0;
    j[ind++] = 0.0;
    j[ind++] = 0.0;
    j[ind++] = 0.0;
}










