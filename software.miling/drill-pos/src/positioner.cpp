
#include "positioner.h"

const bool Positioner::DEBUG = true;
const double Positioner::SEARCH_RANGE = 5.0; // This is in centimeters.
const double Positioner::ALPHA = 0.2;


static double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 );
static void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares );
static void drawSquares( cv::Mat& image, std::vector<std::vector<cv::Point> >& squares );
static void displayA( cv::Mat & img, cv::Mat & A );

Positioner::Positioner()
{
    loadSettings();
    resetImage2Floor();
}

Positioner::~Positioner()
{
}

bool Positioner::loadSettings()
{
    // Locad calibrated camera parameters.
    cv::FileStorage fs( "./data/out_camera_data.xml", cv::FileStorage::READ ); // Read the settings
    if (!fs.isOpened())
    {
          std::cout << "Could not open the configuration file" << std::endl;
          return false;
    }

    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs   = cv::Mat::zeros(5, 1, CV_64F);

    fs[ "camera_matrix" ] >> cameraMatrix;
    fs[ "distortion_coefficients" ] >> distCoeffs;
    fs.release();


    cv::FileStorage fsP( "./perspective.xml", cv::FileStorage::READ ); // Read the settings
    if (!fsP.isOpened())
    {
          std::cout << "Could not open the configuration file" << std::endl;
          return false;
    }
    fsP[ "perspective" ] >> perspective;
    fsP.release();

    return true;
}

void Positioner::frame( cv::Mat & img )
{
    std::vector<std::vector<cv::Point>> squares;
    cv::Mat undistorted;
    undistort( img, undistorted, cameraMatrix, distCoeffs );
    findSquares( undistorted, squares );

    matchSquares( squares );


    if ( DEBUG )
    {
        drawSquares( img, squares );
        displayA( img, img2Floor );
        imshow( "squares", img );

        dbgDisplay( img.size() );
    }
}

void Positioner::resetImage2Floor()
{
    img2Floor = cv::Mat::zeros(2, 3, CV_64F);
    img2Floor.at<double>( 0, 0 ) = 1.0;
    img2Floor.at<double>( 1, 1 ) = 1.0;
}

void Positioner::startDrillPos()
{
}

void Positioner::appendDrillPos( cv::Point2d r, cv::Point2d n )
{
}

void Positioner::endDrillPos()
{
}

void Positioner::startAxesPos()
{

}

void Positioner::appendAxesPos( int stepsX, int stepsY )
{

}

void Positioner::finishAxesPos()
{

}

void Positioner::startLinePos()
{
}

void Positioner::appendLinePos()
{
}

void Positioner::endLinePos()
{
}

void Positioner::matchSquares( std::vector<std::vector<cv::Point>> & squares )
{
    applyPerspective( squares );
    applyCamera();

    std::vector<cv::Point2d> knownPts;
    std::vector<cv::Point2d> foundPts;
    std::vector<int>         newRects;
    std::vector<bool>        newAlready;
    // And now match all squares one by one with known ones.
    int locatedSz = static_cast<int>( locatedSquaresImg.size() );
    int knownSz   = static_cast<int>( knownSquares.size() );
    newAlready.resize( locatedSz, false );
    if ( knownSz > 0 )
    {
        for( int i=0; i<locatedSz; i++ )
        {
            bool match = false;
            for( int j=0; j<knownSz; j++ )
            {
                bool m = matchSquares( j, i, knownPts, foundPts );
                match = (match || m);
                if ( m )
                    break;
            }
            // If it doesn't match any known rects append known rects with this one.
            if ( ( !match ) && ( !newAlready[i] ) )
            {
                newRects.push_back( i );
                newAlready[i] = true;
            }
        }
    }
    else
    {
        for ( int i=0; i<locatedSz; i++ )
            newRects.push_back( i );
    }

    // if at least one square is found adjust camera position matrix.
    // X - image coordinates.
    // Y - floor coordinates.
    int xSz = static_cast<int>( knownPts.size() );
    if ( xSz > 3 )
    {
        cv::Mat X = cv::Mat::zeros( xSz, 3, CV_64F );
        cv::Mat Y = cv::Mat::zeros( xSz, 2, CV_64F );
        for ( int i=0; i<xSz; i++ )
        {
            cv::Point2d foundPt = foundPts[i];
            cv::Point2d knownPt = knownPts[i];
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

        // Limit transformation to shift and rotation only.
        /*
        double a00 = A.at<double>(0, 0);
        double a01 = A.at<double>(0, 1);
        double a10 = A.at<double>(1, 0);
        double a11 = A.at<double>(1, 1);

        double b00 = (a00 + a11)/2.0;
        double b10 = (a10 - a01)/2.0;
        double l = sqrt( b00*b00 + b10*b10 );
        b00 /= l;
        b10 /= l;
        
        A.at<double>(0, 0) = b00;
        A.at<double>(0, 1) = b10;
        A.at<double>(1, 0) = -b10;
        A.at<double>(1, 1) = b00;
        */

        img2Floor = img2Floor*(1.0-ALPHA) + A * ALPHA;
    }

    // Adjust newly discoversd rectangles.
    int newSz = static_cast<int>( newRects.size() );
    for ( int i=0; i<newSz; i++ )
    {
        int ind = newRects[i];
        std::vector<cv::Point2d> & rectImg = locatedSquaresImg[ind];
        std::vector<cv::Point2d> rectFloor;
        for ( int j=0; j<4; j++ )
        {
            cv::Point2d & pt = rectImg[j];
            cv::Point2d ptF;
            ptF.x = pt.x * img2Floor.at<double>( 0, 0 ) + pt.y * img2Floor.at<double>( 0, 1 ) + img2Floor.at<double>( 0, 2 );
            ptF.y = pt.x * img2Floor.at<double>( 1, 0 ) + pt.y * img2Floor.at<double>( 1, 1 ) + img2Floor.at<double>( 1, 2 );
            rectFloor.push_back( ptF );
        }
        knownSquares.push_back( rectFloor );
    }
}

void Positioner::applyPerspective( std::vector<std::vector<cv::Point>> & squares )
{
    locatedSquaresImg.clear();
    double P[8];
    for ( int i=0; i<8; i++ )
        P[i] = perspective.at<double>( i, 0 );

    for( std::vector<std::vector<cv::Point>>::iterator i=squares.begin();
         i!=squares.end(); i++ )
    {
        const std::vector<cv::Point> & rect = *i;
        std::vector<cv::Point2d> rectP;
        for ( int j=0; j<4; j++ )
        {
            cv::Point2d pi = rect[j];
            cv::Point2d pd;
            pd.x = ( pi.x * P[0] +
                     pi.y * P[1] +
                            P[2] ) /
                   ( pi.x * P[6] +
                     pi.y * P[7] +
                     1.0 );
            pd.y = ( pi.x * P[3] +
                     pi.y * P[4] +
                            P[5] ) /
                   ( pi.x * P[6] +
                     pi.y * P[7] +
                     1.0 );
            rectP.push_back( pd );
        }
        locatedSquaresImg.push_back( rectP );
    }
}

void Positioner::applyCamera()
{
    locatedSquaresFloor.clear();
    int sz = static_cast<int>( locatedSquaresImg.size() );
    for( int i=0; i<sz; i++ )
    {
        const std::vector<cv::Point2d> & rect = locatedSquaresImg[i];
        std::vector<cv::Point2d> rectA;
        // Transform each point to camera current RF.
        for ( int j=0; j<4; j++ )
        {
            const cv::Point2d & pt = rect[j];
            cv::Point2d ptA;
            ptA.x = pt.x * img2Floor.at<double>( 0, 0 ) +
                    pt.y * img2Floor.at<double>( 0, 1 ) +
                           img2Floor.at<double>( 0, 2 );
            ptA.y = pt.x * img2Floor.at<double>( 1, 0 ) +
                    pt.y * img2Floor.at<double>( 1, 1 ) +
                           img2Floor.at<double>( 1, 2 );
            rectA.push_back( ptA );
        }
        locatedSquaresFloor.push_back( rectA );
    }
}

bool Positioner::matchSquares( int knownInd,
                               int foundInd,
                               std::vector<cv::Point2d> & knownPts,
                               std::vector<cv::Point2d> & foundPts )
{
    int inds[4];
    double dists[4];
    const std::vector<cv::Point2d> & knownR = knownSquares[knownInd];
    const std::vector<cv::Point2d> & foundR = locatedSquaresFloor[foundInd];
    for ( int i=0; i<4; i++ )
    {
        // Default is very first point.
        inds[i] = 0;
        double dx = knownR[i].x - foundR[0].x;
        double dy = knownR[i].y - foundR[0].y;
        dists[i] = sqrt( dx*dx+dy*dy );
        for ( int j=1; j<4; j++ )
        {
            dx = knownR[i].x - foundR[j].x;
            dy = knownR[i].y - foundR[j].y;
            double d  = sqrt( dx*dx+dy*dy );
            if ( d < dists[i] )
            {
                dists[i] = d;
                inds[i]  = j;
            }
        }
    }

    // Validate 1) all inds are different and 2) dists < SEARCH_RANGE.
    bool valid = true;
    for ( int i=0; i<3; i++ )
    {
        for ( int j=i+1; j<4; j++ )
        {
            if ( inds[i] == inds[j] )
            {
                valid = false;
                break;
            }
        }
        if ( !valid )
            break;
        if ( dists[i] > SEARCH_RANGE )
        {
            valid = false;
            break;
        }
    }

    if ( valid )
    {
        // Validation passed.
        // Add points to RF transformation list.
        const std::vector<cv::Point2d> & foundR = locatedSquaresImg[foundInd];
        for ( int i=0; i<4; i++ )
        {
            knownPts.push_back( knownR[ i ] );
            foundPts.push_back( foundR[ inds[i] ] );
        }
    }
    return valid;
}




// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
void Positioner::findSquares( const cv::Mat & image, std::vector<std::vector<cv::Point> >& squares )
{
    squares.clear();

    cv::Mat gray;
    cv::cvtColor( image, gray, CV_RGB2GRAY );

    cv::Mat tresh;
    cv::adaptiveThreshold( gray, gray, 255,
                           cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
                           201 | 1, 0.0 );

    if ( DEBUG )
        imshow( "gray", gray );
    // find contours and store them all as a list
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( gray, contours, cv::RETR_LIST,
                                      cv::CHAIN_APPROX_SIMPLE );

    std::vector<cv::Point> approx;

    // test each contour
    for( size_t i = 0; i < contours.size(); i++ )
    {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if( approx.size() == 4 &&
            fabs(contourArea(cv::Mat(approx))) > 1000 &&
            cv::isContourConvex(cv::Mat(approx)) )
        {
            squares.push_back(approx);
        }
    }
}

void Positioner::dbgDisplay( cv::Point imgSz )
{
    if ( !DEBUG )
        return;
    const int sz = 512;
    const double SCALE = 20.0;
    cv::Mat img = cv::Mat( sz, sz, CV_8UC3, cv::Scalar( 0, 0, 0 ) );

    const cv::Scalar BLUE( 0, 0, 255.0, 1.0 );
    const cv::Scalar GREEN( 0, 255.0, 0, 1.0 );
    const cv::Scalar RED( 255.0, 0, 0, 1.0 );
    int knownSz = static_cast<int>(knownSquares.size());
    for ( int i=0; i<knownSz; i++ )
    {
        cv::Point2d pt0 = knownSquares[i][0] * SCALE + cv::Point2d( sz/2, sz/4 );
        cv::Point2d pt1 = knownSquares[i][1] * SCALE + cv::Point2d( sz/2, sz/4 );
        cv::Point2d pt2 = knownSquares[i][2] * SCALE + cv::Point2d( sz/2, sz/4 );
        cv::Point2d pt3 = knownSquares[i][3] * SCALE + cv::Point2d( sz/2, sz/4 );
        cv::line( img, pt0, pt1, GREEN );
        cv::line( img, pt1, pt2, GREEN );
        cv::line( img, pt2, pt3, GREEN );
        cv::line( img, pt3, pt0, GREEN );
    }
    int foundSz = static_cast<int>(locatedSquaresFloor.size());
    for ( int i=0; i<foundSz; i++ )
    {
        cv::Point2d pt0 = locatedSquaresFloor[i][0] * SCALE + cv::Point2d( sz/2, sz/4 );
        cv::Point2d pt1 = locatedSquaresFloor[i][1] * SCALE + cv::Point2d( sz/2, sz/4 );
        cv::Point2d pt2 = locatedSquaresFloor[i][2] * SCALE + cv::Point2d( sz/2, sz/4 );
        cv::Point2d pt3 = locatedSquaresFloor[i][3] * SCALE + cv::Point2d( sz/2, sz/4 );
        cv::line( img, pt0, pt1, RED );
        cv::line( img, pt1, pt2, RED );
        cv::line( img, pt2, pt3, RED );
        cv::line( img, pt3, pt0, RED );
    }


    double P[8];
    for ( int i=0; i<8; i++ )
        P[i] = perspective.at<double>( i, 0 );

    // Convert image corners into points on a floor.
    cv::Point2d pt[4];
    pt[0] = cv::Point2d( 0, 0 );
    pt[1] = cv::Point2d( imgSz.x, 0 );
    pt[2] = cv::Point2d( imgSz.x, imgSz.y );
    pt[3] = cv::Point2d( 0, imgSz.y );
    for ( int i=0; i<4; i++ )
    {
            cv::Point2d ptA;
            cv::Point2d p = pt[i];
            ptA.x = ( p.x * P[0] +
                      p.y * P[1] +
                            P[2] ) /
                   ( p.x * P[6] +
                     p.y * P[7] +
                     1.0 );
            ptA.y = ( p.x * P[3] +
                      p.y * P[4] +
                            P[5] ) /
                   ( p.x * P[6] +
                     p.y * P[7] +
                     1.0 );
            cv::Point2d ptF;
            ptF.x = ptA.x * img2Floor.at<double>( 0, 0 ) +
                    ptA.y * img2Floor.at<double>( 0, 1 ) +
                            img2Floor.at<double>( 0, 2 );
            ptF.y = ptA.x * img2Floor.at<double>( 1, 0 ) +
                    ptA.y * img2Floor.at<double>( 1, 1 ) +
                            img2Floor.at<double>( 1, 2 );
            pt[i] = ptF;
    }

    cv::Point2d pt0 = (pt[0] * SCALE) + cv::Point2d( sz/2, sz/4 );
    cv::Point2d pt1 = (pt[1] * SCALE) + cv::Point2d( sz/2, sz/4 );
    cv::Point2d pt2 = (pt[2] * SCALE) + cv::Point2d( sz/2, sz/4 );
    cv::Point2d pt3 = (pt[3] * SCALE) + cv::Point2d( sz/2, sz/4 );
    cv::line( img, pt0, pt1, BLUE );
    cv::line( img, pt1, pt2, BLUE );
    cv::line( img, pt2, pt3, BLUE );
    cv::line( img, pt3, pt0, BLUE );

    imshow( "top view", img );
}



static void drawSquares( cv::Mat& image, std::vector<std::vector<cv::Point> >& squares )
{
    for( std::vector<std::vector<cv::Point>>::iterator i=squares.begin();
         i!=squares.end(); i++ )
    {
        std::vector<cv::Point> & sq = *i;
        if ( sq.size() < 4 )
            continue;
        cv::Point & pt1 = sq[0];
        cv::Point & pt2 = sq[1];
        cv::Point & pt3 = sq[2];
        cv::Point & pt4 = sq[3];
        cv::Scalar GREEN( 0.0, 100.0, 0.0, 1.0 );
        cv::line( image, pt1, pt2, GREEN );
        cv::line( image, pt2, pt3, GREEN );
        cv::line( image, pt3, pt4, GREEN );
        cv::line( image, pt4, pt1, GREEN );
    }
}




static double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}


static void displayA( cv::Mat & img, cv::Mat & A )
{
    //std::ostringstream os;
    char stri[128];
    const cv::Scalar GREEN(0, 100, 0);
    sprintf( stri, "%5.2f, %5.2f, %5.2f", A.at<double>(0, 0), A.at<double>(0, 1), A.at<double>(0, 2) );
    cv::putText( img, stri, cv::Point( 10, 10 ), 1, 1, GREEN );
    sprintf( stri, "%5.2f, %5.2f, %5.2f", A.at<double>(1, 0), A.at<double>(1, 1), A.at<double>(1, 2) );
    cv::putText( img, stri, cv::Point( 10, 30 ), 1, 1, GREEN );
}


