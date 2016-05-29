
#include "positioner.h"

const bool Positioner::DEBUG = true;
const double SEARCH_RANGE    = 1.5; // This is in centimeters.


static double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 );
static void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares );
static void drawSquares( cv::Mat& image, std::vector<std::vector<cv::Point> >& squares );

Positioner::Positioner()
{
}

Positioner::~Positioner()
{
}

void Positioner::loadSettings()
{
    // Locad calibrated camera parameters.
    FileStorage fs( "./data/out_camera_data.xml", FileStorage::READ ); // Read the settings
    if (!fs.isOpened())
    {
          cout << "Could not open the configuration file" << endl;
          return -1;
    }

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    distCoeffs   = Mat::zeros(5, 1, CV_64F);

    fs[ "camera_matrix" ] >> cameraMatrix;
    fs[ "distortion_coefficients" ] >> distCoeffs;
    fs.release();


    FileStorage fsP( "./perspective.xml", FileStorage::READ ); // Read the settings
    if (!fsP.isOpened())
    {
          cout << "Could not open the configuration file" << endl;
          return -1;
    }
    fsP[ "perspective" ] >> perspective;
    fsP.release();
}

void Positioner::frame( cv::Mat & img )
{
    std::vector<std::vector<cv::Point>> squares;
    //findSquares( img, squares );




    if ( DEBUG )
    {
        drawSquares( img, squares );
        imshow( "squares", img );
    }
}

void Positioner::resetPosition()
{
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
    std::vector<cv::Point>   foundPts;
    std::vector<int>         newRects;
    // And now match all squares one by one with known ones.
    int locatedSz = static_cast<int>( locatedSquares.size() );
    int knownSz   = static_cast<int>( knownSquares.size() );
    for( int i=0; i<locatedSz; i++ )
    {
        for( j=0; j<knownSz; j++ )
        {
            matchSquares( j, i,
                          knownPts,
                          foundPts,
                          newRects );
        }
    }

    // if at least one square is found adjust camera position matrix.
    // X - image coordinates.
    // Y - floor coordinates.
    int xSz = static_cast<int>( knownPts.size() );
    if ( xSz > 3 )
    {
        cv::Mat X = zeros( xSz, 3, CV_64F );
        cv::Mat Y = zeros( xSz, 2, CV_64F );
        for ( int i=0; i<xSz; i++ )
        {
            X.at<double>( i, 0 ) = static_cast<double>( foundPts[i].x );
            X.at<double>( i, 1 ) = static_cast<double>( foundPts[i].y );
            X.at<double>( i, 2 ) = 1.0;
            Y.at<double>( i, 0 ) = knownPts[i].x;
            Y.at<double>( i, 1 ) = knownPts[i].y;
        }
        cv::Mat Xt = X.t();
        cv::Mat XtX = Xt * X;
        XtX = XtX.inv();
        cv::Mat XtY = Xt * Y;
        img2Floor = XtX * XtY;
    }
    else
    {
        img2Floor = Mat::zeros(2, 3, CV_64F);
        img2Floor.at<double>( 0, 0 ) = 1.0;
        img2Floor.at<double>( 1, 1 ) = 1.0;
    }

    // Adjust newly discoversd rectangles.
    for ( int i=0; i< )
}

void Positioner::applyPerspective( std::vector<std::vector<cv::Point>> & squares )
{
    locatedSquaresImg.clear();
    double P[8];
    for ( inti=0; i<8; i++ )
        P[i] = perspective.at<double>( i, 0 );

    for( std::vector<std::vector<cv::Point>>::iterator i=squares.begin();
         i!=squares.end(); i++ )
    {
        const std::vector<cv::Point> & rect = *i;
        std::vector<cv::Point2d> rectP;
        for ( i=0; i<4; i++ )
        {
            cv::Point2d pi = rect[i];
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
        locatedSquaresImg.push_back( pd );
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
            ptA.x = pt.x * A.at<double>( 0, 0 ) +
                    pt.y * A.at<double>( 0, 1 ) +
                           A.at<double>( 0, 2 );
            ptA.y = pt.x * A.at<double>( 1, 0 ) +
                    pt.y * A.at<double>( 1, 1 ) +
                           A.at<double>( 1, 2 );
            rectA.push_back( ptA );
        }
        locatedSquaresFloor.push_back( rectA );
    }
}

void Positioner::matchSquares( int knownInd,
                               int foundInd,
                               std::vector<cv::Point> & knownPts,
                               std::vector<cv::Point> & foundPts,
                               std::vector<int> & newRects )
{
    int inds[4];
    double dists[4];
    const std::vector<cv::Point2d> & knownR = knownSquares[knownInd];
    const std::vector<cv::Point2d> & foundR = locatedSquaresFloor[knownInd];
    for ( i=0; i<4; i++ )
    {
        // Default is very first point.
        inds[i] = 0;
        double dx = knownR[i].x - foundR[0].x;
        double dy = knownR[i].y - foundR[0].y;
        dists[i] = sqrt( dx*dx+dy*dy );
        for ( j=1; j<4; j++ )
        {
            double dx = knownR[i].x - foundR[j].x;
            double dy = knownR[i].y - foundR[j].y;
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
        for ( int i=0; i<4; i++ )
        {
            knownPts.push_back( knownR[ i ] );
            foundPts.push_back( foundR[ inds[i] ] );
        }
    }
    else
    {
        // Add rectangle to known rectangles list
        // after RF correction.
        newRects.push_back( foundInd );
    }
}




// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares )
{
    squares.clear();

    cv::Mat gray;
    cv::cvtColor( img, gray, CV_RGB2GRAY );

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
