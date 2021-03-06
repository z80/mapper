
#include "positioner.h"
#include <iostream>
#include <iomanip>
#include <locale>
#include <algorithm>
#include <numeric>

const bool Positioner::DEBUG = true;
const double Positioner::SEARCH_RANGE = 5.0; // This is in centimeters.
const double Positioner::ALPHA = 0.4;
const int    Positioner::MIN_NO_FLOW_FRAMES = 100;
const int    Positioner::IMAGE_MARGIN = 50;
const double Positioner::MAX_FLOW_SPEED = 1.0;
const double Positioner::FLOOR_POS_MARGIN = 0.001;
const double Positioner::FLOOR_DIR_MARGIN = 0.0001;

Square::Square()
{
}

Square::~Square()
{
}

Square::Square( const Square & inst )
{
    *this = inst;
}

const Square & Square::operator=( const Square & inst )
{
    if ( this != &inst )
    {
        imgPts       = inst.imgPts;
        prevImgPts   = inst.prevImgPts;
        floorPts     = inst.floorPts;
        prevFloorPts = inst.prevFloorPts;
        finalPts     = inst.finalPts;
    }

    return *this;
}

bool Square::operator<( const Square & inst ) const
{
    double area = cv::contourArea( imgPts );
    double instArea = cv::contourArea( inst.imgPts );
    return ( area < instArea );
}

void Square::orderPts()
{
    cv::Point2f m( 0.0, 0.0 );
    m = std::accumulate( imgPts.begin(), imgPts.end(), m );
    m /= 4.0;

    std::sort( imgPts.begin(), imgPts.end(), [&](const cv::Point2f & a1, const cv::Point2f & a2)
    {
        cv::Point2f a = a1 - m;
        cv::Point2f b = a2 - m;
        double la = sqrt( a.x * a.x + a.y * a.y );
        double ca = a.x/la;
        double sa = a.y/la;
        double lb = sqrt( b.x * b.x + b.y * b.y );
        double cb = b.x/lb;
        double sb = b.y/lb;
        return ( atan2( sa, ca ) < atan2( sb, cb ) );
    } );
}




static double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 );
static void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares );
static void drawSquares( cv::Mat& image, std::vector<std::vector<cv::Point> >& squares );
static void displayA( cv::Mat & img, cv::Mat & A );

Positioner::Positioner()
{
    appendNew = false;
    roundMode = true;
    sideSize  = 2.0;
    floorLocked = false;

    sampleAngle = sampleX = sampleY = 0.0;
    img2FloorSmooth = cv::Mat::zeros(2, 3, CV_64F );
    img2FloorSmooth.at<double>( 0, 0 ) = 1.0;
    img2FloorSmooth.at<double>( 1, 1 ) = 1.0;
    noOpticalFlowCounter = 0;

    loadSettings();
    resetImage2Floor();
    resetFloor2Sample();
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

    // End mill position.
    cv::FileStorage fsR( "./end_mill.xml", cv::FileStorage::READ );
    if (fsR.isOpened())
    {
        fsR[ "end_mill" ] >> R;
        fsR.release();
    }

    // Load current camera position.
    loadImg2Floor();


    return true;
}

void Positioner::setRoundMode( bool en, double size )
{
    roundMode = en;
    sideSize  = size;
}

void Positioner::frame( cv::Mat & img )
{
    std::vector<Square> squares;
    cv::Mat gray;
    cv::cvtColor( img, gray, CV_RGB2GRAY );
    cv::Mat undistorted;
    cv::undistort( gray, undistorted, cameraMatrix, distCoeffs );
    cv::blur( gray, gray, cv::Size( 15, 15 ) );
    imgSize = cv::Size( gray.cols, gray.rows );

    findSquares( undistorted, squares );
    prevPointPositions( gray, squares );
    applyPerspective( squares );
    matchSquaresRound( squares );
    applyCamera( squares );

    this->squares = squares;



    if ( DEBUG )
    {
        //drawSquares( img, squares );
        //displayA( img, img2Floor );
        //imshow( "squares", img );

        dbgDisplay( img.size() );
    }
}

void Positioner::appendNewShapes()
{
    appendNew = true;
}

void Positioner::resetImage2Floor()
{
    img2Floor = cv::Mat::zeros(2, 3, CV_64F);
    img2Floor.at<double>( 0, 0 ) = 1.0;
    img2Floor.at<double>( 1, 1 ) = 1.0;
}

void Positioner::resetFloor2Sample()
{
    floor2Sample = cv::Mat::zeros(2, 3, CV_64F);
    floor2Sample.at<double>( 0, 0 ) = 1.0;
    floor2Sample.at<double>( 1, 1 ) = 1.0;
}

void Positioner::startDrillPos()
{
    drillAs.clear();
}

void Positioner::appendDrillPos()
{
    drillAs.reserve( drillAs.size() + 6 );
    drillAs.push_back( img2Floor.at<double>( 0, 0 ) );
    drillAs.push_back( img2Floor.at<double>( 0, 1 ) );
    drillAs.push_back( img2Floor.at<double>( 0, 2 ) );
    drillAs.push_back( img2Floor.at<double>( 1, 0 ) );
    drillAs.push_back( img2Floor.at<double>( 1, 1 ) );
    drillAs.push_back( img2Floor.at<double>( 1, 2 ) );
}

void Positioner::endDrillPos()
{
    int sz = static_cast<int>( drillAs.size() );
    sz = sz / 6;
    cv::Mat X( 2*sz, 4, CV_64F );
    cv::Mat Y( 2*sz, 1, CV_64F );
    for ( int i=0; i<sz; i++ )
    {
        int ind = 6*i;
        X.at<double>( 2*i, 0 ) = drillAs[ ind ];
        X.at<double>( 2*i, 1 ) = drillAs[ ind+1 ];
        X.at<double>( 2*i, 2 ) = -1.0;
        X.at<double>( 2*i, 3 ) = 0.0;
        X.at<double>( 2*i+1, 0 ) = drillAs[ ind+3 ];
        X.at<double>( 2*i+1, 1 ) = drillAs[ ind+4 ];
        X.at<double>( 2*i+1, 2 ) = 0.0;
        X.at<double>( 2*i+1, 3 ) = -1.0;

        Y.at<double>( 2*i, 0 ) = -drillAs[ ind+2 ];
        Y.at<double>( 2*i+1, 0 ) = -drillAs[ ind+5 ];
    }
    cv::Mat Xt = X.t();
    cv::Mat XtX = Xt * X;
    cv::Mat invXtX = XtX.inv();
    cv::Mat XtY = Xt * Y;
    cv::Mat rR = invXtX * XtY;

    std::cout << X << std::endl;
    std::cout << Y << std::endl;
    std::cout << rR << std::endl;

    R.x = rR.at<double>( 0, 0 );
    R.y = rR.at<double>( 1, 0 );

    // Save end mill position.
    cv::FileStorage fs( "./end_mill.xml", cv::FileStorage::WRITE ); // Read the settings
    if (!fs.isOpened())
          return;

    fs << "end_mill" << R;
    fs.release();

}

void Positioner::startAxesPos()
{
    vise2As.clear();
}

void Positioner::appendAxesPos( int stepsX, int stepsY )
{
    vise2As.push_back( img2Floor.at<double>(0, 2) );
    vise2As.push_back( img2Floor.at<double>(1, 2) );
    vise2As.push_back( static_cast<double>( stepsX ) );
    vise2As.push_back( static_cast<double>( stepsY ) );
}

void Positioner::finishAxesPos()
{
    int sz = static_cast<int>( vise2As.size() );
    sz /= 4;
    if ( sz < 2 )
        return;
    cv::Mat X( sz, 3, CV_64F );
    cv::Mat Y( sz, 2, CV_64F );
    for ( int i=0; i<sz; i++ )
    {
        int ind = i*4;

        X.at<double>( i, 0 ) = vise2As[ind];
        X.at<double>( i, 1 ) = vise2As[ind+1];
        X.at<double>( i, 2 ) = 1.0;

        Y.at<double>( i, 0 ) = vise2As[ind+2];
        Y.at<double>( i, 1 ) = vise2As[ind+3];
    }
    cv::Mat Xt = X.t();
    cv::Mat XtX = Xt * X;
    XtX = XtX.inv();
    cv::Mat XtY = Xt * Y;
    floor2CrossVise = XtX * XtY;

}

void Positioner::startSamplePos()
{
    sample2As.clear();
    sampleOffEdge.clear();
}

void Positioner::appendSamplePos( cv::Point2d r, cv::Point2d n )
{
    /*
    sample2As.push_back( img2Floor.at<double>(0, 0) );
    sample2As.push_back( img2Floor.at<double>(0, 1) );
    sample2As.push_back( img2Floor.at<double>(0, 2) );
    sample2As.push_back( img2Floor.at<double>(1, 0) );
    sample2As.push_back( img2Floor.at<double>(1, 1) );
    sample2As.push_back( img2Floor.at<double>(1, 2) );
    */
    double x, y;
    drillPos( x, y, true );
    sample2As.push_back( x );
    sample2As.push_back( y );
    sample2As.push_back( r.x );
    sample2As.push_back( r.y );
    sample2As.push_back( n.x );
    sample2As.push_back( n.y );

    std::cout << std::setprecision(5) << "pt: " << x << " " << y << " r: " << r.x << " " << r.y << " n: " << n.x << " " << n.y << std::endl;
}

void Positioner::appendSampleFrontPos( cv::Point2d r, cv::Point2d n )
{

    double x, y;
    drillPos( x, y, true );
    sampleOffEdge.push_back( x );
    sampleOffEdge.push_back( y );
    sampleOffEdge.push_back( r.x );
    sampleOffEdge.push_back( r.y );
    sampleOffEdge.push_back( n.x );
    sampleOffEdge.push_back( n.y );

    std::cout << std::setprecision(5) << "n: " << x << " " << y << " r: " << r.x << " " << r.y << " n: " << n.x << " " << n.y << std::endl;
}

void Positioner::endSamplePos()
{
    // Derivation of both alignment matrix and end mill diameter.
    int sz = static_cast<int>( sample2As.size() );
    sz /= 10;
    if ( sz < 2 )
        return;
    cv::Mat X( sz, 7, CV_64F );
    cv::Mat Y( sz, 1, CV_64F );
    for ( int i=0; i<sz; i++ )
    {
        int ind = i*10;

        double rx = sample2As[ind+6];
        double ry = sample2As[ind+7];

        cv::Mat Rt1( 1, 3, CV_64F );
        Rt1.at<double>( 0, 0 ) = rx;
        Rt1.at<double>( 0, 1 ) = ry;
        Rt1.at<double>( 0, 2 ) = 1.0;

        cv::Mat At( 3, 2, CV_64F );
        At.at<double>( 0, 0 ) = sample2As[ind];
        At.at<double>( 1, 0 ) = sample2As[ind+1];
        At.at<double>( 2, 0 ) = sample2As[ind+2];
        At.at<double>( 0, 1 ) = sample2As[ind+3];
        At.at<double>( 1, 1 ) = sample2As[ind+4];
        At.at<double>( 2, 1 ) = sample2As[ind+5];

        cv::Mat nn( 3, 7, CV_64F );
        double nx = sample2As[ind+8];
        double ny = sample2As[ind+9];
        // Ensure normal length is 1.0.
        double l = sqrt( nx*nx + ny*ny );
        nx /= l;
        ny /= l;

        nn.at<double>( 0, 0 ) = nx;
        nn.at<double>( 0, 1 ) = ny;
        nn.at<double>( 0, 2 ) = 0.0;
        nn.at<double>( 0, 3 ) = 0.0;
        nn.at<double>( 0, 4 ) = 0.0;
        nn.at<double>( 0, 5 ) = 0.0;
        nn.at<double>( 0, 6 ) = -1.0;

        nn.at<double>( 0, 0 ) = 0.0;
        nn.at<double>( 0, 1 ) = 0.0;
        nn.at<double>( 0, 2 ) = nx;
        nn.at<double>( 0, 3 ) = ny;
        nn.at<double>( 0, 4 ) = 0.0;
        nn.at<double>( 0, 5 ) = 0.0;
        nn.at<double>( 0, 6 ) = -1.0;

        nn.at<double>( 0, 0 ) = 0.0;
        nn.at<double>( 0, 1 ) = 0.0;
        nn.at<double>( 0, 2 ) = 0.0;
        nn.at<double>( 0, 3 ) = 0.0;
        nn.at<double>( 0, 4 ) = nx;
        nn.at<double>( 0, 5 ) = ny;
        nn.at<double>( 0, 6 ) = -1.0;

        cv::Mat xx = Rt1 * At * nn;

        for ( int j=0; j<7; j++ )
            X.at<double>( i, j ) = xx.at<double>( 0, j );

        Y.at<double>( i, 0 ) = rx*nx + ry*ny;
    }
    cv::Mat Xt = X.t();
    cv::Mat XtX = Xt * X;
    XtX = XtX.inv();
    cv::Mat XtY = Xt * Y;
    cv::Mat f2s = XtX * XtY;

    floor2Sample = cv::Mat::zeros( 2, 3, CV_64F );
    floor2Sample.at<double>( 0, 0 ) = f2s.at<double>( 0, 0 );
    floor2Sample.at<double>( 1, 0 ) = f2s.at<double>( 1, 0 );
    floor2Sample.at<double>( 0, 1 ) = f2s.at<double>( 2, 0 );
    floor2Sample.at<double>( 1, 1 ) = f2s.at<double>( 3, 0 );
    floor2Sample.at<double>( 0, 2 ) = f2s.at<double>( 4, 0 );
    floor2Sample.at<double>( 1, 2 ) = f2s.at<double>( 5, 0 );
    r = f2s.at<double>( 6, 0 );

    calcSample2Floor();
}

void Positioner::endSamplePos( double d )
{
    if ( sample2As.size() < 3 )
        return;
    // Only alignment matrix.
    // Derivation of both alignment matrix and end mill diameter.
    newtonSam.matchPoints( sample2As, sampleOffEdge, d, floor2Sample );
    
    
    calcSample2Floor();

    cv::FileStorage fs( "./floor2sample.xml", cv::FileStorage::WRITE ); // Read the settings
    if (!fs.isOpened())
          return;

    fs << "floor2Sample" << floor2Sample;
    fs.release();

}

void Positioner::calcSample2Floor()
{
    cv::Mat a( 3, 3, CV_64F );
    for ( int i=0; i<2; i++ )
    {
        for ( int j=0; j<3; j++ )
        {
            a.at<double>( i, j ) = floor2Sample.at<double>( i, j );
        }
    }
    a.at<double>( 2, 0 ) = 0.0;
    a.at<double>( 2, 1 ) = 0.0;
    a.at<double>( 2, 2 ) = 1.0;
    a = a.inv();

    sample2Floor = cv::Mat( 2, 3, CV_64F );
    for ( int i=0; i<2; i++ )
    {
        for ( int j=0; j<3; j++ )
        {
            sample2Floor.at<double>( i, j ) = a.at<double>( i, j );
        }
    }
}

void Positioner::matchSquaresRound( std::vector<Square> & squares )
{
    if ( squares.size() < 1 )
        return;


    std::vector<cv::Point2d> knownPts;
    std::vector<cv::Point2d> foundPts;

    // Sort by area to make the very first square to have the biggest area.
    //std::sort(  squares.begin(), squares.end(), [&]( const Square & s1, const Square & s2 )
    //{
    //    return !( s1 < s2 );
    //} );

    // Sort other squares by distance from the biggest one.
    cv::Point2f m( 0.0, 0.0 );
    m = std::accumulate( squares[0].imgPts.begin(), squares[0].imgPts.end(), m );
    m /= 4.0;

    std::sort( squares.begin(), squares.end(), [&]( const Square & a1, const Square & a2 )
    {
        double x1 = (a1.imgPts[0].x + a1.imgPts[1].x + a1.imgPts[2].x + a1.imgPts[3].x)/4.0 - m.x;
        double y1 = (a1.imgPts[0].y + a1.imgPts[1].y + a1.imgPts[2].y + a1.imgPts[3].y)/4.0 - m.y;
        double x2 = (a2.imgPts[0].x + a2.imgPts[1].x + a2.imgPts[2].x + a2.imgPts[3].x)/4.0 - m.x;
        double y2 = (a2.imgPts[0].y + a2.imgPts[1].y + a2.imgPts[2].y + a2.imgPts[3].y)/4.0 - m.y;
        double r1 = sqrt( x1*x1 + y1*y1 );
        double r2 = sqrt( x2*x2 + y2*y2 );
        return ( r1 < r2 );
    } );





    int xSz = static_cast<int>( squares.size() );
    for ( auto i=0; i<xSz; i++ )
    {
        Square & square = squares[i];
        if ( !floorLocked )
        {
            floorLocked = true;
            // Points are presorted counterclockwise.

            // If rounding take the very first rectangle and declare it's position.
            knownPts.push_back( cv::Point2d( 0.0, 0.0 ) );
            knownPts.push_back( cv::Point2d( sideSize, 0.0 ) );
            knownPts.push_back( cv::Point2d( sideSize, sideSize ) );
            knownPts.push_back( cv::Point2d( 0.0, sideSize ) );

            foundPts.push_back( square.floorPts[0] );
            foundPts.push_back( square.floorPts[1] );
            foundPts.push_back( square.floorPts[2] );
            foundPts.push_back( square.floorPts[3] );
            newtonCam.matchPoints( knownPts, foundPts, img2Floor );
            img2FloorSmooth = img2Floor.clone();
        }
        else
        {
            for ( auto j=0; j<4; j++ )
            {
                cv::Point2f & pt = square.prevFloorPts[j];
                cv::Point2d ptF;
                ptF.x = pt.x * img2Floor.at<double>( 0, 0 ) + pt.y * img2Floor.at<double>( 0, 1 ) + img2Floor.at<double>( 0, 2 );
                ptF.y = pt.x * img2Floor.at<double>( 1, 0 ) + pt.y * img2Floor.at<double>( 1, 1 ) + img2Floor.at<double>( 1, 2 );

                ptF.x = floor( ptF.x / sideSize  + 0.5 ) *sideSize;
                ptF.y = floor( ptF.y / sideSize  + 0.5 ) *sideSize;

                foundPts.push_back( square.floorPts[j] );
                knownPts.push_back( ptF );
            }
        }
    }
    // Adjust position matrix based on newly discovered points.
    newtonCam.matchPoints( knownPts, foundPts, img2Floor );
    //newtonCam.removeOutlayers( knownPts, foundPts, img2Floor, 0.85 );
    // Smoothing matrix to determine end mill position.
    img2FloorSmooth = (1.0 - ALPHA)*img2FloorSmooth + ALPHA * img2Floor;
}

void Positioner::applyPerspective( std::vector<Square> & squares )
{
    double P[8];
    for ( int i=0; i<8; i++ )
        P[i] = perspective.at<double>( i, 0 );

    for( std::vector<Square>::iterator i=squares.begin();
         i!=squares.end(); i++ )
    {
        Square & rect = *i;
        std::vector<cv::Point2d> rectP;
        rect.floorPts.clear();
        rect.floorPts.reserve( 4 );
        rect.prevFloorPts.clear();
        rect.prevFloorPts.reserve( 4 );
        for ( int j=0; j<4; j++ )
        {
            cv::Point2f pi = rect.imgPts[j];
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
            rect.floorPts.push_back( pd );

            pi = rect.prevImgPts[j];
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
            rect.prevFloorPts.push_back( pd );
        }
    }
}

void Positioner::applyCamera( std::vector<Square> & squares )
{
    double P[8];
    int ind = 0;
    for ( int i=0; i<2; i++ )
    {
        for ( int j=0; j<3; j++ )
        {
            P[ind++] = img2FloorSmooth.at<double>( i, j );
        }
    }

    for( std::vector<Square>::iterator i=squares.begin();
         i!=squares.end(); i++ )
    {
        Square & rect = *i;
        std::vector<cv::Point2d> rectP;
        rect.finalPts.clear();
        rect.finalPts.reserve( 4 );
        for ( int j=0; j<4; j++ )
        {
            cv::Point2d pi = rect.floorPts[j];
            cv::Point2d pd;
            pd.x = ( pi.x * P[0] +
                     pi.y * P[1] +
                            P[2] );
            pd.y = ( pi.x * P[3] +
                     pi.y * P[4] +
                            P[5] );
            rect.finalPts.push_back( pd );
        }
    }
}



bool Positioner::saveImg2Floor()
{
    cv::FileStorage fs( "./img_2_floor.xml", cv::FileStorage::WRITE );
    if (!fs.isOpened())
          return false;

    fs << "img2Floor" << img2Floor;
    fs.release();

    return true;
}

bool Positioner::loadImg2Floor()
{
    cv::FileStorage fs( "./img_2_floor.xml", cv::FileStorage::READ );
    if (!fs.isOpened())
          return false;

    fs[ "img2Floor" ] >> img2Floor;
    fs.release();

    return true;
}



bool Positioner::prevPointPositions( cv::Mat & gray, std::vector<Square> & squares )
{
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
    cv::Size subPixWinSize(10,10), winSize(31,31);

    std::vector<uchar> status;
    std::vector<float> err;

    if(grayPrev.empty())
        gray.copyTo(grayPrev);
    auto sz = squares.size() * 4;
    if ( sz > 1 )
    {
        std::vector<cv::Point2f> ptsCur, ptsPrev;
        ptsCur.reserve( sz );
        ptsPrev.reserve( sz );
        std::for_each( squares.begin(), squares.end(), [&]( const Square & s )
        {
            ptsCur.push_back( s.imgPts[0] );
            ptsCur.push_back( s.imgPts[1] );
            ptsCur.push_back( s.imgPts[2] );
            ptsCur.push_back( s.imgPts[3] );
        } );
        cv::calcOpticalFlowPyrLK( gray, grayPrev, ptsCur, ptsPrev, status, err, winSize,
                             3, termcrit, 0, 0.001);
        int ind = 0;
        std::for_each( squares.begin(), squares.end(), [&]( Square & s )
        {
            s.prevImgPts.clear();
            s.prevImgPts.push_back( ptsPrev[ind++] );
            s.prevImgPts.push_back( ptsPrev[ind++] );
            s.prevImgPts.push_back( ptsPrev[ind++] );
            s.prevImgPts.push_back( ptsPrev[ind++] );
        } );
    }
    
    gray.copyTo( grayPrev );
    return true;
}


bool Positioner::fieldOfView( std::vector<double> & corners )
{
    corners.clear();
    double P[8];
    for ( int i=0; i<8; i++ )
        P[i] = perspective.at<double>( i, 0 );

    // Convert image corners into points on a floor.
    cv::Size imgSz = cv::Size( imgSize.width, imgSize.height );
    cv::Point2d pt[12];
    pt[0] = cv::Point2d( 0, 0 );
    pt[1] = cv::Point2d( imgSz.width/3, 0 );
    pt[2] = cv::Point2d( imgSz.width*2/3, 0 );
    pt[3] = cv::Point2d( imgSz.width, 0 );

    pt[4] = cv::Point2d( imgSz.width, imgSz.height/3 );
    pt[5] = cv::Point2d( imgSz.width, imgSz.height*2/3 );
    pt[6] = cv::Point2d( imgSz.width, imgSz.height );

    pt[7] = cv::Point2d( imgSz.width*2/3, imgSz.height );
    pt[8] = cv::Point2d( imgSz.width/3, imgSz.height );
    pt[9] = cv::Point2d( 0, imgSz.height );

    pt[10] = cv::Point2d( 0, imgSz.height*2/3 );
    pt[11] = cv::Point2d( 0, imgSz.height/3 );


    for ( int i=0; i<12; i++ )
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
            corners.push_back( ptF.x );
            corners.push_back( ptF.y );
    }    
    return true;
}

bool Positioner::drillPos( double & x, double & y, bool ignoreSample )
{
    double xf = img2FloorSmooth.at<double>(0, 0) * R.x + img2FloorSmooth.at<double>(0, 1) * R.y + img2FloorSmooth.at<double>(0, 2);
    double yf = img2FloorSmooth.at<double>(1, 0) * R.x + img2FloorSmooth.at<double>(1, 1) * R.y + img2FloorSmooth.at<double>(1, 2);
    if ( ignoreSample )
    {
        x = xf;
        y = yf;
        return true;
    }
    x = floor2Sample.at<double>(0, 0) * xf + floor2Sample.at<double>(0, 1) * yf + floor2Sample.at<double>(0, 2);
    y = floor2Sample.at<double>(1, 0) * xf + floor2Sample.at<double>(1, 1) * yf + floor2Sample.at<double>(1, 2);
    return true;
}

bool Positioner::knownFeatures( std::vector<double> & corners )
{
    corners.clear();
/*

    int sz = static_cast<int>( knownSquares.size() );
    for ( int i=0; i<sz; i++ )
    {
        for ( int j=0; j<4; j++ )
        {
            corners.push_back( knownSquares[i][j].x );
            corners.push_back( knownSquares[i][j].y );
        }
    }
    */
    return true;
}

bool Positioner::visibleFeatures( std::vector<double> & corners )
{
    corners.clear();
    
    int sz = static_cast<int>( squares.size() );
    for ( int i=0; i<sz; i++ )
    {
        Square & s = squares[i];
        for ( int j=0; j<4; j++ )
        {
            corners.push_back( s.finalPts[j].x );
            corners.push_back( s.finalPts[j].y );
        }
    }
    
    return true;
}




// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
void Positioner::findSquares( const cv::Mat & gray, std::vector<Square> & squares )
{
    squares.clear();

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

    cv::Size imgSize = cv::Size( gray.cols, gray.rows );
    // Test each contour.
    for( size_t i = 0; i<contours.size(); i++ )
    {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        cv::approxPolyDP( cv::Mat( contours[i] ), approx, cv::arcLength( cv::Mat( contours[i] ), true )*0.1, true );

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if( ( approx.size() == 4 ) &&
            ( fabs(contourArea(cv::Mat(approx))) > 400 ) &&
            ( cv::isContourConvex( cv::Mat(approx) ) ) )
        {
            // Accept suqres only within screen margin.
            bool accept = true;
            for ( int j=0; j<4; j++ )
            {
                cv::Point pt = approx.at( j );
                if ( ( pt.x < IMAGE_MARGIN ) || ( pt.x > (imgSize.width-IMAGE_MARGIN) ) || 
                     ( pt.y < IMAGE_MARGIN ) || ( pt.y > (imgSize.height-IMAGE_MARGIN) ) )
                {
                    accept = false;
                    break;
                }
            }
            if ( accept )
            {
                Square s;
                s.imgPts.reserve( 4 );
                s.imgPts.push_back( approx[0] );
                s.imgPts.push_back( approx[1] );
                s.imgPts.push_back( approx[2] );
                s.imgPts.push_back( approx[3] );
                s.orderPts();
                squares.push_back( s );
            }
        }
    }
}

void Positioner::dbgDisplay( cv::Point imgSz )
{
/*
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
*/
}



static void drawSquares( cv::Mat & image, std::vector<Square> & squares )
{
    for( std::vector<Square>::iterator i=squares.begin();
         i!=squares.end(); i++ )
    {
        std::vector<cv::Point2d> & sq = (*i).floorPts;
        if ( sq.size() < 4 )
            continue;
        cv::Point2d & pt1 = sq[0];
        cv::Point2d & pt2 = sq[1];
        cv::Point2d & pt3 = sq[2];
        cv::Point2d & pt4 = sq[3];
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


