
#include "positioner.h"

const bool Positioner::DEBUG = true;
static const int thresh = 50;
static const int N = 11;


static double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 );
static void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares );
static void drawSquares( cv::Mat& image, std::vector<std::vector<cv::Point> >& squares );

Positioner::Positioner()
{
}

Positioner::~Positioner()
{
}

void Positioner::frame( cv::Mat & img )
{
    std::vector<std::vector<cv::Point>> squares;
    //findSquares( img, squares );

    cv::Mat gray;
    cv::cvtColor( img, gray, CV_RGB2GRAY );

    cv::Mat tresh;
    cv::adaptiveThreshold( gray, gray, 255,
                           cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
                           201 | 1, 0.0 );

    imshow( "gray", gray );
    // find contours and store them all as a list
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

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
            /*
            double maxCosine = 0;

            for( int j = 2; j < 5; j++ )
            {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = MAX(maxCosine, cosine);
            }
            */

            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            //if( maxCosine < 0.3 )
                squares.push_back(approx);
        }
    }


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

void Positioner::startLinePos()
{
}

void Positioner::appendLinePos()
{
}

void Positioner::endLinePos()
{
}

void Positioner::prepareImage()
{

}

void Positioner::detectTriangles()
{

}

void Positioner::matchPoints()
{

}





// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares )
{
    squares.clear();

    cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    cv::pyrDown(image, pyr, cv::Size(image.cols/2, image.rows/2));
    cv::pyrUp(pyr, timg, image.size());
    std::vector<std::vector<cv::Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                cv::Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                cv::dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

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
                    /*
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }
                    */

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    //if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
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
