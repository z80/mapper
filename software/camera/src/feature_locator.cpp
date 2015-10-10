
#include "feature_locator.h"

FeatureDesc::FeatureDesc()
{
    triangulated = false;
}

FeatureDesc::~FeatureDesc()
{

}

FeatureDesc::FeatureDesc( const FeatureDesc & inst )
{
    *this = inst;
}

const FeatureDesc & FeatureDesc::operator=( const FeatureDesc & inst )
{
    if ( this != &inst )
    {
        screenPos.clear();
        for( std::list<PointDesc>::const_iterator i=inst.screenPos.begin();
             i!=inst.screenPos.end(); i++ )
        {
            screenPos.push_back( *i );
        }
        triangulated = inst.triangulated;
        if ( triangulated )
            worldPos = inst.worldPos;
    }
    return *this;
}


















FeatureLocator::FeatureLocator()
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->setMaxFeatures( /*stats.keypoints*/ 1024 );
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create( "BruteForce-Hamming" );

    this->detector = orb;
    this->matcher  = matcher;
}

FeatureLocator::~FeatureLocator()
{

}

bool FeatureLocator::processFrame( const cv::Mat & img, const cv::Mat & camToWorld )
{
    cv::Mat scaled;
    cv::Mat gray;
    cv::Mat blurred;
    cv::cvtColor( img, gray, CV_RGB2GRAY );

    int smoothSz = 5;
    int tresholdWndSz = 101;
    if ( smoothSz > 0 )
    {
        // Median accepts only odd values.
        cv::medianBlur( gray, blurred, (smoothSz & 1) ? smoothSz: smoothSz+1 );
        cv::blur( blurred, blurred, cv::Size( smoothSz, smoothSz ) );
    }
    else
        blurred = gray;

    if ( !(tresholdWndSz & 1) )
        tresholdWndSz |= 1;
    cv::adaptiveThreshold( blurred, blurred, 255,
                           cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,
                           tresholdWndSz, 0.0 );

    return false;
}

void FeatureLocator::rescaleImage()
{
}

void FeatureLocator::blurImage()
{
}

void FeatureLocator::subtractBackgroung()
{
}




