
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

    imageSz        = cv::Size( 320, 240 );
    smoothSz       = 5;
    tresholdWndSz  = 101;
    nn_match_ratio = 0.8f;
}

FeatureLocator::~FeatureLocator()
{

}

bool FeatureLocator::processFrame( const cv::Mat & img, const cv::Mat & camToWorld )
{
    cv::Mat scaled;
    //cv::Mat gray;
    cv::Mat blurred;
    cv::Mat subtracted;

    rescaleImage( img, scaled );
    blurImage( scaled, blurred );
    subtractBackgroung( blurred, subtracted );

    return false;
}

void FeatureLocator::rescaleImage( const cv::Mat & orig, cv::Mat & scaled )
{
    //scaled = img.clone();
    cv::resize( orig, scaled, imageSz );
    cv::cvtColor( scaled, scaled, CV_RGB2GRAY );
}

void FeatureLocator::blurImage( const cv::Mat & orig, cv::Mat & blurred )
{
    if ( smoothSz > 0 )
    {
        // Median accepts only odd values.
        cv::medianBlur( orig, blurred, (smoothSz & 1) ? smoothSz: smoothSz+1 );
        cv::blur( blurred, blurred, cv::Size( smoothSz, smoothSz ) );
    }
    else
        blurred = orig;
}

void FeatureLocator::subtractBackgroung( const cv::Mat & orig, cv::Mat & subtracted )
{
    if ( !(tresholdWndSz & 1) )
        tresholdWndSz |= 1;
    cv::adaptiveThreshold( orig, subtracted, 255,
                           cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,
                           tresholdWndSz, 0.0 );
}

int FeatureLocator::match( const cv::Mat & img, const cv::Mat & camToWorld )
{
    detector->detect( img, keypoints );
    detector->compute( img, keypoints, descs );

    // Copy obtained features.
    FeatureDesc desc;
    desc.screenPos.resize( keypoints.size() );
    desc.feature    = descs;
    desc.camToWorld = camToWorld;
    unsigned i = 0;
    for( std::vector<cv::KeyPoint>::const_iterator it=keypoints.begin(); it!=keypoints.end(); it++ )
    {
        cv::KeyPoint kp = *it;
        desc.screenPos[ i ] = kp.pt;
        i++;
    }

    // If there are previous frames analyzed.
    if ( frames.size() < 1 )
        return 0;

    FeatureDesc & prevDesc = frames[ frames.size() - 1 ];


    //std::vector< cv::KeyPoint > matched1, matched2;
    int matched = 0;
    matcher->knnMatch( prevDesc.feature, desc.feature, matches, 2 );
    for( unsigned int i=0; i<matches.size(); i++ )
    {
        if ( matches[i][0].distance < nn_match_ratio * matches[i][1].distance )
        {
            //matched1.push_back( first_kp[matches[i][0].queryIdx] );
            //matched2.push_back(       kp[matches[i][0].trainIdx] );
            desc.screenPos[i].matchedIndex = matches[i][0].trainIdx;
            matched += 1;
        }
    }
    return matched;
}

bool FeatureLocator::triangulatePoints()
{
    // Find most remote camera positions.
    //cv::triangulatePoints( 
    return true;
}

bool FeatureLocator::calcCameraPosition()
{
    
    return true;
}






