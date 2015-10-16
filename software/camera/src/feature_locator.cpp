
#include "feature_locator.h"

FeatureDesc::FeatureDesc()
{
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
        for( std::vector<PointDesc>::const_iterator i=inst.screenPos.begin();
             i!=inst.screenPos.end(); i++ )
        {
            screenPos.push_back( *i );
        }
        camToWorld = inst.camToWorld;
    }
    return *this;
}

bool FeatureDesc::addPoint( int index, int newIndex, const cv::Point2f & screenPos, const cv::Mat & camToWorld )
{
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

    // Debugging.
    cv::Mat imgWithFeatures = img.clone();
    drawFeatures( imgWithFeatures );
    imshow( "Features", imgWithFeatures );
    imshow( "Subtracted", subtracted );
    // End of debugging.


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
    //FeatureDesc desc;
    //desc.screenPos.resize( keypoints.size() );
    //desc.feature    = descs;
    //desc.camToWorld = camToWorld;
    //unsigned i = 0;
    //for( std::vector<cv::KeyPoint>::const_iterator it=keypoints.begin(); it!=keypoints.end(); it++ )
    //{
    //    cv::KeyPoint kp = *it;
    //    desc.screenPos[ i ] = kp.pt;
    //    i++;
    //}

    FeatureDesc desc;
    desc.camToWorld = camToWorld;
    int matched = 0;
    // If there are previous frames analyzed.
    if ( frames.size() < 1 )
    {
        // Add all points to the list of potential points.
        desc.screenPos.resize( keypoints.size() );
        unsigned i = 0;
        for( std::vector<cv::KeyPoint>::const_iterator it=keypoints.begin(); it!=keypoints.end(); it++ )
        {
            cv::KeyPoint kp = *it;
            PointDesc pd;
            pd.screenPos = kp.pt;
            pd.selfIndex = i;
            desc.screenPos[ i ] = pd;
            i++;
        }
        frames.push_back( desc );
    }
    else
    {
        // Perform match in the case of existing previous frames.
        matcher->knnMatch( descsPrev, descs, matches, 2 );
        for( unsigned int i=0; i<matches.size(); i++ )
        {
            if ( matches[i][0].distance < ( nn_match_ratio * matches[i][1].distance ) )
            {
                // Add point to the list.
                PointDesc pd;
                pd.matchedIndex = matches[i][0].trainIdx;
                pd.selfIndex    = matches[i][1].queryIdx;
                pd.screenPos    = keypoints[i].pt;
                desc.screenPos.push_back( pd );
                matched += 1;
            }
        }
    }
    descsPrev = descs.clone();
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

void FeatureLocator::drawFeatures( cv::Mat & img )
{
    const int sz = 9;
    for ( std::vector<cv::KeyPoint>::const_iterator i=keypoints.begin(); i!=keypoints.end(); i++ )
    {
        cv::KeyPoint pt = *i;
        pt.pt.x = img.size().width / imageSz.width;
        pt.pt.y = img.size().height / imageSz.height;
        line( img, cv::Point( pt.pt.x-sz, pt.pt.y ), cv::Point( pt.pt.x+sz, pt.pt.y ), cv::Scalar( 200., 0., 0., 0.2 ), 2  );
        line( img, cv::Point( pt.pt.x, pt.pt.y-sz ), cv::Point( pt.pt.x, pt.pt.y+sz ), cv::Scalar( 0., 200., 0., 0.2 ), 2  );
    }
}






