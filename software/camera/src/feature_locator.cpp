
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
    // Looking for point with existing index.
    return true;
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

void FeatureLocator::setCameraMatrix( const cv::Mat & projMatrix )
{
    this->projMatrix = projMatrix.clone();
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


    // Processing frame and features detection.
    this->camToWorld = camToWorld.clone();
    detectFeatures( subtracted );

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

void FeatureLocator::detectFeatures( const cv::Mat & img )
{
    detector->detect( img, keypoints );
    detector->compute( img, keypoints, features );
}

void FeatureLocator::analyzeMatches()
{
    // Check if it is the very first frame.
    if ( worldFrames.size() == 0 )
        addAll();
    else
        analyze();
}

void FeatureLocator::addAll()
{
    // 1) Add features.
    featuresPrev = features;
    // 2) Add all points.
    int ind = 0;
    for ( std::vector<cv::KeyPoint>::const_iterator it=keypoints.begin(); it!=keypoints.end(); it++ )
    {
        std::vector<cv::Point2f> pts;
        cv::KeyPoint kp = *it;
        pts.push_back( kp.pt );
        pointFrames.insert( std::pair< int, std::vector<cv::Point2f> >( ind, pts ) );
    }
    // 3) add worldMatrix.
    worldFrames.push_back( camToWorld.clone() );
}

void FeatureLocator::analyze()
{
    //unsigned frameIndex = featureFrames.size();
    matcher->knnMatch( featuresPrev, features, matches, 1 );
    featuresPrev = features;

    pointFramesNew.clear();
    unsigned maxSz = 0;

    for( unsigned int i=0; i<matches.size(); i++ )
    {
        // Add point to the list.
        cv::DMatch m = matches[i][0];
        int trainInd = m.trainIdx;
        int queryInd = m.queryIdx;
        std::map< int, std::vector<cv::Point2f> >::iterator it = pointFrames.find( trainInd );
        if ( it != pointFrames.end() )
        {
            std::vector<cv::Point2f> & arr = pointFrames[ trainInd ];
            arr.push_back( keypoints[trainInd].pt );
            pointFramesNew.insert( std::pair< int, std::vector<cv::Point2f> >( queryInd, arr ) );
            maxSz = ( maxSz > arr.size() ) ? maxSz : arr.size();
        }
        else
        {
            std::vector<cv::Point2f> arr;
            arr.push_back( keypoints[trainInd].pt );
            pointFramesNew.insert( std::pair< int, std::vector<cv::Point2f> >( queryInd, arr ) );
            maxSz = ( maxSz > arr.size() ) ? maxSz : arr.size();
        }
    }
    pointFrames = pointFramesNew;

    // Crop world history length.
    int sz = static_cast<int>( worldFrames.size() );
    int maxSize = static_cast<int>( maxSz );
    int from = sz - maxSz + 1;
    worldFramesNew.clear();
    for ( int i=from; i<worldFrames.size(); i++ )
        worldFramesNew.push_back( worldFrames[i] );
    worldFramesNew.push_back( camToWorld );
    worldFrames = worldFramesNew;
}


bool FeatureLocator::triangulatePoints()
{
    int camHistSz = static_cast<int>( worldFrames.size() );
    // Find the most remote camera positions for each point and triangulate the point.
    for ( std::map< int, std::vector<cv::Point2f> >::iterator listIter = pointFrames.begin();
          listIter != pointFrames.end(); listIter++ )
    {
        double d = 0.0;
        int bestPtInd1 = -1;
        int bestPtInd2 = -1;
        int bestWorldInd1 = -1;
        int bestWorldInd2 = -1;
        std::vector<cv::Point2f> & pts = listIter->second;
        unsigned histSize = static_cast<int>( pts.size() );
        // Loop over points and find the most distant camera positions.
        int ptsCnt = static_cast<int>( pts.size() );
        for ( int ptInd1=0; ptInd1<ptsCnt; ptInd1++ )
        {
            int posInd1 = camHistSz - histSz + ptInd1;
            cv::Mat & m = worldFrames[ posInd ];
            double x1 = m.at<double>( 0, 3 );
            double y1 = m.at<double>( 1, 3 );
            double z1 = m.at<double>( 2, 3 );

            for ( int ptInd2=0; ptInd2<ptsCnt; ptInd2++ )
            {
                if ( ptInd1 == ptInd2 )
                    continue;

                int posInd2 = camHistSz - histSz + ptInd2;
                cv::Mat & m = worldFrames[ posInd2 ];
                double x2 = m.at<double>( 0, 3 );
                double y2 = m.at<double>( 1, 3 );
                double z2 = m.at<double>( 2, 3 );

                double dx = x2 - x1;
                double dy = y2 - y1;
                double dz = z2 - z1;
                double dd = dx*dx + dy*dy + dz*dz;
                if ( dd > d )
                {
                    d = dd;
                    bestPtInd1 = ptInd1;
                    bestPtInd2 = ptInd2;
                    bestWorldInd1 = posInd1;
                    bestWorldInd2 = posInd2;
                }
            }
        }
        // Triangulate point if there are appropriate camera positions.
        // if ( !camToWorld.empty() )
        {
            if ( bestPtInd1 >= 0 )
            {
                // cv::triangulatePoints( projMatrix, projMatrix, )
            }
        }
    }

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






