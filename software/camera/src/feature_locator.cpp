
#include "feature_locator.h"
#include <iostream>

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
    orb->setMaxFeatures( /*stats.keypoints*/ 5 );
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create( "BruteForce-Hamming" );

    this->detector = orb;
    this->matcher  = matcher;

    imageSz        = cv::Size( 320, 240 );
    smoothSz       = 5;
    tresholdWndSz  = 101;
    nn_match_ratio = 0.8f;

    triangMinDist = 0.2;
    triangMinTang = 0.1;
}

FeatureLocator::~FeatureLocator()
{

}

void FeatureLocator::setCameraMatrix( const cv::Mat & projMatrix, const cv::Mat & distCoefs )
{
    this->projMatrix = projMatrix.clone();
    this->distCoefs  = distCoefs.clone();
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

    // Processing frame and features detection.
    this->camToWorld = camToWorld.clone();
    detectFeatures( subtracted );
    analyzeMatches();

    // Debugging.
    cv::Mat imgWithFeatures = img.clone();
    drawFeatures( imgWithFeatures );
    drawTracks( imgWithFeatures );
    imshow( "Features", imgWithFeatures );
    imshow( "Subtracted", subtracted );
    // End of debugging.

    if ( this->camToWorld.empty() )
    {
        bool res = calcCameraPosition();
        if ( !res )
            return false;
    }
    bool res = triangulatePoints();


    return res;
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
    worldPointsNew.clear();
    unsigned maxSz = 0;

    unsigned int limitSz = ( matches.size() <= keypoints.size() ) ? matches.size() : keypoints.size();
    for( unsigned int i=0; i<limitSz; i++ )
    {
        // Add point to the list.
        cv::DMatch m = matches[i][0];
        int trainInd = m.trainIdx;
        int queryInd = m.queryIdx;
        std::map< int, std::vector<cv::Point2f> >::iterator it = pointFrames.find( trainInd );
        if ( it != pointFrames.end() )
        {
            std::vector<cv::Point2f> & arr = pointFrames[ trainInd ];
            arr.push_back( keypoints[queryInd].pt );
            pointFramesNew.insert( std::pair< int, std::vector<cv::Point2f> >( queryInd, arr ) );
            maxSz = ( maxSz > arr.size() ) ? maxSz : arr.size();
        }
        else
        {
            std::vector<cv::Point2f> arr;
            arr.push_back( keypoints[queryInd].pt );
            pointFramesNew.insert( std::pair< int, std::vector<cv::Point2f> >( queryInd, arr ) );
            maxSz = ( maxSz > arr.size() ) ? maxSz : arr.size();

        }
        std::map<int, cv::Point3f>::iterator wi = worldPoints.find( trainInd );
        if ( wi != worldPoints.end() )
            worldPointsNew.insert( std::pair<int, cv::Point3f>( queryInd, wi->second ) );
    }
    pointFrames = pointFramesNew;
    worldPoints = worldPointsNew;

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

bool FeatureLocator::triangulateOne( int index, cv::Point3f & r )
{
    double fx = projMatrix.at<double>( 0, 0 );
    double fy = projMatrix.at<double>( 1, 1 );
    double cx = projMatrix.at<double>( 0, 2 );
    double cy = projMatrix.at<double>( 1, 2 );
    const std::vector<cv::Point2f> & pts = pointFrames[ index ];

    int sz = static_cast<int>( pts.size() );
    int worldSz = static_cast<int>( worldFrames.size() );

    cv::Mat A( 3*sz, 3, CV_64FC1 );
    cv::Mat B( 3*sz, 1, CV_64FC1 );

    for ( int i=0; i<sz; i++ )
    {
        int worldIndex = worldSz - sz + i;
        const cv::Mat wrld = worldFrames[ worldIndex ].clone();
        cv::Point2f at = pts[i];

        cv::Mat m( 4, 1, CV_64FC1 );
        double x = (static_cast<double>( at.x ) - cx) / fx;
        double y = (static_cast<double>( at.y ) - cy) / fy;
        double z = 1.0;
        double l = sqrt( x*x + y*y + z*z );
        x /= l;
        y /= l;
        z /= l;
        m.at<double>( 0, 0 ) = x;
        m.at<double>( 1, 0 ) = y;
        m.at<double>( 2, 0 ) = z;
        m.at<double>( 3, 0 ) = 0.0; // Yes, 0 to ignore translation part.

        // Camera position.
        double r0[3];
        r0[0] = m.at<double>( 0, 3 );
        r0[1] = m.at<double>( 1, 3 );
        r0[2] = m.at<double>( 2, 3 );

        // Convert to world ref. frame.
        m = wrld*m;
        double a[3];
        a[0] = m.at<double>( 0, 0 );
        a[1] = m.at<double>( 1, 0 );
        a[2] = m.at<double>( 2, 0 );

        A.at<double>( 3*i, 0 ) = 1.0 - a[0]*a[0];
        A.at<double>( 3*i, 1 ) = -a[0]*a[1];
        A.at<double>( 3*i, 2 ) = -a[0]*a[2];

        A.at<double>( 3*i+1, 0 ) = -a[0]*a[1];
        A.at<double>( 3*i+1, 1 ) = 1.0 - a[1]*a[1];
        A.at<double>( 3*i+1, 2 ) = -a[1]*a[2];

        A.at<double>( 3*i+2, 0 ) = -a[0]*a[2];
        A.at<double>( 3*i+2, 1 ) = -a[1]*a[2];
        A.at<double>( 3*i+2, 2 ) = 1.0 - a[2]*a[2];

        B.at<double>( 3*i, 0 )   = (1.0 - a[0]*a[0])*r0[0] - a[0]*a[1]*r0[1] - a[0]*a[2]*r0[2];
        B.at<double>( 3*i+1, 0 ) = -a[0]*a[1]*r0[0] + (1.0 - a[1]*a[1])*r0[1] - a[1]*a[2]*r0[2];
        B.at<double>( 3*i+1, 0 ) = -a[0]*a[2]*r0[0] - a[1]*a[2]*r0[1] + (1.0 - a[2]*a[2])*r0[2];
    }
    cv::Mat invA = A.inv( cv::DECOMP_SVD );
    cv::Mat R = invA * B;
    r.x = R.at<double>( 0, 0 );
    r.y = R.at<double>( 1, 0 );
    r.z = R.at<double>( 2, 0 );

    return true;
}

bool FeatureLocator::triangulatePoints()
{
    int camHistSz = static_cast<int>( worldFrames.size() );
    // Find the most remote camera positions for each point and triangulate the point.
    for ( std::map< int, std::vector<cv::Point2f> >::iterator listIter = pointFrames.begin();
          listIter != pointFrames.end(); listIter++ )
    {
        // First check if this point is triangulated.
        // And try only if it is not.
        if ( worldPoints.find( listIter->first ) != worldPoints.end() )
            continue;


        double d = 0.0;
        int bestPtInd1 = -1;
        int bestPtInd2 = -1;
        int bestWorldInd1 = -1;
        int bestWorldInd2 = -1;
        std::vector<cv::Point2f> & pts = listIter->second;
        unsigned histSz = static_cast<int>( pts.size() );
        // Loop over points and find the most distant camera positions.
        int ptsCnt = static_cast<int>( pts.size() );
        for ( int ptInd1=0; ptInd1<ptsCnt; ptInd1++ )
        {
            int posInd1 = camHistSz - histSz + ptInd1;
            cv::Mat & m = worldFrames[ posInd1 ];
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
                // Check distance between max distant camera positions.
                cv::Mat m1 = worldFrames[ bestWorldInd1 ].clone();
                cv::Mat m2 = worldFrames[ bestWorldInd2 ].clone();
                double dx = m1.at<double>( 0, 3 ) - m2.at<double>( 0, 3 );
                double dy = m1.at<double>( 1, 3 ) - m2.at<double>( 1, 3 );
                double dz = m1.at<double>( 2, 3 ) - m2.at<double>( 2, 3 );
                double d = sqrt( dx*dx + dy*dy + dz*dz );
                if ( d < triangMinDist )
                    continue;

                // Call triangulation.
                // cv::triangulatePoints( projMatrix, projMatrix, )
                int index = listIter->first;
                cv::Point3f r;
                bool res = triangulateOne( index, r );
                if ( res )
                {
                    // Remember triangulation only if tangent is not less then
                    // minimal boundary value for it.

                }
            }
        }
    }

    return true;
}

bool FeatureLocator::calcCameraPosition()
{
        // locate camera;
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

        bool useExtrinsicGuess = false;
        if ( camToWorld.empty() )
        {
            if ( worldFrames.size() > 0 )
            {
                camToWorld = worldFrames[ worldFrames.size() - 1 ].clone();
            }
        }
        if ( !camToWorld.empty() )
        {
            cv::Mat rot = camToWorld( cv::Rect( 0, 0, 3, 3 ) );
            cv::Rodrigues( rot, rvec );
            tvec = camToWorld( cv::Rect( 3, 0, 1, 3 ) ); // (!!!) should checke if dimentions are in their places.
            useExtrinsicGuess = true;
        }
        else
            tvec.at<double>( 2, 0 ) = 1.0;

        corners2d.clear();
        corners3d.clear();
        for ( std::map< int, cv::Point3f >::const_iterator it=worldPoints.begin(); it!=worldPoints.end(); it++ )
        {
            cv::Point3f pt3 = it->second;
            corners3d.push_back( pt3 );
            int index = it->first;
            const std::vector<cv::Point2f> & pts = worldFrames[ index ];
            cv::Point2f pt2 = pts[ pts.size()-1 ];
            corners2d.push_back( pt2 );
        }

        // Pose estimation
        bool correspondence;
        try {
            correspondence = cv::solvePnPRansac( cv::Mat(corners3d), cv::Mat(corners2d), projMatrix, distCoefs,
                                            rvec, tvec,
                                            useExtrinsicGuess, CV_ITERATIVE );
        }
        catch ( cv::Exception & e )
        {
            correspondence = false;
            std::cout << e.what() << std::endl;
        }


        if ( !correspondence )
            return false;
        // Transforms Rotation Vector to Matrix

        cv::Mat rot;
        cv::Rodrigues( rvec, rot );
        cv::Mat objToCam = cv::Mat::zeros( 4, 4, CV_64FC1);
        for ( int iy=0; iy<3; iy++ )
        {
            for ( int ix=0; ix<3; ix++ )
            {
                objToCam.at<double>( iy, ix ) = rot.at<double>( iy, ix );
            }
            objToCam.at<double>( iy, 3 ) = tvec.at<double>( iy, 0 );
        }
        objToCam.at<double>( 3, 3 ) = 1.0;
        camToWorld = objToCam.inv();    
    return true;
}

void FeatureLocator::drawFeatures( cv::Mat & img )
{
    const int SZ = 9;
    // Draw current features.
    for ( std::vector<cv::KeyPoint>::const_iterator i=keypoints.begin(); i!=keypoints.end(); i++ )
    {
        cv::KeyPoint kpt = *i;
        cv::Point2f pt = kpt.pt;
        pt.x *= img.size().width / imageSz.width;
        pt.y *= img.size().height / imageSz.height;
        cv::Point pt1 = cv::Point( pt.x-SZ, pt.y-SZ );
        cv::Point pt2 = cv::Point( pt.x+SZ, pt.y+SZ );
        cv::Point pt3 = cv::Point( pt.x-SZ, pt.y+SZ );
        cv::Point pt4 = cv::Point( pt.x+SZ, pt.y-SZ );
        line( img, pt1, pt2, cv::Scalar( 200., 0., 0., 0.2 ), 2  );
        line( img, pt3, pt4, cv::Scalar( 0., 200., 0., 0.2 ), 2  );
    }
}

void FeatureLocator::drawTracks( cv::Mat & img )
{
    for ( std::map< int, std::vector<cv::Point2f> >::const_iterator it = pointFrames.begin(); it != pointFrames.end(); it++ )
    {
        const std::vector<cv::Point2f> & pts = it->second;
        const int cnt = static_cast<int>( pts.size() ) - 1;
        for ( int i=0; i<cnt; i++ )
        {
            cv::Point pt1 = cv::Point( pts[i].x, pts[i].y );
            cv::Point pt2 = cv::Point( pts[i+1].x, pts[i+1].y );
            line( img, pt1, pt2, cv::Scalar( 200., 250., 0., 0.2 ), 2  );
        }

        const int SZ = 9;
        cv::Point pt1 = cv::Point( pts[cnt].x-SZ, pts[cnt].y-SZ );
        cv::Point pt2 = cv::Point( pts[cnt].x+SZ, pts[cnt].y+SZ );
        cv::Point pt3 = cv::Point( pts[cnt].x-SZ, pts[cnt].y+SZ );
        cv::Point pt4 = cv::Point( pts[cnt].x+SZ, pts[cnt].y-SZ );
        line( img, pt1, pt2, cv::Scalar( 0., 250., 200., 0.2 ), 2  );
        line( img, pt3, pt4, cv::Scalar( 0., 250., 200., 0.2 ), 2  );
    }
}






