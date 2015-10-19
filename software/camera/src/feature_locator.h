
#ifndef __FEATURE_LOCATOR_H_
#define __FEATURE_LOCATOR_H_

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"

#include <list>
#include <vector>
#include <map>


class PointDesc
{
public:
    std::vector<cv::Point2f>  screenPos;
    std::vector<cv::Mat>      camToWorld;
    cv::Point3f  worldPos;
    int          selfIndex;
    bool         triangulated;

    PointDesc()
    {
        selfIndex    = -1;
    }

    PointDesc( const PointDesc & inst )
    {
        *this = inst;
    }

    ~PointDesc()
    {
    }

    const PointDesc & operator=( const PointDesc & inst )
    {
        if ( this != &inst )
        {
            screenPos    = inst.screenPos;
            camToWorld   = inst.camToWorld;
            worldPos     = inst.worldPos;
            selfIndex    = inst.selfIndex;
            triangulated = inst.triangulated;
        }
        return *this;
    }

};

class FeatureDesc
{
public:
    FeatureDesc();
    ~FeatureDesc();
    FeatureDesc( const FeatureDesc & inst );
    const FeatureDesc & operator=( const FeatureDesc & inst );

    bool addPoint( int index, int newIndex, const cv::Point2f & screenPos, const cv::Mat & camToWorld );

public:
    // Feature history and position.
    std::vector<PointDesc> screenPos;

    cv::Mat camToWorld;
};



class FeatureLocator
{
public:
    FeatureLocator();
    ~FeatureLocator();

    void setCameraMatrix( const cv::Mat & projMatrix );

    bool processFrame( const cv::Mat & img, const cv::Mat & camToWorld = cv::Mat() );
    bool triangulatePoints();
    bool calcCameraPosition();

    void resetTracking();
private:
    void rescaleImage( const cv::Mat & orig, cv::Mat & scaled );
    void blurImage( const cv::Mat & orig, cv::Mat & blurred );
    void subtractBackgroung( const cv::Mat & orig, cv::Mat & subtracted );

    void detectFeatures( const cv::Mat & img );
    void analyzeMatches();
    void addAll();
    void analyze();
    bool triangulateOne( int index, cv::Point3f & r );

    // debug utilities.
    void drawFeatures( cv::Mat & img );

    /*
    std::vector<RawPoint>          rawPoints;

    cv::Ptr<cv::Feature2D>         detector;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    cv::Mat                                descs, 
                                           descsPrev;
    std::vector<cv::KeyPoint>              keypoints;
    std::vector< std::vector<cv::DMatch> > matches;

    std::vector<FeatureDesc> frames;
    */

    // Analyzers.
    cv::Ptr<cv::Feature2D>         detector;
    cv::Ptr<cv::DescriptorMatcher> matcher;


    // To just hold current values.
    cv::Mat                                projMatrix;
    cv::Mat                                camToWorld;
    std::vector<cv::KeyPoint>              keypoints;
    std::vector< std::vector<cv::DMatch> > matches;

    // For instant recognition.
    std::vector<cv::Point2f> points,
                             pointsPrev;

    cv::Mat                  features,
                             featuresPrev;

    // Tracked points history.
    std::map< int, std::vector<cv::Point2f> > pointFrames,
                                              pointFramesNew;
    std::map< int, cv::Point3f > worldPoints,
                                 worldPointsNew;
    // World matrix hostory.
    std::vector<cv::Mat>                worldFrames,
                                        worldFramesNew;
    // Features history.
    //std::list<cv::Mat>                featureFrames;


    // Settings.
    cv::Size imageSz;
    int      smoothSz;
    int      tresholdWndSz;
    double   nn_match_ratio; // = 0.8f; // Nearest-neighbour matching ratio
    // Triangulation conditions.
    double triangMinDist;
    double triangMinTang;
};


#endif


