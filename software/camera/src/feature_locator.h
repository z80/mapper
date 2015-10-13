
#ifndef __FEATURE_LOCATOR_H_
#define __FEATURE_LOCATOR_H_

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"

#include <list>
#include <vector>

class PointDesc
{
public:
    cv::Point2d  screenPos;
    cv::Point3d  worldPos;
    int          matchedIndex;
    bool         triangulated;

    PointDesc()
    {
        matchedIndex = -1;
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
            matchedIndex = inst.matchedIndex;
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

public:
    // Feature features for recognition.
    cv::Mat feature;

    // Feature history and position.
    std::list<PointDesc> screenPos;

    cv::Mat camToWorld;
    cv::Mat camMatrix;
};



class FeatureLocator
{
public:
    FeatureLocator();
    ~FeatureLocator();

    bool processFrame( const cv::Mat & img, const cv::Mat & camToWorld );
    bool triangulatePoints();
    bool calcCameraPosition();

private:
    void rescaleImage( const cv::Mat & orig, cv::Mat & scaled );
    void blurImage( const cv::Mat & orig, cv::Mat & blurred );
    void subtractBackgroung( const cv::Mat & orig, cv::Mat & subtracted );
    int  match( const cv::Mat & img, const cv::Mat & camToWorld );

    cv::Ptr<cv::Feature2D>         detector;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    cv::Mat                   descs;
    std::vector<cv::KeyPoint> keypoints;
    std::vector< std::vector<cv::DMatch> > matches;

    std::vector<FeatureDesc> frames;

    // Settings.
    cv::Size imageSz;
    int      smoothSz;
    int      tresholdWndSz;
    double   nn_match_ratio; // = 0.8f; // Nearest-neighbour matching ratio
};


#endif


