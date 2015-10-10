
#ifndef __FEATURE_LOCATOR_H_
#define __FEATURE_LOCATOR_H_

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"

#include <list>
#include <vector>

class PointDesc
{
public:
    cv::Point2d screenPos;
    cv::Mat     camMatrix;

    PointDesc()
    {
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
            screenPos = inst.screenPos;
            camMatrix = inst.camMatrix;
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

    bool triangulated;
    cv::Point3d worldPos;
};



class FeatureLocator
{
public:
    FeatureLocator();
    ~FeatureLocator();

    bool processFrame( const cv::Mat & img, const cv::Mat & camToWorld );


private:
    void rescaleImage();
    void blurImage();
    void subtractBackgroung();

    cv::Ptr<cv::Feature2D>         detector;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    cv::Mat                   descs;
    std::vector<cv::KeyPoint> keypoints;


};


#endif


