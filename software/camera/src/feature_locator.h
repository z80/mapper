
#ifndef __FEATURE_LOCATOR_H_
#define __FEATURE_LOCATOR_H_

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"

#include <queue>

class FeatureDesc
{
public:
    FeatureDesc();
    ~FeatureDesc();
    FeatureDesc( const & FeatureDesc & inst );
    const FeatureDesc & operator=( const FeatureDesc & inst );

public:
    // Feature features for recognition.
    cv::Mat feature;

    // Feature history and position.
    std::queue<cv::Point2d> screenPos;

    bool triangulated;
    cv::Point3d worldPos;
};

FeatureDesc::FeatureDesc()
{
    triangulated = false;
}

FeatureDesc::~FeatureDesc()
{

}

FeatureDesc::FeatureDesc( const & FeatureDesc & inst )
{
    *this = inst;
}

const FeatureDesc & FeatureDesc::operator=( const FeatureDesc & inst )
{
    if ( this != &inst )
    {
        screenPos.clear();
        for( std::queue<cv::Point2d>::const_iterator i=inst.screenPos.begin();
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


class FeatureLocator
{
public:
    FeatureLocator();
    ~FeatureLocator();

    bool processFrame( const cv::Mat & img, const cv::Mat & camToWorld );
};


#endif


