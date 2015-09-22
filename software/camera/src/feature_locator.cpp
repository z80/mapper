
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

}

FeatureLocator::~FeatureLocator()
{

}

bool FeatureLocator::processFrame( const cv::Mat & img, const cv::Mat & camToWorld )
{

    return false;
}



