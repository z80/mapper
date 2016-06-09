
#include "descriptor.h"

Descriptor::Descriptor( double eps )
{
    this->eps = eps;
}

Descriptor::Descriptor( const Descriptor & inst )
{
    this->eps   = inst.eps;
    this->dists = inst.dists;
}

Descriptor::~Descriptor()
{
}

Descriptor & Descriptor::operator=( const Descriptor & inst )
{
    if ( this != &inst )
        *this = inst;
    return *this;
}

bool Descriptor::operator==( const Descriptor & inst )
{

    return true;
}

void Descriptor::build( std::vector<std::vector<cv::Point2d>> & rects, int rectInd, int ptInd, double range )
{
    cv::Point2d pt = rects[rectInd][ptInd];
    int sz = static_cast<int>( rects.size() );
    for ( int i=0; i<sz; i++ )
    {
        for ( int j=0; j<4; j++ )
        {
            if ( ( i != rectInd ) || ( j != ptInd ) )
            {
                cv::Point2d to = rects[i][j];
                double x = to.x - pt.x;
                double y = to.y - pt.y;
                double l = sqrt( x*x + y*y );
                if ( l <= range )
                {
                    dists.push_back( l );
                }
            }
        }
    }
}





