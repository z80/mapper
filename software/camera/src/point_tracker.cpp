
#include "point_tracker.h"


PointTracker::PointTracker()
{
    imageSz = cv::Size( 160, 120 );
}

PointTracker::~PointTracker()
{

}

void PointTracker::setCameraMatrix( const cv::Mat & projMatrix, const cv::Mat & distCoefs )
{
    this->projMatrix = projMatrix.clone();
    this->distCoefs  = distCoefs.clone();
}

void PointTracker::process( const cv::Mat & frame, const cv::Mat & worldM )
{

}

void PointTracker::finish()
{

}

void PointTracker::prepareImage( const cv::Mat & frame )
{
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    cv::resize( gray, gray, imageSz );
}

void PointTracker::calcOpticalFlow()
{
    if( !grayPrev.empty() )
        calcOpticalFlowFarneback( grayPrev, gray, uflow, 0.5, 6, 30, 3, 5, 1.2, 0);
    grayPrev = gray.clone();
}

void PointTracker::countOpticalFlow()
{
    for ( int i=0; i<imageSz.height; i++ )
    {
        for ( int j=0; j<imageSz.width; j++ )
        {
            int index = i + imageSz.width * i;
            cv::Point2f dr = uflow.at<cv::Point2f>(i, j);

            std::vector<cv::Point2f> & points = pointHistXy( j, i );
            // Determine if there was a movement.
            if ( (fabs(dr.x) < 1.0) && fabs(dr.y) < 1.0 )
            {
                // No movement. Triangulate if there are at least two points in history.
                // Otherwise it obviously doesn't make sense.
                if ( points.size() > 1 )
                    calc3dPoint( points );
                // Clear history.
                points.clear();
                // Push current position.
                cv::Point2f at = cv::Point2f( static_cast<float>( j ), static_cast<float>( i ) );
                points.push_back( at );
                // Put it to a new array
                pushPointHistXy( j, i, points );
            }
            else
            {
                // Movement exists.
                // Push current position plus displacement.
                cv::Point2f at = cv::Point2f( static_cast<float>( j ) + dr.x, static_cast<float>( i ) + dr.y );
                points.push_back( at );
                // Put it to a new array
                int col = cvRound( at.x );
                int row = cvRound( at.y );

                pushPointHistXy( row, col, points );
            }
        }
    }

    // Trim world history.
    int ptSz = longestPointHist();
    int wSz = static_cast<int>( worldHist.size() );
    worldHistNew.clear();
    for ( int i=0; i<sz; i++ )
    {
        int ind = wSz - ptSz + i;
        cv::Mat m = worldHist[ ind ].clone();
        worldHistNew.push_back( m );
    }
    worldHist = worldHistNew;
}

void PointTracker::calc3dPoint( std::vector<cv::Point2f> & points )
{

}

std::vector<cv::Point2f> & PointTracker::pointHistXy( int row, int col )
{
    int index = imageSz.width * col + row;
    std::map< int, std::vector<cv::Point2f> >::iterator at = pointHist.find( index );
    if ( at != pointHist.end() )
        return at.second;

    pointHist.insert( std::pair< int, std::vector<cv::Point2f> >( index, std::vector<cv::Point2f>() ) );
    std::vector<cv::Point2f> & points = pointHist[ index ];
    return points;
}

void PointTracker::pushPointHistXy( int row, int col, std::vector<cv::Point2f> & points )
{
    int index = imageSz.width * col + row;
    pointHistNew.insert( std::pair< int, std::vector<cv::Point2f> >( index, points ) );
}

int PointTracker::longestPointHist() const
{
    int sz = 0;
    std::map< int, std::vector<cv::Point2f> >::const_iterator at;
    for ( at=pointHist.begin(); at!=pointHist.end(); at++ )
    {
        int length = static_cast<int>( at->second.size() );
        sz = ( sz < length ) ? length : sz;
    }
    return sz;
}






