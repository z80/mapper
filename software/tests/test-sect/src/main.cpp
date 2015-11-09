#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"
#include <iostream>

void calc3dPoint( std::vector<cv::Point2f> & aList,
                  std::vector<cv::Point3f> & r0List,
                  cv::Point3f & at, cv::Point3f & from );




int main()
{
    std::cout << "Built with OpenCV " << CV_VERSION << std::endl;

    std::vector<cv::Point2f> a;
    std::vector<cv::Point3f> r0;
    a.push_back( cv::Point2f( 0.0, 0.0 ) );
    r0.push_back( cv::Point3f( 0.0, 0.0, 0.0 ) );

    a.push_back( cv::Point2f( -1.0, 0.0 ) );
    r0.push_back( cv::Point3f( 1.0, 0.5, 0.0 ) );

    a.push_back( cv::Point2f( 1.0, 0.0 ) );
    r0.push_back( cv::Point3f( -1.0, -0.5, 0.0 ) );

    cv::Point3f at, from;
    calc3dPoint( a, r0, at, from );

    return 1;
}



void calc3dPoint( std::vector<cv::Point2f> & aList,
                  std::vector<cv::Point3f> & r0List,
                  cv::Point3f & at, cv::Point3f & from )
{
    int sz = static_cast<int>( aList.size() );

    cv::Mat A( 3*sz, 3, CV_64FC1 );
    cv::Mat B( 3*sz, 1, CV_64FC1 );

    for ( int i=0; i<sz; i++ )
    {
        cv::Point2f pt = aList[i];

        cv::Mat m( 4, 1, CV_64FC1 );
        double x = pt.x;
        double y = pt.y;
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
        r0[0] = r0List[i].x;
        r0[1] = r0List[i].y;
        r0[2] = r0List[i].z;

        // Remember the very first camera position to find appropriate
        // surface plane direction afterwards.
        if ( i == 0 )
        {
            from.x = r0[0];
            from.y = r0[1];
            from.z = r0[2];
        }

        // Convert to world ref. frame.
        //m = wrld*m;
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
        B.at<double>( 3*i+2, 0 ) = -a[0]*a[2]*r0[0] - a[1]*a[2]*r0[1] + (1.0 - a[2]*a[2])*r0[2];
    }

    cv::Mat Atr = A.t();
    cv::Mat AtrA = Atr * A;
    cv::Mat invAtrA = AtrA.inv();
    //cv::Mat unity = invAtrA * AtrA;
    cv::Mat AtrB = Atr * B;
    cv::Mat R = invAtrA * AtrB;
    at.x = R.at<double>( 0, 0 );
    at.y = R.at<double>( 1, 0 );
    at.z = R.at<double>( 2, 0 );
}





