
#ifndef __TEXT_DRAWER_H_
#define __TEXT_DRAWER_H_

#include "opencv2/core.hpp"
//#include "opencv2/imgproc.hpp"
#include <iostream>


class Drawer
{
public:
    static void drawText( cv::Mat & image, const std::string & stri, int line )
    {
        cv::putText( image, stri,
                 cv::Point(20, (50 + line * 50)),
                 cv::FONT_HERSHEY_COMPLEX, 1, // font face and scale
                 cv::Scalar(255, 255, 255), // white
                 1, cv::LINE_AA); // line thickness and type
    }

};


#endif




