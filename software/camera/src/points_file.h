
#ifndef __POINTS_FILE_H_
#define __POINTS_FILE_H_

#include <iostream>
#include <fstream>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"


class PointsFile
{
public:
    PointsFile( const std::string & fname );
    ~PointsFile();

    bool write( const std::vector<cv::Point3f> & ats, const std::vector<cv::Point3f> & froms );
    bool read( const std::vector<cv::Point3f> & ats, const std::vector<cv::Point3f> & froms );
};


#endif




