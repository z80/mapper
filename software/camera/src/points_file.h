
#ifndef __POINTS_FILE_H_
#define __POINTS_FILE_H_

#include <iostream>
#include <fstream>


class PointsFile
{
public:
    PointsFile( const std::string & fname );
    ~PointsFile();

    bool write( const std::vector<cv::Point3f> & ats, const std::vector<cv::Point3f> & froms );
    bool read( const std::vector<cv::Point3f> & ats, const std::vector<cv::Point3f> & froms );
};


#endif




