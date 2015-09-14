
#ifndef __CAM_LOCATOR_H_
#define __CAM_LOCATOR_H_

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"


class CamLocator
{
public:
    CamLocator( int rows = 5, int cols = 7, double step = 0.05 );
    ~CamLocator();
    bool findChessboard( const cv::Mat & mat, cv::Mat & vRot, cv::Mat & vTrans );
private:
    class PD;
    PD * pd;
};




#endif




