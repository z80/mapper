
#ifndef __POSITIONER_H_
#define __POSITIONER_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <iostream>

class Positioner
{
public:
    Positioner();
    ~Positioner();

    void frame( cv::Mat & img );
    void resetPosition();

    void startDrillPos();
    void appendDrillPos( double r[], double n[] );
    void endDrillPos();

    void startLinePos();
    void appendLinePos();
    void endLinePos();

private:
    std::vector<cv::Point2d> knownPts;
    double a[9]; // Ref. frame transformation.
    double R[2]; // Drill position.
    double d;    // Drill diameter.
};


#endif


