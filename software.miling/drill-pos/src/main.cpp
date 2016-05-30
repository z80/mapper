/*
 * matching_test.cpp
 *
 *  Created on: Oct 17, 2010
 *      Author: ethan
 */
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <iostream>
#include "positioner.h"

using namespace cv;
using namespace std;

void thresh_callback(int, void* );

int thresh = 184;
int max_thresh = 255;
RNG rng(12345);

Mat img, gray;
Mat blurred;
Mat toProcess;
int thresholdValue = 192;
int cutoffTo   = 255;

int blurValue = 10;
int eps       = 5;
int threshholdWindowSz = 3;

const double EDGE_SIZE = 10.0;
cv::Mat      perspectiveCumulative;
int          perspectiveQty;

int main(int argc, const char ** argv)
{
    VideoCapture inputCapture;
    inputCapture.open( 0 );
    if ( !inputCapture.isOpened() )
    {
        cout << "Failed to open camera!";
        return 1;
    }


    Positioner positioner;
    while ( true )
    {
        inputCapture >> img;

        positioner.frame( img );

        int res = waitKey( 20 );
        if ( res < 0 )
            continue;
        res &= 0xff;
        if ( res == 'q' )
            break;
    }
    inputCapture.release();

    return 0;
}





