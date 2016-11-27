#include <stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <limits>
#include <functional>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio/videoio.hpp"


#include "ukfm.h"
#include "mag.h"
#include "mag1d.h"

int main()
{
    Mag1D * m = new Mag1D();
    m->process();

    //Mag * m = new Mag();
    //m->process();

    return 0;
}









