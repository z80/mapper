
#ifndef __POSITIONER_H_
#define __POSITIONER_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <vector>
#include <iostream>

#include "newton_cam.h"
#include "newton_sam.h"

class Square
{
public:
    Square();
    ~Square();
    Square( const Square & inst );
    const Square & operator=( const Square & inst );
    bool operator<( const Square & inst ) const;
    void orderPts();

    std::vector<cv::Point2f> imgPts;
    std::vector<cv::Point2f> prevImgPts;
    std::vector<cv::Point2d> floorPts;
    std::vector<cv::Point2f> prevFloorPts;
    std::vector<cv::Point2d> finalPts;
};


class Positioner
{
public:
    Positioner();
    ~Positioner();

    bool loadSettings();

    void setRoundMode( bool en, double size = 2.0 );

    void frame( cv::Mat & img );
    void appendNewShapes();
    void resetImage2Floor();
    void resetFloor2Sample();

    // Determining drill shift vector by
    // rotating around stationary drill.
    // For example, drill may ne places into
    // a tight hole.
    void startDrillPos();
    void appendDrillPos();
    void endDrillPos();

    // Calibrating positioner axes.
    void startAxesPos();
    void appendAxesPos( int stepsX, int stepsY );
    void finishAxesPos();

    // Align to sample.
    void startSamplePos();
    void appendSamplePos( cv::Point2d r, cv::Point2d n );
    void appendSampleFrontPos( cv::Point2d r, cv::Point2d n );
    void endSamplePos(); // Determine end mill diameter.
    void endSamplePos( double d ); // End mill diameter is provided, use it while aligning RF.
    void calcSample2Floor();


    void matchSquaresRound( std::vector<Square> & squares );
    void applyPerspective( std::vector<Square> & squares );
    void applyCamera( std::vector<Square> & squares );
    bool saveImg2Floor();
    bool loadImg2Floor();

    bool prevPointPositions( cv::Mat & gray, std::vector<Square> & squares );

    bool fieldOfView( std::vector<double> & corners );
    bool drillPos( double & x, double & y, bool ignoreSample = false );
    bool knownFeatures( std::vector<double> & corners );
    bool visibleFeatures( std::vector<double> & corners );

// Just for now public.
public:
    static void findSquares( const cv::Mat & image, std::vector<Square> & squares );
    void dbgDisplay( cv::Point imgSz );

    // To derive optical flow.
    cv::Size imgSize;
    cv::Mat grayPrev;

    // Camera transformation matrices which are defined exernally.
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat perspective;

    // Probably will determine a few more.
    cv::Mat img2Floor; // Image position matrix.
    bool roundMode;
    double sideSize;
    bool floorLocked;
    cv::Point2d R;     // Drill position in camera ref frame.
    double r;          // Drill radius.
    cv::Mat floor2CrossVise;
    cv::Mat floor2Sample;
    cv::Mat sample2Floor;
    // Experimental.
    double sampleAngle, sampleX, sampleY;
    cv::Mat img2FloorSmooth;
    int     noOpticalFlowCounter;

    // As a result need sample to tool.
    // Which is supposed to be
    // ptOnTool = cam2Tool*A*( R + sample2Cam * ptOnSample ).

    std::vector<Square> squares; // For visualization.
    bool appendNew;


    // Drill pos alignment.
    std::vector<double> drillAs;

    // Vise axes alignment.
    std::vector<double> vise2As;

    // Sample alignment.
    std::vector<double> sample2As;
    std::vector<double> sampleOffEdge;

    NewtonCam newtonCam;
    NewtonSam newtonSam;

    // Constants.
    static const double SEARCH_RANGE;
    static const bool   DEBUG;
    static const double ALPHA;
    static const int    MIN_NO_FLOW_FRAMES;
    static const int    IMAGE_MARGIN;
    static const double MAX_FLOW_SPEED;
    static const double FLOOR_POS_MARGIN;
    static const double FLOOR_DIR_MARGIN;
};


#endif


