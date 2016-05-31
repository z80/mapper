
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

    bool loadSettings();

    void frame( cv::Mat & img );
    void appendNewShapes();
    void resetImage2Floor();

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
    void startLinePos();
    void appendLinePos( cv::Point2d r, cv::Point2d n );
    void endLinePos();


    void matchSquares( std::vector<std::vector<cv::Point>> & squares );
    void applyPerspective( std::vector<std::vector<cv::Point>> & squares );
    void applyCamera();
    bool matchSquares( int knownInd,
                       int foundInd,
                       std::vector<cv::Point2d> & knownPts,
                       std::vector<cv::Point2d> & foundPts );

// Just for now public.
public:
    static void Positioner::findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares );
    void dbgDisplay( cv::Point imgSz );

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat perspective;

    cv::Mat img2Floor; // Image position matrix.
    cv::Point2d R;     // Drill position in camera ref frame.
    double r;          // Drill radius.

    // Probably will determine a few more.
    cv::Mat cam2Tool;
    cv::Mat sample2Cam;
    // As a result need sample to tool.
    // Which is supposed to be
    // ptOnTool = cam2Tool*A*( R + sample2Cam * ptOnSample ).

    std::vector<std::vector<cv::Point2d>> knownSquares,      //
                                          locatedSquaresFloor; // After A.
    std::vector<std::vector<cv::Point2d>> locatedSquaresImg; // After perspective.
    bool appendNew;


    // Drill pos alignment.
    std::vector<double> drillAs;

    // Vise axes alignment.
    std::vector<double> viseSteps2As;

    // Constants.
    static const double SEARCH_RANGE;
    static const bool   DEBUG;
    static const double ALPHA;
};


#endif


