#ifndef VISION_H
#define VISION_H

#include <iostream>
#include "../rosInterfaces/header/robotVisionInterface.h"
#include "../rosInterfaces/header/worldInterface.h"
#include "../helper/header/infosVision.h"
#include "../helper/header/logger.h"

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpKeyPoint.h>
#include <visp3/mbt/vpMbGenericTracker.h>

//#include <visp3/tt/vpTemplateTrackerSSDInverseCompositional.h>
//#include <visp3/tt/vpTemplateTrackerWarpHomography.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

//feature 2d + homography
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <cmat/cmat.h>



const std::string configFileL = "/home/tori/UWsim/Peg/src/peg/src/vision/data/camera_left.xml";
const std::string configFileR = configFileL; //ONLY because instrinsic param of cameras L and R are the same
const std::string caoModel = "/home/tori/UWsim/Peg/src/peg/src/vision/data/blockHoleCilinder.cao";
const std::string configFileDetector = "/home/tori/UWsim/Peg/src/peg/src/vision/data/detection-config-SIFT.xml";
const std::string initFileClickLeft = "/home/tori/UWsim/Peg/src/peg/src/vision/data/3DPointSquareFace4_left.init";
const std::string initFileClickRight = "/home/tori/UWsim/Peg/src/peg/src/vision/data/3DPointSquareFace4_right.init";
const std::string initFileClickLeft_w2D = "/home/tori/UWsim/Peg/src/peg/src/vision/data/3DPointSquareFace4_w2d_left.init";
const std::string initFileClickRight_w2D = "/home/tori/UWsim/Peg/src/peg/src/vision/data/3DPointSquareFace4_w2d_right.init";
const std::string learnData = "/home/tori/UWsim/Peg/src/peg/src/vision/data/blockHole_learning_data.bin";
const std::string transfcameraLtoR = "/home/tori/UWsim/Peg/src/peg/src/vision/data/lTr.txt";


int objDetectionInit(vpImage<unsigned char> I, vpMbGenericTracker *tracker);
int objDetection(vpImage<unsigned char> I, vpMbGenericTracker *tracker,
                 vpKeyPoint *keypoint_detection, vpHomogeneousMatrix *cMo,
                 double *error, double* elapsedTime);
int stereoTracking(vpImage<unsigned char> I_left, vpImage<unsigned char> I_right,
                   vpMbGenericTracker *tracker,
                   vpHomogeneousMatrix *cLeftTo, vpHomogeneousMatrix *cRightTo);
int initTracker(vpMbGenericTracker *tracker, int nCameras, Eigen::Matrix4d cLTcR);


/// function to find square, which works well
void drawSquares( cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares,
                         const char* wndname = "Square Detection Demo");
void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares,
                         int thresh = 50, int N = 11);
int orderAngles(std::vector<std::vector<cv::Point>> angles, std::vector<std::vector<cv::Point>> *orderedAngles);
int orderAngles(std::vector<cv::Point> angles, std::vector<cv::Point> *orderedAngles);
cv::Point getCenter(std::vector<cv::Point> points);
double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 );


#endif // VISION_H
