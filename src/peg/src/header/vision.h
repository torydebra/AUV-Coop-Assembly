#ifndef VISION_H
#define VISION_H

#include <iostream>
#include "../rosInterfaces/header/robotVisionInterface.h"
#include "../rosInterfaces/header/worldInterface.h"
#include "../helper/header/infosVision.h"
#include "../helper/header/logger.h"

#include <visp3/gui/vpDisplayX.h> //recommended for linux from docuentation

#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpKeyPoint.h>

#include <visp3/mbt/vpMbGenericTracker.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>

#include <cmat/cmat.h>



const std::string configFileL = "/home/tori/UWsim/Peg/src/peg/src/vision/cameraL.xml";
const std::string configFileR = configFileL; //ONLY because instrinsic param of cameras L and R are the same
const std::string caoModel = "/home/tori/UWsim/Peg/src/peg/src/vision/blockHoleCilinder.cao";
const std::string configFileDetector = "/home/tori/UWsim/Peg/src/peg/src/vision/detection-config-SIFT.xml";
const std::string initFileClickLeft = "/home/tori/UWsim/Peg/src/peg/src/vision/3DPointSquareFace4_left.init";
const std::string initFileClickRight = "/home/tori/UWsim/Peg/src/peg/src/vision/3DPointSquareFace4_right.init";
const std::string learnData = "/home/tori/UWsim/Peg/src/peg/src/vision/blockHole_learning_data.bin";
const std::string transfcameraLtoR = "/home/tori/UWsim/Peg/src/peg/src/vision/lTr.txt";


//int objDetectionInit(vpImage<unsigned char> I, vpMbGenericTracker *tracker);
//int objDetection(vpImage<unsigned char> I, vpMbGenericTracker *tracker,
//                 vpKeyPoint *keypoint_detection, vpHomogeneousMatrix *cMo,
//                 double *error, double* elapsedTime);
int stereoTracking(vpImage<unsigned char> I_left, vpImage<unsigned char> I_right,
                   vpMbGenericTracker *tracker,
                   vpHomogeneousMatrix *cLeftTo, vpHomogeneousMatrix *cRightTo);
int initTracker(vpMbGenericTracker *tracker, int nCameras, Eigen::Matrix4d cLTcR);


#endif // VISION_H
