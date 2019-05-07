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


int objDetectionInit(vpImage<unsigned char> I, vpMbEdgeTracker *tracker);
int objDetection(vpImage<unsigned char> I, vpMbEdgeTracker *tracker,
                 vpKeyPoint *keypoint_detection, vpHomogeneousMatrix *cMo,
                 double *error, double* elapsedTime);

#endif // VISION_H
