#ifndef MISSIONMANAGERVISION_H
#define MISSIONMANAGERVISION_H

#include <iostream>
#include <fstream>
#include "../rosInterfaces/header/robotVisionInterface.h"
#include "../rosInterfaces/header/worldInterface.h"
#include "../rosInterfaces/header/visionInterfaceVision.h"
#include "../helper/header/infosVision.h"
#include "../helper/header/logger.h"

#include "../vision/header/detector.h"
#include "../vision/header/tracker.h"
#include "../vision/header/depthCameraSupport.h"

#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include <pcl_ros/io/pcd_io.h>

const std::string relativeMissMan = "/vision/data/";
const std::string templName = relativeMissMan + "templateFrontLittle.jpg";



int main(int, char**);
void append2Dto3Dfile(std::vector<cv::Point> found4CornersL, std::vector<cv::Point> found4CornersR,
                      std::string sourcePath);
void append2Dto3Dfile(std::vector<cv::Point> found4CornersL,
                      std::string sourcePath);


#endif // MISSIONMANAGERVISION_H
