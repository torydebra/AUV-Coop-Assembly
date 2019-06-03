#ifndef TRACKER_H
#define TRACKER_H

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <boost/filesystem/path.hpp>

#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/vision/vpKeyPoint.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#include <pcl_ros/io/pcd_io.h>

#include "../../support/header/conversions.h"


//relative to source (tracker.cpp) path to config file and images used in algos
const std::string relative = "/data/";
const std::string configFile = relative + "camera_";
const std::string caoModelNoCil = relative + "blockHole.cao";
//const std::string caoModel = relative + "blockHoleCilinder.cao";
//const std::string caoModel = relative + "blockHole.cao";
const std::string caoModel = relative + "pegModelMay.cao";
const std::string configFileDetector = relative + "detection-config-MONO.xml";
//const std::string initFileClick = relative + "3DPointSquareFace4_";
const std::string initFileClick = relative + "3DPointPeg_";
const std::string initFile_w2D = relative + "3DPointSquareFace4_w2d_";
const std::string learnData = relative + "blockHole_learning_data_";
//const std::string transfcameraLtoR = relative + "lTr.txt";

class MonoTracker
{
public:
  MonoTracker(std::string callerName, std::string cameraName,
              int trackerType);
  ~MonoTracker();

  int initTrackingByClick(vpImage<unsigned char> *I);
  int initTrackingByPoint(vpImage<unsigned char> *I);
  int monoTrack(vpImage<unsigned char> *I,
                vpHomogeneousMatrix *cMo,
                double *error, double* elapsedTime);
  void getCameraParams(vpCameraParameters *param);
  void display(vpImage<unsigned char> *I);

private:
  vpMbGenericTracker tracker;
  std::string callerName;
  std::string cameraName;
  std::string sourcePath;
  vpKeyPoint keypoint_detection;


  int monoTrackInit_priv(vpImage<unsigned char> *I);

};


class StereoTracker
{
public:
  StereoTracker(std::string callerName, std::vector<std::string> cameraNames,
                std::map<std::string, vpHomogeneousMatrix> mapCameraTransf,
                std::vector<int> trackerTypes);
  ~StereoTracker();

  int initTrackingByClick( std::map<std::string, const vpImage<unsigned char>*> mapOfImages);
  int initTrackingByPoint(std::map<std::string, const vpImage<unsigned char>*> mapOfImages);
  int stereoTrack(std::map<std::string, const vpImage<unsigned char>*> mapOfImages,
                  std::map<std::string, vpHomogeneousMatrix> *mapOfCameraPoses);
  int stereoTrack(std::map<std::string, const vpImage<unsigned char>*> mapOfImages,
                 std::map<std::string, pcl::PointCloud< pcl::PointXYZ >::ConstPtr> mapOfPointclouds,
                 std::map<std::string, vpHomogeneousMatrix> *mapOfcameraToObj);
  void getCamerasParams(std::map<std::string, vpCameraParameters> *mapOfCameraParams);
  void display(std::map<std::string, const vpImage<unsigned char>*> mapOfImages);

private:
  vpMbGenericTracker *tracker;

  std::string callerName;
  std::vector<std::string> cameraNames;

  std::string sourcePath;


};

#endif // TRACKER_H
