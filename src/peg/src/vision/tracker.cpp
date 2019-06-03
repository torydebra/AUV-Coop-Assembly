#include "header/tracker.h"

MonoTracker::MonoTracker(std::string callerName, std::string cameraName,
                         int trackerType){

  this->callerName = callerName;
  this->cameraName = cameraName;

  // path of source to find config files
  boost::filesystem::path path(__FILE__);
  path.remove_filename();
  sourcePath = path.string();


  tracker.setTrackerType(trackerType);

  tracker.loadConfigFile(sourcePath + configFile + cameraName + ".xml");

  //WARNING: Cannot set pose when model contains cylinder(s). This feature is not implemented yet.
  tracker.loadModel(sourcePath + caoModelNoCil);
  tracker.setOgreVisibilityTest(false);
  tracker.setDisplayFeatures(true);

  // Tells if the tracker has to compute the projection error
  tracker.setProjectionErrorComputation(true);

  std::cout << "[" << callerName << "][MONOTRACKER " << cameraName << "]"
            << " created\n";

}

MonoTracker::~MonoTracker(){
  //To clean up memory allocated by the xml library,
  //the user has to call this before the exit().
  vpXmlParser::cleanup();
}


int MonoTracker::initTrackingByClick(vpImage<unsigned char> *I){

  tracker.initClick(*I, sourcePath+initFileClick+cameraName + ".init", true);

  MonoTracker::monoTrackInit_priv(I);
  keypoint_detection.loadConfigFile(sourcePath+configFileDetector);
  keypoint_detection.loadLearningData(sourcePath+learnData+cameraName+".bin", true);

  std::cout << "[" << callerName << "][MONOTRACKER " << cameraName << "]"
            << " init by Click done\n";

}

int MonoTracker::initTrackingByPoint(vpImage<unsigned char> *I){

  tracker.initFromPoints(*I, sourcePath+initFile_w2D+cameraName+".init");

  MonoTracker::monoTrackInit_priv(I);
  keypoint_detection.loadConfigFile(sourcePath+configFileDetector);
  keypoint_detection.loadLearningData(sourcePath+learnData+cameraName+".bin", true);

  std::cout << "[" << callerName << "][MONOTRACKER " << cameraName << "]"
            << " init by Point done\n";

}

int MonoTracker::monoTrackInit_priv(vpImage<unsigned char> *I){

  try {

  vpCameraParameters cam;
  vpHomogeneousMatrix cMo;

//  vpDisplayOpenCV display;

//  display.init(*I, 300, 300, "Mono tracker "+cameraName);


  tracker.getCameraParameters(cam);

  tracker.track(*I);

  vpKeyPoint keypoint_learning;
  keypoint_learning.loadConfigFile(sourcePath+configFileDetector);

  std::vector<cv::KeyPoint> trainKeyPoints;
  double elapsedTime;
  keypoint_learning.detect(*I, trainKeyPoints, elapsedTime);
  // display found points
  vpDisplay::display(*I);
  for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
    vpDisplay::displayCross(*I, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
  }
  vpDisplay::displayText(*I, 10, 10, "All Keypoints...", vpColor::red);
  vpDisplay::displayText(*I, 30, 10, "Click to continue with detection...", vpColor::red);
  vpDisplay::flush(*I);
  vpDisplay::getClick(*I, true);

  std::vector<vpPolygon> polygons;
  std::vector<std::vector<vpPoint> > roisPt;
  std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair
      = tracker.getPolygonFaces(false);
  polygons = pair.first;
  roisPt = pair.second;
  std::vector<cv::Point3f> points3f;
  tracker.getPose(cMo);
  vpKeyPoint::compute3DForPointsInPolygons(cMo, cam, trainKeyPoints,
                                           polygons, roisPt, points3f);
  keypoint_learning.buildReference(*I, trainKeyPoints, points3f);
  keypoint_learning.saveLearningData(sourcePath+learnData+cameraName+".bin", true);

  // display found points
  vpDisplay::display(*I);
  for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
    vpDisplay::displayCross(*I, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
  }
  vpDisplay::displayText(*I, 10, 10, "Keypoints only on block...", vpColor::red);
  vpDisplay::displayText(*I, 30, 10, "Click to continue with detection...", vpColor::red);
  vpDisplay::flush(*I);
  vpDisplay::getClick(*I, true);


  } catch (vpException &e) {
    std::cout << "[" << callerName << "][MONOTRACKER " << cameraName << "]"
              << "Init_priv exception: " << e << std::endl;
  }

  return 0;
}


int MonoTracker::monoTrack(vpImage<unsigned char> *I,
                           vpHomogeneousMatrix *cMo,
                 double *error, double* elapsedTime){

  try {

    vpCameraParameters cam;
    tracker.getCameraParameters(cam);

    if (keypoint_detection.matchPoint(*I, cam, *cMo, *error, *elapsedTime)) {
      tracker.setPose(*I, *cMo);

      std::cout << "keypoint erro: " << *error << std::endl;
      std::cout << "projectERRO: " << tracker.getProjectionError() << "\n";
      std::cout << "Residual: " << sqrt( (tracker.getError()).sumSquare()) << std::endl;
      std::cout << "Residual normalized: "
                << sqrt( (tracker.getError()).sumSquare())/tracker.getError().size() << std::endl;
    }



  } catch (vpException &e) {
    std::cout << "[" << callerName << "][MONOTRACKER " << cameraName << "]"
              << "tracking exception: " << e << std::endl;
  }

  return 0;

}

void MonoTracker::getCameraParams(vpCameraParameters *param){
  tracker.getCameraParameters(*param);
}

void MonoTracker::display(vpImage<unsigned char> *I){

  vpHomogeneousMatrix cThole;
  tracker.getPose(cThole);

  vpCameraParameters cam;
  tracker.getCameraParameters(cam);

  tracker.display(*I, cThole, cam, vpColor::red, 2 , true);
}

/* ************************************************************************************************************/


StereoTracker::StereoTracker(std::string callerName, std::vector<std::string> cameraNames,
                             std::map<std::string, vpHomogeneousMatrix> mapCameraTransf,
                             std::vector<int> trackerTypes) {

  this->callerName = callerName;
  this->cameraNames = cameraNames;

  // path of source to find config files
  boost::filesystem::path path(__FILE__);
  path.remove_filename();
  sourcePath = path.string();

  tracker = new vpMbGenericTracker(cameraNames, trackerTypes);

  std::map<std::string, std::string> mapOfConfigFiles;

  for (int i=0; i< cameraNames.size(); i++){
    std::string configFileName =
        sourcePath+configFile+cameraNames.at(i)+".xml";
    mapOfConfigFiles[cameraNames.at(i)] = configFileName;
  }

   tracker->loadConfigFile(mapOfConfigFiles);

   tracker->loadModel(sourcePath+caoModel);

   tracker->setCameraTransformationMatrix(mapCameraTransf);

   tracker->setOgreVisibilityTest(false);
   tracker->setDisplayFeatures(true);

  std::cout << "[" << callerName << "][STEREOTRACKER] created\n";

}

StereoTracker::~StereoTracker(){
  delete tracker;
  //To clean up memory allocated by the xml library,
  //the user has to call this before the exit().
  vpXmlParser::cleanup();
}


int StereoTracker::initTrackingByClick(
    std::map<std::string, const vpImage<unsigned char>*> mapOfImages){

  std::map<std::string, std::string> mapOfInitFiles;

  for (int i=0; i< cameraNames.size(); i++){
    //dont store init file for depth camera, visp says it is not need
    if (cameraNames.at(i).compare("rangeRight") != 0) {
      std::string initFileName =
          sourcePath+initFileClick+cameraNames.at(i)+".init";

      mapOfInitFiles[cameraNames.at(i)] =  initFileName;
    }
  }

  try{
    tracker->initClick(mapOfImages, mapOfInitFiles, true);
  } catch (const vpException &e) {
    std::cerr << "[" << callerName << "][STEREOTRACKER] Catch a ViSP exception: " << e.what() << std::endl;
  }

  return 0;
}

int StereoTracker::initTrackingByPoint(
    std::map<std::string, const vpImage<unsigned char>*> mapOfImages){

  std::map<std::string, std::string> mapOfInitFiles;

  for (int i=0; i< cameraNames.size(); i++){
    //dont store init file for depth camera, visp says it is not need
    if (cameraNames.at(i).compare("rangeRight") != 0) {
      std::string initFileName =
          sourcePath+initFile_w2D+cameraNames.at(i)+".init";
      std::cout << initFileName << "\n";
      mapOfInitFiles[cameraNames.at(i)] =  initFileName;
    }
  }

  try{
    tracker->initFromPoints(mapOfImages, mapOfInitFiles);
  } catch (const vpException &e) {
    std::cerr << "[" << callerName << "][STEREOTRACKER] Catch a ViSP exception: " << e.what() << std::endl;
  }

}


int StereoTracker::stereoTrack(std::map<std::string, const vpImage<unsigned char>*> mapOfImages,
                   std::map<std::string, vpHomogeneousMatrix> *mapOfcameraToObj){
  try {

    tracker->track(mapOfImages);
    tracker->getPose(*mapOfcameraToObj);

  } catch (const vpException &e) {
    std::cerr << "[" << callerName << "][STEREOTRACKER] Catch a ViSP exception: " << e.what() << std::endl;
  }
  return 0;
}

int StereoTracker::stereoTrack(std::map<std::string, const vpImage<unsigned char>*> mapOfImages,
               std::map<std::string, pcl::PointCloud< pcl::PointXYZ >::ConstPtr> mapOfPointclouds,
               std::map<std::string, vpHomogeneousMatrix> *mapOfcameraToObj){
  try {

    tracker->track(mapOfImages, mapOfPointclouds);
    tracker->getPose(*mapOfcameraToObj);

  } catch (const vpException &e) {
    std::cerr << "[" << callerName << "][STEREOTRACKER] Catch a ViSP exception: " << e.what() << std::endl;
  }
  return 0;
}

void StereoTracker::getCamerasParams(std::map<std::string, vpCameraParameters> *mapOfCameraParams){

  tracker->getCameraParameters(*mapOfCameraParams);

}

void StereoTracker::display(std::map<std::string, const vpImage<unsigned char>*> mapOfImages){

  std::map<std::string, vpHomogeneousMatrix> mapOfcameraToObj;
  tracker->getPose(mapOfcameraToObj);

  std::map<std::string, vpCameraParameters> mapOfCamParams;
  tracker->getCameraParameters(mapOfCamParams);

  tracker->display(mapOfImages, mapOfcameraToObj, mapOfCamParams, vpColor::red, 2, true);
}








