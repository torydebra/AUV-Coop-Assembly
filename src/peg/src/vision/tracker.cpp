#include "header/tracker.h"

MonoTracker::MonoTracker(std::string callerName, std::string cameraName,
                         vpMbGenericTracker::vpTrackerType trackerType){

  this->callerName = callerName;
  this->cameraName = cameraName;

  // path of source to find config files
  boost::filesystem::path path(__FILE__);
  path.remove_filename();
  sourcePath = path.string();


  std::vector<int> trackerTypes;

  trackerTypes.push_back(trackerType);
  trackerTypes.push_back(trackerType);

  tracker = vpMbGenericTracker(trackerTypes);

  tracker.loadConfigFile(sourcePath + configFile + cameraName + ".xml");

  tracker.loadModel(sourcePath + caoModel);
  tracker.setOgreVisibilityTest(false);
  tracker.setDisplayFeatures(true);

  std::cout << "[" << callerName << "][MONOTRACKER " << cameraName << "]"
            << " init done\n";

}

int MonoTracker::initTrackingByClick(vpImage<unsigned char> I){

  tracker.initClick(I, sourcePath+initFileClick+cameraName + ".init", true);

  MonoTracker::monoTrackInit_priv(I);
  keypoint_detection.loadConfigFile(sourcePath+configFileDetector);
  keypoint_detection.loadLearningData(sourcePath+learnData, true);

  std::cout << "[" << callerName << "][MONOTRACKER " << cameraName << "]"
            << " init by Click done\n";

}

int MonoTracker::initTrackingByPoint(vpImage<unsigned char> I){

  tracker.initFromPoints(I, sourcePath+initFile_w2D+cameraName+".init");

  MonoTracker::monoTrackInit_priv(I);
  keypoint_detection.loadConfigFile(sourcePath+configFileDetector);
  keypoint_detection.loadLearningData(sourcePath+learnData, true);

  std::cout << "[" << callerName << "][MONOTRACKER " << cameraName << "]"
            << " init by Point done\n";

}

int MonoTracker::monoTrackInit_priv(vpImage<unsigned char> I){

  try {

  vpCameraParameters cam;
  vpHomogeneousMatrix cMo;

  vpDisplayOpenCV display;

  display.init(I, 300, 300, "Mono tracker "+cameraName);


  tracker.getCameraParameters(cam);

  tracker.track(I);

  vpKeyPoint keypoint_learning;
  keypoint_learning.loadConfigFile(configFileDetector);

  std::vector<cv::KeyPoint> trainKeyPoints;
  double elapsedTime;
  keypoint_learning.detect(I, trainKeyPoints, elapsedTime);
  // display found points
  vpDisplay::display(I);
  for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
    vpDisplay::displayCross(I, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
  }
  vpDisplay::displayText(I, 10, 10, "All Keypoints...", vpColor::red);
  vpDisplay::displayText(I, 30, 10, "Click to continue with detection...", vpColor::red);
  vpDisplay::flush(I);
  vpDisplay::getClick(I, true);

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
  keypoint_learning.buildReference(I, trainKeyPoints, points3f);
  keypoint_learning.saveLearningData(learnData, true);

  // display found points
  vpDisplay::display(I);
  for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
    vpDisplay::displayCross(I, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
  }
  vpDisplay::displayText(I, 10, 10, "Keypoints only on block...", vpColor::red);
  vpDisplay::displayText(I, 30, 10, "Click to continue with detection...", vpColor::red);
  vpDisplay::flush(I);
  vpDisplay::getClick(I, true);


  } catch (vpException &e) {
    std::cout << "[" << callerName << "][MONOTRACKER " << cameraName << "]"
              << "Init priv exception: " << e << std::endl;
  }

  return 0;
}


int MonoTracker::monoTrack(vpImage<unsigned char> I,
                           vpHomogeneousMatrix *cMo,
                 double *error, double* elapsedTime){

  try {

    vpCameraParameters cam;
    tracker.getCameraParameters(cam);

    if (keypoint_detection.matchPoint(I, cam, *cMo, *error, *elapsedTime)) {
      tracker.setPose(I, *cMo);
      tracker.display(I, *cMo, cam, vpColor::red, 2);
    }

  } catch (vpException &e) {
    std::cout << "[" << callerName << "][MONOTRACKER " << cameraName << "]"
              << "tracking exception: " << e << std::endl;
  }

  return 0;

}

/* ************************************************************************************************************/


StereoTracker::StereoTracker(std::string callerName, std::vector<std::string> cameraNames,
                             std::map<std::string, vpHomogeneousMatrix> mapCameraTransf,
                             vpMbGenericTracker::vpTrackerType trackerType) {

  this->callerName = callerName;
  this->cameraNames = cameraNames;

  // path of source to find config files
  boost::filesystem::path path(__FILE__);
  path.remove_filename();
  sourcePath = path.string();


  std::vector<int> trackerTypes;

  trackerTypes.push_back(trackerType);
  trackerTypes.push_back(trackerType);

  tracker = vpMbGenericTracker(trackerTypes);

  std::map<std::string, std::string> mapOfConfigFiles;

  for (int i=0; i< cameraNames.size(); i++){
    std::string configFileName =
        sourcePath+configFile+cameraNames.at(i)+".xml";
    mapOfConfigFiles.insert({cameraNames.at(i), configFileName});
  }

  tracker.loadConfigFile(mapOfConfigFiles);
  tracker.loadModel(sourcePath+caoModel);

  tracker.setCameraTransformationMatrix(mapCameraTransf);

  tracker.setOgreVisibilityTest(false);
  tracker.setDisplayFeatures(true);

  std::cout << "[" << callerName << "][STEREOTRACKER] init done\n";


}


int StereoTracker::initTrackingByClick(
    std::map<std::string, const vpImage<unsigned char>*> mapOfImages){

  std::map<std::string, std::string> mapOfInitFiles;

  for (int i=0; i< cameraNames.size(); i++){
    std::string initFileName =
        sourcePath+initFileClick+cameraNames.at(i)+".init";
    mapOfInitFiles.insert({cameraNames.at(i), initFileName});
  }


  tracker.initClick(mapOfImages, mapOfInitFiles, true);
  return 0;
}

int StereoTracker::initTrackingByPoint(
    std::map<std::string, const vpImage<unsigned char>*> mapOfImages){

  std::map<std::string, std::string> mapOfInitFiles;

  for (int i=0; i< cameraNames.size(); i++){
    std::string initFileName =
        sourcePath+initFile_w2D+cameraNames.at(i)+".init";
    mapOfInitFiles.insert({cameraNames.at(i), initFileName});
  }

  tracker.initFromPoints(mapOfImages, mapOfInitFiles);

}


int StereoTracker::stereoTrack(std::map<std::string, const vpImage<unsigned char>*> mapOfImages,
                   std::map<std::string, vpHomogeneousMatrix> *mapOfCameraPoses){
  try {

    tracker.track(mapOfImages);
    tracker.getPose(*mapOfCameraPoses);

  } catch (const vpException &e) {
    std::cerr << "[" << callerName << "][STEREOTRACKER] Catch a ViSP exception: " << e.what() << std::endl;
  }
  return 0;
}

int StereoTracker::getCamerasParams(std::vector<vpCameraParameters> ){

}








