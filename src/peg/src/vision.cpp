#include "header/vision.h"

int main(int argc, char** argv){

  if (argc < 2){
    std::cout << "[VISION] Please insert robot name for Vision"<< std::endl;
    return -1;
  }
  std::string robotName = argv[1];
  std::string pathLog;
  if (LOG && (argc > 2)){   //if flag log setted to 1 and path log is given
    pathLog = argv[2];
  }


  /// ROS NODE
  ros::init(argc, argv, robotName + "_Vision");
  std::cout << "[" << robotName << "][VISION] Start" << std::endl;
  ros::NodeHandle nh;

/** ***************************************************************************************************************
                                 INITIALIZE THINGS
*******************************************************************************************************************/

  /// struct container data to pass among functions
  InfosVision robVisInfo;

  /// Ros Interfaces
  RobotVisionInterface robotVisInterface(nh, robotName);
  robotVisInterface.init();

  std::string cameraLName = "bowtechL_C";
  std::string cameraRName = "bowtechR_C";
  std::string holeName = "hole";
  WorldInterface worldInterface("Vision2");
  worldInterface.waitReady(holeName);


  /// Set initial state
  robotVisInterface.getwTv(&(robVisInfo.robotState.wTv_eigen));

  worldInterface.getT(&robVisInfo.robotStruct.vTcameraL, robotName, cameraLName);
  worldInterface.getT(&robVisInfo.robotStruct.vTcameraR, robotName, cameraRName);
  worldInterface.getT(&robVisInfo.robotStruct.cLTcR, cameraLName, cameraRName);
  // real hole pose to log errors of pose estimation
  worldInterface.getwT(&(robVisInfo.transforms.wTh_eigen), holeName);

  /// Initial images
  vpImage<unsigned char> imageL_vp, imageR_vp;
  cv::Mat imageL_cv, imageR_cv;

  robotVisInterface.getLeftImage(&imageL_cv);
  robotVisInterface.getRightImage(&imageR_cv);
  imageL_cv.convertTo(imageL_cv, CV_8U); //TODO check if necessary convert in 8U
  imageL_cv.convertTo(imageR_cv, CV_8U); //TODO check if necessary convert in 8U
  vpImageConvert::convert(imageL_cv, imageL_vp);
  vpImageConvert::convert(imageR_cv, imageR_vp);


/** ***************************************************************************************************************
                                 LOGGING
********************************************************************************************************************/

  Logger logger;
  if (pathLog.size() > 0){
    logger = Logger(robotName, pathLog);
    logger.createDirectoryForNode();
  }





/** *********************************************************************************************************+*********
                                 MAIN VISION LOOP
*******************************************************************************************************************/
  std::vector<int> trackerTypes;
  trackerTypes.push_back(vpMbGenericTracker::KLT_TRACKER);
  //trackerTypes.push_back(vpMbGenericTracker::KLT_TRACKER);

  vpMbGenericTracker tracker(trackerTypes);
  initTracker(&tracker, trackerTypes.size(), robVisInfo.robotStruct.cLTcR);

  vpDisplayX display_left;
  vpDisplayX display_right;
  display_left.setDownScalingFactor(vpDisplay::SCALE_AUTO);
  display_right.setDownScalingFactor(vpDisplay::SCALE_AUTO);
  display_left.init(imageL_vp, 100, 100, "Model-based tracker (Left)");
  display_right.init(imageR_vp, 110 + (int)imageL_vp.getWidth(), 100,
                     "Model-based tracker (Right)");

  bool initByClick = true;
  if (initByClick){ //init by click
    switch (trackerTypes.size()){
    case 1:
      tracker.initClick(imageL_vp, initFileClick, false);
      break;
    case 2:
      tracker.initClick(imageL_vp, imageR_vp, initFileClick, initFileClick, true);
      break;
    default:
      std::cerr << "WRONG NUMBER OF TRACKERS\n";
    }

  } else { //find point da solo COME??
    //tracker->initTracking(I);
  }

  objDetectionInit(imageL_vp, &tracker);

  vpKeyPoint keypoint_detection;
  keypoint_detection.loadConfigFile(configFileDetector);
  keypoint_detection.loadLearningData(learnData, true);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    robotVisInterface.getwTv(&(robVisInfo.robotState.wTv_eigen));
    worldInterface.getwT(&(robVisInfo.transforms.wTh_eigen), holeName);

    robotVisInterface.getLeftImage(&imageL_cv);
    robotVisInterface.getRightImage(&imageR_cv);
    imageL_cv.convertTo(imageL_cv, CV_8U); //TODO check if necessary convert in 8U
    imageL_cv.convertTo(imageR_cv, CV_8U); //TODO check if necessary convert in 8U
    vpImageConvert::convert(imageL_cv, imageL_vp);
    vpImageConvert::convert(imageR_cv, imageR_vp);

    /// OBJECT DETECTION
    vpHomogeneousMatrix cLThole;
    double ransacError = 0.0;
    double elapsedTime = 0.0;
    objDetection(imageL_vp, &tracker, &keypoint_detection, &cLThole,
                 &ransacError, &elapsedTime);

    robVisInfo.transforms.wTh_estimated_eigen =
        robVisInfo.robotState.wTv_eigen *
        robVisInfo.robotStruct.vTcameraL *
        CONV::matrix_visp2eigen(cLThole);


    CMAT::TransfMatrix wThole_cmat =
        CONV::matrix_eigen2cmat(robVisInfo.transforms.wTh_eigen);
    CMAT::TransfMatrix wTholeEstimated_cmat =
        CONV::matrix_eigen2cmat(robVisInfo.transforms.wTh_estimated_eigen);

    CMAT::Vect6 swappedError =
        CMAT::CartError(wThole_cmat, wTholeEstimated_cmat);
    CMAT::Vect6 error;
    error.SetFirstVect3(swappedError.GetSecondVect3());
    error.SetSecondVect3(swappedError.GetFirstVect3());
    logger.writeCmatMatrix(error, "error");



    ros::spinOnce();
    loop_rate.sleep();
  }


   vpXmlParser::cleanup();

   return 0;
}

int objDetectionInit(vpImage<unsigned char> I, vpMbGenericTracker *tracker){

  try {

  vpCameraParameters cam;
  vpHomogeneousMatrix cMo;

  vpDisplayX display;

  display.init(I, 100, 100, "Model-based edge tracker");

  tracker->getCameraParameters(cam);

  tracker->track(I);


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
      = tracker->getPolygonFaces(false);
  polygons = pair.first;
  roisPt = pair.second;
  std::vector<cv::Point3f> points3f;
  tracker->getPose(cMo);
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
    std::cout << "Catch an exception: " << e << std::endl;
  }

  return 0;
}


///OBJECT DETECTIONN
int objDetection(vpImage<unsigned char> I, vpMbGenericTracker *tracker,
                 vpKeyPoint *keypoint_detection, vpHomogeneousMatrix *cMo,
                 double *error, double* elapsedTime){

  try {

    vpCameraParameters cam;
    tracker->getCameraParameters(cam);

    vpDisplay::display(I);
    vpDisplay::displayText(I, 10, 10, "Detection and localization in process...", vpColor::red);
    if (keypoint_detection->matchPoint(I, cam, *cMo, *error, *elapsedTime)) {
      tracker->setPose(I, *cMo);
      tracker->display(I, *cMo, cam, vpColor::red, 2);
      vpDisplay::displayFrame(I, *cMo, cam, 0.25, vpColor::none, 3);
    }
    vpDisplay::displayText(I, 30, 10, "A click to exit.", vpColor::red);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false)) {
      return 1;
    }

  } catch (vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }

  return 0;

}

int stereoTracking(vpImage<unsigned char> I_left, vpImage<unsigned char> I_right,
                   vpMbGenericTracker *tracker,
                   vpHomogeneousMatrix *cLeftTo, vpHomogeneousMatrix *cRightTo){

  try {
    vpDisplay::display(I_left);
    vpDisplay::display(I_right);


    tracker->track(I_left, I_right);

    tracker->getPose(*cLeftTo, *cRightTo);

    vpCameraParameters cam_left, cam_right;
    tracker->getCameraParameters(cam_left, cam_right);
    tracker->display(I_left, I_right, *cLeftTo, *cRightTo, cam_left, cam_right, vpColor::red, 2);
    vpDisplay::displayFrame(I_left, *cLeftTo, cam_left, 0.25, vpColor::none, 2);
    vpDisplay::displayFrame(I_right, *cRightTo, cam_right, 0.25, vpColor::none, 2);
    vpDisplay::displayText(I_left, 10, 10, "A click to exit...", vpColor::red);
    vpDisplay::flush(I_left);
    vpDisplay::flush(I_right);
    if (vpDisplay::getClick(I_left, false)) {
      return 1;
    }

  } catch (const vpException &e) {
    std::cerr << "Catch a ViSP exception: " << e.what() << std::endl;
  }
  return 0;
}


int initTracker(vpMbGenericTracker *tracker, int nCameras, Eigen::Matrix4d cLTcR){

  //vpMbGenericTracker temp(trackerTypes);

  //*tracker = temp;

  switch (nCameras){
  case 1: {

      tracker->loadConfigFile(configFileL);
      tracker->loadModel(caoModel);
    }

    break;
  case 2: {
      tracker->loadConfigFile(configFileL, configFileR);
      tracker->loadModel(caoModel, caoModel);

      std::map<std::string, vpHomogeneousMatrix> mapCameraTransf;
      mapCameraTransf["Camera1"] = vpHomogeneousMatrix(); //identity
      mapCameraTransf["Camera2"] = CONV::transfMatrix_eigen2visp(cLTcR);;

      tracker->setCameraTransformationMatrix(mapCameraTransf);


    }
    break;
  default:
    std::cerr << "WRONG NUMBER OF TRACKERS\n";

  }
  tracker->setOgreVisibilityTest(false);
  tracker->setDisplayFeatures(true);

  return 0;

}




//    /// KEYPOINT DETECTOR
//    vpImage<unsigned char> I;
//    vpImageIo::read(I, "/home/tori/UWsim/Peg/cameraL_front.png");

////    vpDisplayX d(I);
////    vpDisplay::setTitle(I, "My image");
////    vpDisplay::display(I);
////    vpDisplay::flush(I);
////    std::cout << "A click to quit..." << std::endl;
////    vpDisplay::getClick(I);


//    const std::string detectorName = "ORB";
//    const std::string extractorName = "ORB";
//    // Hamming distance must be used with ORB
//    const std::string matcherName = "BruteForce-Hamming";
//    vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
//    vpKeyPoint keypoint(detectorName, extractorName, matcherName, filterType);
//    std::cout << "Reference keypoints=" << keypoint.buildReference(I) << std::endl;

//    vpImage<unsigned char> Idisp;
//    Idisp.resize(I.getHeight(), 2 * I.getWidth());
//    Idisp.insert(I, vpImagePoint(0, 0));
//    Idisp.insert(I, vpImagePoint(0, I.getWidth()));
//    vpDisplayOpenCV d(Idisp, 0, 0, "Matching keypoints with ORB keypoints");
//    vpDisplay::display(Idisp);
//    vpDisplay::flush(Idisp);

//    while (1) {
//      Idisp.insert(I, vpImagePoint(0, I.getWidth()));
//      vpDisplay::display(Idisp);
//      vpDisplay::displayLine(Idisp, vpImagePoint(0, I.getWidth()), vpImagePoint(I.getHeight(), I.getWidth()),
//                             vpColor::white, 2);
//      unsigned int nbMatch = keypoint.matchPoint(I);
//      std::cout << "Matches=" << nbMatch << std::endl;
//      vpImagePoint iPref, iPcur;
//      for (unsigned int i = 0; i < nbMatch; i++) {
//        keypoint.getMatchedPoints(i, iPref, iPcur);
//        vpDisplay::displayLine(Idisp, iPref, iPcur + vpImagePoint(0, I.getWidth()), vpColor::green);
//      }
//      vpDisplay::flush(Idisp);
//      if (vpDisplay::getClick(Idisp, false))
//        break;
//    }
//    vpDisplay::getClick(Idisp);






   ///KEYPOINT TRACKER
   /// IT WORKS!

//  try {
//    bool opt_init_by_click = false;

//    vpImage<unsigned char> I;

//    cv::Mat cvI = imageCVL.get()->image;
//    cvI.convertTo(cvI, CV_8U);

//    vpImageConvert::convert(cvI, I);

//    vpDisplayOpenCV d(I, 0, 0, "Klt tracking");
//    vpDisplay::display(I);
//    vpDisplay::flush(I);
//    vpKltOpencv tracker;
//    tracker.setMaxFeatures(200);
//    tracker.setWindowSize(10);
//    tracker.setQuality(0.01);
//    tracker.setMinDistance(15);
//    tracker.setHarrisFreeParameter(0.04);
//    tracker.setBlockSize(9);
//    tracker.setUseHarris(1);
//    tracker.setPyramidLevels(3);
//    // Initialise the tracking
//    if (opt_init_by_click) {
//      vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;

//      std::vector<cv::Point2f> feature;

//      vpImagePoint ip;
//      do {
//        vpDisplay::displayText(I, 10, 10, "Left click to select a point, right to start tracking", vpColor::red);
//        if (vpDisplay::getClick(I, ip, button, false)) {
//          if (button == vpMouseButton::button1) {
//            feature.push_back(cv::Point2f((float)ip.get_u(), (float)ip.get_v()));
//            vpDisplay::displayCross(I, ip, 12, vpColor::green);
//          }
//        }
//        vpDisplay::flush(I);
//        vpTime::wait(20);
//      } while (button != vpMouseButton::button3);

//      tracker.initTracking(cvI, feature);

//    } else {
//      tracker.initTracking(cvI);
//    }
//    std::cout << "Tracker initialized with " << tracker.getNbFeatures() << " features" << std::endl;

//    while (ros::ok()) {

//      //(src, dest)
//      cvI = imageCVL.get()->image;
//      cvI.convertTo(cvI, CV_8U);
//      vpImageConvert::convert(cvI, I);
//      vpDisplay::display(I);
//      vpDisplay::flush(I);


////      if (opt_init_by_click) {
////        vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;

////        std::vector<cv::Point2f> feature;
////        vpImagePoint ip;
////        do {
////          vpDisplay::displayText(I, 10, 10, "Left click to select a point, right to start tracking", vpColor::red);
////          if (vpDisplay::getClick(I, ip, button, false)) {
////            if (button == vpMouseButton::button1) {
////              feature.push_back(cv::Point2f((float)ip.get_u(), (float)ip.get_v()));
////              vpDisplay::displayCross(I, ip, 12, vpColor::green);
////            }
////          }
////          vpDisplay::flush(I);
////          vpTime::wait(20);
////        } while (button != vpMouseButton::button3);

////        tracker.initTracking(cvI, feature);
////      }
//      tracker.track(cvI);
//      tracker.display(I, vpColor::red);
//      vpDisplay::flush(I);

//      ros::spinOnce();
//      loop_rate.sleep();

//    }
//    vpDisplay::getClick(I);

//    return 0;
//  } catch (const vpException &e) {
//    std::cout << "Catch an exception: " << e << std::endl;
//  }








    // segui object detection tutorial



/**
  Axis in the model are:
  x going out of the plane
  y pointing on the right
  z go up in the plane
  */







/// tracking ELLIPSE movinge edges

//    try {
//      vpImage<unsigned char> I;

//      vpImageIo::read(I, "/home/tori/UWsim/Peg/cameraL_front.png");
//      vpDisplayX d(I, 0, 0, "Camera view");

//      vpDisplay::display(I);
//      vpDisplay::flush(I);
//      vpMe me;
//      me.setRange(25);
//      me.setThreshold(15000);
//      me.setSampleStep(10);
//      vpMeEllipse ellipse;
//      ellipse.setMe(&me);
//      ellipse.setDisplay(vpMeSite::RANGE_RESULT);
//      ellipse.initTracking(I);
//      while (1) {
//        vpDisplay::display(I);
//        ellipse.track(I);
//        ellipse.display(I, vpColor::red);
//        vpDisplay::flush(I);
//      }
//    } catch (const vpException &e) {
//      std::cout << "Catch an exception: " << e << std::endl;
//    }













//      keypoint_detection.setDetector(detectorName);
//      keypoint_detection.setExtractor(extractorName);
//      keypoint_detection.setMatcher(matcherName);
//      keypoint_detection.setFilterMatchingType(vpKeyPoint::ratioDistanceThreshold);
//      keypoint_detection.setMatchingRatioThreshold(0.8);
//      keypoint_detection.setUseRansacVVS(true);
//      keypoint_detection.setUseRansacConsensusPercentage(true);
//      keypoint_detection.setRansacConsensusPercentage(20.0);
//      keypoint_detection.setRansacIteration(200);
//      keypoint_detection.setRansacThreshold(0.005);

//vpMe me;
//me.setMaskSize(5);
//me.setMaskNumber(180);
//me.setRange(8);
//me.setThreshold(10000);
//me.setMu1(0.5);
//me.setMu2(0.5);
//me.setSampleStep(4);
//me.setNbTotalSample(250);
//tracker.setMovingEdge(me);
//cam.initPersProjWithoutDistortion(257.34082279179285, 257.34082279179285, 160, 120);
//tracker.setCameraParameters(cam);
//tracker.setAngleAppear(vpMath::rad(70));
//tracker.setAngleDisappear(vpMath::rad(80));
//tracker.setNearClippingDistance(0.1);
//tracker.setFarClippingDistance(100.0);
//tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
