#include <iostream>
#include "rosInterfaces/header/robotVisionInterface.h"

#include <visp3/gui/vpDisplayX.h> //recommended for linux from docuentation

#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpKeyPoint.h>

#include <visp3/mbt/vpMbGenericTracker.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>


//cv_bridge::CvImagePtr imageCVL;
//cv_bridge::CvImagePtr imageCVR;


//void subCallbackL(const sensor_msgs::ImageConstPtr& msg)
//{
//   imageCVL = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
//}
//void subCallbackR(const sensor_msgs::ImageConstPtr& msg)
//{

//   imageCVR = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
//}




int main(int argc, char** argv){


  /// ROS SUB to cameras
  ros::init(argc, argv, "Vision2");
  ros::NodeHandle nh;
  //ros::Subscriber subL = nh.subscribe("/uwsim/g500_C/cameraL", 1, subCallbackL);
  //ros::Subscriber subR = nh.subscribe("/uwsim/g500_C/cameraR", 1, subCallbackR);
  //ros::spinOnce();
  RobotVisionInterface robVisInterface(nh, "g500_C");
  robVisInterface.init();


    /// KEYPOINT DETECTOR
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








    /// TRACKING 3D model generic tracker
    ///
    ros::Rate loop_rate(10);

    try {
    vpImage<unsigned char> I_left, I_right;
    vpDisplayX display_left;
    vpDisplayX display_right;

    cv::Mat cvIL, cvIR;
    robVisInterface.getLeftImage(&cvIL);
    robVisInterface.getRightImage(&cvIR);
    cvIL.convertTo(cvIL, CV_8U); //TODO check if necessary convert in 8U
    cvIR.convertTo(cvIR, CV_8U);

    vpImageConvert::convert(cvIL, I_left);
    vpImageConvert::convert(cvIR, I_right);



    display_left.setDownScalingFactor(vpDisplay::SCALE_AUTO);
    display_right.setDownScalingFactor(vpDisplay::SCALE_AUTO);

    display_left.init(I_left, 100, 100, "Model-based tracker (Left)");
    display_right.init(I_right, 110 + (int)I_left.getWidth(), 100, "Model-based tracker (Right)");


    int opt_tracker1 = vpMbGenericTracker::KLT_TRACKER;
    int opt_tracker2 = vpMbGenericTracker::KLT_TRACKER;
    std::vector<int> trackerTypes(2);
    trackerTypes[0] = opt_tracker1;
    trackerTypes[1] = opt_tracker2;
    vpMbGenericTracker tracker(trackerTypes);

    tracker.loadConfigFile("/home/tori/UWsim/Peg/src/peg/src/cameraL.xml", "/home/tori/UWsim/Peg/src/peg/src/cameraL.xml");

    tracker.loadModel("/home/tori/UWsim/Peg/src/peg/src/blockHole.cao", "/home/tori/UWsim/Peg/src/peg/src/blockHole.cao");
    tracker.setDisplayFeatures(true);

    vpHomogeneousMatrix cRightMcLeft;
    std::ifstream file_cRightMcLeft("/home/tori/UWsim/Peg/src/peg/src/lTr.txt");
    cRightMcLeft.load(file_cRightMcLeft);

    std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformationMatrix;
    mapOfCameraTransformationMatrix["Camera1"] = vpHomogeneousMatrix(); //identity
    mapOfCameraTransformationMatrix["Camera2"] = cRightMcLeft;

    tracker.setCameraTransformationMatrix(mapOfCameraTransformationMatrix);

    //click in order: top right, top left, bottom right, bottom left
    tracker.initClick(I_left, I_right, "/home/tori/UWsim/Peg/src/peg/src/3DPointSquareFace4.init", "/home/tori/UWsim/Peg/src/peg/src/3DPointSquareFace4.init", false);

    while (ros::ok()) {

       //(src, dest)
      cv::Mat cvIL, cvIR;
      robVisInterface.getLeftImage(&cvIL);
      robVisInterface.getRightImage(&cvIR);
      cvIL.convertTo(cvIL, CV_8U);
      cvIR.convertTo(cvIR, CV_8U);

      vpImageConvert::convert(cvIL, I_left);
      vpImageConvert::convert(cvIR, I_right);

      vpDisplay::display(I_left);
      vpDisplay::display(I_right);


      tracker.track(I_left, I_right);

      vpHomogeneousMatrix cLeftMo, cRightMo;
      tracker.getPose(cLeftMo, cRightMo);


      vpCameraParameters cam_left, cam_right;
      tracker.getCameraParameters(cam_left, cam_right);
      tracker.display(I_left, I_right, cLeftMo, cRightMo, cam_left, cam_right, vpColor::red, 2);
      vpDisplay::displayFrame(I_left, cLeftMo, cam_left, 0.25, vpColor::none, 2);
      vpDisplay::displayFrame(I_right, cRightMo, cam_right, 0.25, vpColor::none, 2);
      vpDisplay::displayText(I_left, 10, 10, "A click to exit...", vpColor::red);
      vpDisplay::flush(I_left);
      vpDisplay::flush(I_right);
      if (vpDisplay::getClick(I_left, false)) {
        break;
      }
      ros::spinOnce();
    }
    vpDisplay::getClick(I_left);
    } catch (const vpException &e) {
      std::cerr << "Catch a ViSP exception: " << e.what() << std::endl;



    }


   ///KEYPOINT TRACKER
   /// IT WORKS!

//  ros::Rate loop_rate(10);
//  while (ros::ok()) {
//      if (imageCVL == NULL){
//        std::cout << "no imageL ancora\n";
//        ros::spinOnce();
//        loop_rate.sleep();
//      } else{
//          break;
//      }
//  }

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


  ///OBJECT DETECTIONN
//    ros::Rate loop_rate(10);
//    while (ros::ok()) {
//      if (imageCVL == NULL){
//        std::cout << "no imageL ancora\n";
//        ros::spinOnce();
//        loop_rate.sleep();
//      } else{
//        break;
//      }
//    }
//   try {

//    vpImage<unsigned char> I;
//    vpCameraParameters cam;
//    vpHomogeneousMatrix cMo;

//    cv::Mat cvI = imageCVL.get()->image;
//    cvI.convertTo(cvI, CV_8U);
//    vpImageConvert::convert(cvI, I);

//    vpDisplayX display;

//    display.init(I, 100, 100, "Model-based edge tracker");
//    vpMbEdgeTracker tracker;
//    bool usexml = true;

//    if (usexml) {
//      tracker.loadConfigFile("cameraL.xml");
//      tracker.getCameraParameters(cam);

//    } else {
//      vpMe me;
//      me.setMaskSize(5);
//      me.setMaskNumber(180);
//      me.setRange(8);
//      me.setThreshold(10000);
//      me.setMu1(0.5);
//      me.setMu2(0.5);
//      me.setSampleStep(4);
//      me.setNbTotalSample(250);
//      tracker.setMovingEdge(me);
//      cam.initPersProjWithoutDistortion(257.34082279179285, 257.34082279179285, 160, 120);
//      tracker.setCameraParameters(cam);
//      tracker.setAngleAppear(vpMath::rad(70));
//      tracker.setAngleDisappear(vpMath::rad(80));
//      tracker.setNearClippingDistance(0.1);
//      tracker.setFarClippingDistance(100.0);
//      tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
//    }
//    tracker.setOgreVisibilityTest(false);
//    tracker.loadModel("blockHole.cao");

//    tracker.setDisplayFeatures(true);
//    tracker.initClick(I, "3DPointSquareFace.init", true);
//    tracker.track(I);

//    std::string detectorName = "SIFT";
//    std::string extractorName = "SIFT";
//    std::string matcherName = "BruteForce";
//    std::string configurationFile = "detection-config-SIFT.xml";


//    vpKeyPoint keypoint_learning;
//    if (usexml) {
//      keypoint_learning.loadConfigFile(configurationFile);

//    } else {
//      keypoint_learning.setDetector(detectorName);
//      keypoint_learning.setExtractor(extractorName);
//      keypoint_learning.setMatcher(matcherName);
//    }
//    std::vector<cv::KeyPoint> trainKeyPoints;
//    double elapsedTime;
//    keypoint_learning.detect(I, trainKeyPoints, elapsedTime);
//    std::vector<vpPolygon> polygons;
//    std::vector<std::vector<vpPoint> > roisPt;
//    std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces(false);
//    polygons = pair.first;
//    roisPt = pair.second;
//    std::vector<cv::Point3f> points3f;
//    tracker.getPose(cMo);
//    vpKeyPoint::compute3DForPointsInPolygons(cMo, cam, trainKeyPoints, polygons, roisPt, points3f);
//    keypoint_learning.buildReference(I, trainKeyPoints, points3f);
//    keypoint_learning.saveLearningData("blockHole_learning_data.bin", true);
//    vpDisplay::display(I);
//    for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
//      vpDisplay::displayCross(I, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
//    }
//    vpDisplay::displayText(I, 10, 10, "Learning step: keypoints are detected on visible teabox faces", vpColor::red);
//    vpDisplay::displayText(I, 30, 10, "Click to continue with detection...", vpColor::red);
//    vpDisplay::flush(I);
//    vpDisplay::getClick(I, true);
//    vpKeyPoint keypoint_detection;
//    if (usexml) {
//      keypoint_detection.loadConfigFile(configurationFile);
//    } else {
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
//    }
//    keypoint_detection.loadLearningData("blockHole_learning_data.bin", true);
//    double error;
//    bool click_done = false;

//    while (ros::ok()) {
//      cv::Mat cvI = imageCVL.get()->image;
//      cvI.convertTo(cvI, CV_8U);
//      vpImageConvert::convert(cvI, I);
//      vpDisplay::display(I);
//      vpDisplay::displayText(I, 10, 10, "Detection and localization in process...", vpColor::red);
//      if (keypoint_detection.matchPoint(I, cam, cMo, error, elapsedTime)) {
//        tracker.setPose(I, cMo);
//        tracker.display(I, cMo, cam, vpColor::red, 2);
//        vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
//      }
//      vpDisplay::displayText(I, 30, 10, "A click to exit.", vpColor::red);
//      vpDisplay::flush(I);
//      if (vpDisplay::getClick(I, false)) {
//        click_done = true;
//        break;
//      }

//      vpMbHiddenFaces<vpMbtPolygon> &faces = tracker.getFaces();
//      std::cout << "Number of faces: " << faces.size() << std::endl;
//      for (unsigned int i=0; i < faces.size(); i++) {
//        std::vector<vpMbtPolygon*> &poly = faces.getPolygon();
//        std::cout << "face " << i << " with index: " << poly[i]->getIndex()
//            << (poly[i]->getName().empty() ? "" : (" with name: " + poly[i]->getName()))
//            << " is " << (poly[i]->isVisible() ? "visible" : "not visible")
//            << " and has " << poly[i]->getNbPoint() << " points"
//            << " and LOD is " << (poly[i]->useLod ? "enabled" : "disabled") << std::endl;

//        for (unsigned int j=0; j<poly[i]->getNbPoint(); j++) {
//          vpPoint P = poly[i]->getPoint(j);
//          P.project(cMo);
//          std::cout << " P obj " << j << ": " << P.get_oX() << " " << P.get_oY() << " " << P.get_oZ() << std::endl;
//          std::cout << " P cam" << j << ": " << P.get_X() << " " << P.get_Y() << " " << P.get_Z() << std::endl;
//          vpImagePoint iP;
//          vpMeterPixelConversion::convertPoint(cam, P.get_x(), P.get_y(), iP);
//          std::cout << " iP " << j << ": " << iP.get_u() << " " << iP.get_v() << std::endl;
//        }
//      }

//      ros::spinOnce();
//      loop_rate.sleep();
//    }
//    if (!click_done)
//      vpDisplay::getClick(I);

//    vpXmlParser::cleanup();

//  } catch (vpException &e) {
//    std::cout << "Catch an exception: " << e << std::endl;
//  }




    // segui object detection tutorial

    return 0;

}


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
