#include "header/missionManagerVision.h"


/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 * @todo racconta del init by click, che lo setti perfetto tanto è facile cliccare sui 4 angoli zoomando con il
 * display di opencv. E cmq è da fare solo una volta, poi salva i file .pos
 */
int main(int argc, char** argv){

  if (argc < 2){
    std::cout << "[MISSION_MANAGER] Please insert robot name for Vision"<< std::endl;
    return -1;
  }

  /// path of source to find images and config files for vision  //TODO REMOVE
  boost::filesystem::path sourcePath(__FILE__);
  sourcePath.remove_filename();

  std::string robotName = argv[1];
  std::string pathLog;
  if (LOG && (argc > 2)){   //if flag log setted to 1 and path log is given
    pathLog = argv[2];
  }

  /// ROS NODE
  ros::init(argc, argv, robotName + "_MissionManagerVision");
  std::cout << "[" << robotName << "][MISSION_MANAGER] Start" << std::endl;
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
  WorldInterface worldInterface(robotName);
  worldInterface.waitReady(holeName);


  /// Set initial state
  robotVisInterface.getwTv(&(robVisInfo.robotState.wTv_eigen));

  worldInterface.getT(&robVisInfo.robotStruct.vTcameraL, robotName, cameraLName);
  worldInterface.getT(&robVisInfo.robotStruct.vTcameraR, robotName, cameraRName);
  worldInterface.getT(&robVisInfo.robotStruct.cLTcR, cameraLName, cameraRName);
  worldInterface.getT(&robVisInfo.robotStruct.cRTcL, cameraRName, cameraLName);

  // real hole pose to log errors of pose estimation
  worldInterface.getwT(&(robVisInfo.transforms.wTh_eigen), holeName);




/** ***************************************************************************************************************
                                 LOGGING
********************************************************************************************************************/

  Logger logger;
  if (pathLog.size() > 0){
    logger = Logger(robotName, pathLog);
    logger.createDirectoryForNode();
  }



/** *********************************************************************************************************+*********
                                  INIT VISION
*******************************************************************************************************************/

  /// Initial images
  vpImage<unsigned char> imageL_vp, imageR_vp;
  cv::Mat imageL_cv, imageR_cv;

  robotVisInterface.getLeftImage(&imageL_cv);
  robotVisInterface.getRightImage(&imageR_cv);
  imageL_cv.convertTo(imageL_cv, CV_8U); //TODO check if necessary convert in 8U
  imageR_cv.convertTo(imageR_cv, CV_8U); //TODO check if necessary convert in 8U
  //cut top part of image where a piece of auv is visible and can distract cv algos
  imageL_cv = imageL_cv(cv::Rect(0, 60, imageL_cv.cols, imageL_cv.rows-60));
  imageR_cv = imageR_cv(cv::Rect(0, 60, imageL_cv.cols, imageL_cv.rows-60));

  vpImageConvert::convert(imageL_cv, imageL_vp);
  vpImageConvert::convert(imageR_cv, imageR_vp);

  /// display things
  vpDisplayOpenCV display_left;
  vpDisplayOpenCV display_right;
  display_left.setDownScalingFactor(vpDisplay::SCALE_AUTO);
  display_right.setDownScalingFactor(vpDisplay::SCALE_AUTO);
  display_left.init(imageL_vp, 100, 100, "Model-based tracker (Left)");
  display_right.init(imageR_vp, 110 + (int)imageL_vp.getWidth(), 100,
                     "Model-based tracker (Right)");

  /// init trackers

  std::vector<std::string> cameraNames(2);
  cameraNames.at(0) = "left";
  cameraNames.at(1) = "right";

  /// Mono
  MonoTracker monoTrackerL(robotName, cameraNames.at(0),
          vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
  MonoTracker monoTrackerR(robotName, cameraNames.at(1),
          vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);

  /// Stereo
  std::map<std::string, vpHomogeneousMatrix> mapCameraTransf;
  // note: here it must be putted the transf from RIGTH to LEFT and not viceversa
  mapCameraTransf.insert({cameraNames.at(0), vpHomogeneousMatrix()});  //identity
  mapCameraTransf.insert({cameraNames.at(1), robVisInfo.robotStruct.cRTcL});

  std::map<std::string, const vpImage<unsigned char>*> mapOfImages;
  mapOfImages.insert({cameraNames.at(0), &imageL_vp});
  mapOfImages.insert({cameraNames.at(1), &imageR_vp});

  std::map<std::string, vpHomogeneousMatrix> mapOfcameraToObj;
  mapOfcameraToObj.insert({cameraNames.at(0), vpHomogeneousMatrix()});
  mapOfcameraToObj.insert({cameraNames.at(1), vpHomogeneousMatrix()});

  StereoTracker stereoTracker(robotName, cameraNames, mapCameraTransf,
                vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);


  bool initByClick = false;
  std::vector<std::vector<cv::Point>> found4CornersVectorL, found4CornersVectorR; //used if initclick false
  if (initByClick){
      monoTrackerL.initTrackingByClick(imageL_vp);
      monoTrackerR.initTrackingByClick(imageR_vp);


      stereoTracker.initTrackingByClick(mapOfImages);


  } else { //find point without click //TODO, parla anche di altri metodi provati

    /// FIND SQUARE METHOD
      // https://docs.opencv.org/3.4/db/d00/samples_2cpp_2squares_8cpp-example.html#a20
    Detector::findSquare(imageL_cv, &found4CornersVectorL);
    Detector::findSquare(imageR_cv, &found4CornersVectorR);

    /// TEMPLATE MATCHING OPENCV
     // https://docs.opencv.org/3.4.6/de/da9/tutorial_template_matching.html
    cv::Mat templL1 = cv::imread(sourcePath + templName);
    cv::Mat templR1 = cv::imread(sourcePath + templName);
    found4CornersVectorL.resize(1);
    found4CornersVectorR.resize(1);
    Detector::templateMatching(imageL_cv, templL1, &(found4CornersVectorL).at(0));
    Detector::templateMatching(imageR_cv, templR1, &(found4CornersVectorR).at(0));

    //TODO  if found4CornersVector contain more than one element, pick the best...
    append2Dto3Dfile(found4CornersVectorL.at(0), found4CornersVectorR.at(0), sourcePath);

    monoTrackerL.initTrackingByPoint(imageL_vp);
    monoTrackerR.initTrackingByPoint(imageR_vp);

    stereoTracker.initTrackingByPoint(mapOfImages);

  }


/** *********************************************************************************************************+*********
                                   MAIN VISION LOOP
*******************************************************************************************************************/
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    robotVisInterface.getwTv(&(robVisInfo.robotState.wTv_eigen));

    robotVisInterface.getLeftImage(&imageL_cv);
    robotVisInterface.getRightImage(&imageR_cv);
    imageL_cv.convertTo(imageL_cv, CV_8U); //TODO check if necessary convert in 8U
    imageR_cv.convertTo(imageR_cv, CV_8U); //TODO check if necessary convert in 8U
    vpImageConvert::convert(imageL_cv, imageL_vp);
    vpImageConvert::convert(imageR_cv, imageR_vp);

    vpDisplay::display(imageL_vp);
    vpDisplay::display(imageR_vp);

    //updats map of img, used for stereo methods
    mapOfImages.at(cameraNames.at(0)) =  &imageL_vp;
    mapOfImages.at(cameraNames.at(1)) = &imageR_vp;


    /// METHOD 1 OBJECT DETECTION
//    vpDisplay::displayText(imageL_vp, 10, 10, "Detection and localization in process...", vpColor::red);
//    vpHomogeneousMatrix cLThole;
//    double ransacError = 0.0;
//    double elapsedTime = 0.0;
//    objDetection(imageL_vp, &tracker, &keypoint_detection, &cLThole,
//                 &ransacError, &elapsedTime);

//    vpDisplay::displayFrame(imageL_vp, cLThole, cam_left, 0.25, vpColor::none, 3);


//    robVisInfo.transforms.wTh_estimated_eigen =
//        robVisInfo.robotState.wTv_eigen *
//        robVisInfo.robotStruct.vTcameraL *
//        CONV::matrix_visp2eigen(cLThole);




    /// method TRACKING DETECTION VISP ***********************************************************
                    ///TODO
    /// method TRACKING DETECTION VISP ***********************************************************


    /// METHOD 2 STEREO




    stereoTracker.stereoTrack(mapOfImages, &mapOfcameraToObj);
    vpHomogeneousMatrix cLThole_st = mapOfcameraToObj.at(cameraNames.at(0));
    vpHomogeneousMatrix cRThole_st = mapOfcameraToObj.at(cameraNames.at(1));

    // display
    vpCameraParameters cam_left, cam_right;
    StereoTracker.getCameraParameters(cam_left, cam_right);
    //tracker.display(imageL_vp, imageR_vp, cLThole_st, cRThole_st, cam_left, cam_right, vpColor::red, 2);
    vpDisplay::displayFrame(imageL_vp, cLThole_st, cam_left, 0.25, vpColor::none, 2);
    vpDisplay::displayFrame(imageR_vp, cRThole_st, cam_right, 0.25, vpColor::none, 2);


    Eigen::Matrix4d wTh_estimated_stereo_left =
        robVisInfo.robotState.wTv_eigen *
        robVisInfo.robotStruct.vTcameraL *
        CONV::matrix_visp2eigen(cLThole_st);

    Eigen::Matrix4d wTh_estimated_stereo_right =
        robVisInfo.robotState.wTv_eigen *
        robVisInfo.robotStruct.vTcameraR *
        CONV::matrix_visp2eigen(cRThole_st);


    vpDisplay::displayText(imageL_vp, 30, 10, "A click to exit.", vpColor::red);
    vpDisplay::flush(imageL_vp);
    vpDisplay::flush(imageR_vp);
    if (vpDisplay::getClick(imageL_vp, false)) {
      return 1;
    }


    if (pathLog.size() != 0){

      worldInterface.getwT(&(robVisInfo.transforms.wTh_eigen), holeName);
      CMAT::TransfMatrix wThole_cmat =
          CONV::matrix_eigen2cmat(robVisInfo.transforms.wTh_eigen);


      CMAT::TransfMatrix wTholeEstimated_stereoL_cmat =
          CONV::matrix_eigen2cmat(wTh_estimated_stereo_left);
      CMAT::TransfMatrix wTholeEstimated_stereoR_cmat =
          CONV::matrix_eigen2cmat(wTh_estimated_stereo_right);

      CMAT::Vect6 swappedError =
          CMAT::CartError(wThole_cmat, wTholeEstimated_stereoL_cmat);
      CMAT::Vect6 error;
      error.SetFirstVect3(swappedError.GetSecondVect3());
      error.SetSecondVect3(swappedError.GetFirstVect3());
      logger.writeCmatMatrix(error, "errorStereoL");


      /// TODO check if error l and r are same, they must are when stereo method is used
      CMAT::Vect6 swappedError3 =
          CMAT::CartError(wThole_cmat, wTholeEstimated_stereoR_cmat);
      CMAT::Vect6 error3;
      error3.SetFirstVect3(swappedError3.GetSecondVect3());
      error3.SetSecondVect3(swappedError3.GetFirstVect3());
      logger.writeCmatMatrix(error3, "errorStereoR");

    } //END LOGGING



    ros::spinOnce();
    loop_rate.sleep();
  }

   //To clean up memory allocated by the xml library,
   //the user has to call this before the exit().
   vpXmlParser::cleanup();
   return 0;
}




/**
 * @brief append2Dto3Dfile function to write 2D point in the file where corresponded 3D point are
 * @param found4CornersL the 4 point in the left image
 * @param found4CornersR the 4 point in the right image
 * @param sourcePath the path of the .cpp file
 * @note there isn't a visp function that permits to init using file for
 *       3D points and vector for 2D points. So we have to write (append) 2D points
 *       in the file of 3D points and use
 *       void vpMbGenericTracker::initFromPoints( const vpImage< unsigned char > &  	I1,
 *                                                 const vpImage< unsigned char > &  	I2,
 *                                               const std::string &  	initFile1,
 *                                               const std::string &  	initFile2 )
 */
void append2Dto3Dfile(std::vector<cv::Point> found4CornersL, std::vector<cv::Point> found4CornersR,
                      std::string sourcePath){

  // copy original files, if destination exist, it will be overwrite (that is what we want)
  std::ifstream  srcL(sourcePath+initFileClick+"left.init", std::ios::binary);
  std::ofstream  dstL(sourcePath+initFile_w2D+"left.init",  std::ios::binary);
  dstL << srcL.rdbuf();
  srcL.close();

  std::ifstream  srcR(sourcePath+initFileClick+"right.init", std::ios::binary);
  std::ofstream  dstR(sourcePath+initFileClick_w2D+"right.init",   std::ios::binary);
  dstR << srcR.rdbuf();
  srcR.close();

  //write on the new file
  dstL << std::endl << "#Generate by code: append 2D points" << std::endl
       << "4" << std::endl // number of points
       << found4CornersL.at(0).x << " " << found4CornersL.at(0).y << std::endl
       << found4CornersL.at(1).x << " " << found4CornersL.at(1).y << std::endl
       << found4CornersL.at(2).x << " " << found4CornersL.at(2).y << std::endl
       << found4CornersL.at(3).x << " " << found4CornersL.at(3).y << std::endl ;
  dstL.close();
  dstR << std::endl << "#Generate by code: append 2D points" << std::endl
       << "4" << std::endl // number of points
       << found4CornersR.at(0).x << " " << found4CornersR.at(0).y << std::endl
       << found4CornersR.at(1).x << " " << found4CornersR.at(1).y << std::endl
       << found4CornersR.at(2).x << " " << found4CornersR.at(2).y << std::endl
       << found4CornersR.at(3).x << " " << found4CornersR.at(3).y << std::endl ;
  dstR.close();

  return;
}



/**
  Axis in the model are:
  x going out of the plane
  y pointing on the right
  z go up in the plane
  */
