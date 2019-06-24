#include "header/missionManagerVision.h"


/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 * @todo racconta del init by click, che lo setti perfetto tanto è facile cliccare sui 4 angoli zoomando con il
 * display di opencv. E cmq è da fare solo una volta, poi salva i file .pos
 * @todo color images considerations?
 */
int main(int argc, char** argv){

  if (argc < 2){
    std::cout << "[MISSION_MANAGER] Please insert robot name for Vision"<< std::endl;
    return -1;
  }

  /// path of source to find images and config files for vision  //TODO REMOVE
  boost::filesystem::path path(__FILE__);
  path.remove_filename();
  std::string sourcePath = path.string();

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
  std::string cameraRangeRName = "bowtechRangeR_C";
  std::string holeName = "hole";
  WorldInterface worldInterface(robotName);
  worldInterface.waitReady(holeName);

  VisionInterfaceVision visionInterface(nh, robotName);


  /// Set initial state
  robotVisInterface.getwTv(&(robVisInfo.robotState.wTv_eigen));

  worldInterface.getT(&robVisInfo.robotStruct.vTcameraL, robotName, cameraLName);
  worldInterface.getT(&robVisInfo.robotStruct.vTcameraR, robotName, cameraRName);
  worldInterface.getT(&robVisInfo.robotStruct.cLTcR, cameraLName, cameraRName);
  worldInterface.getT(&robVisInfo.robotStruct.cRTcL, cameraRName, cameraLName);
  worldInterface.getT(&robVisInfo.robotStruct.vTcameraRangeR, robotName, cameraRangeRName);
  worldInterface.getT(&robVisInfo.robotStruct.cRangeRTcL, cameraRangeRName, cameraLName);

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

  bool stereo = true;
  bool initByClick = true;
  bool useDepth = true;


  /// Initial images
  vpImage<unsigned char> imageL_vp, imageR_vp, imageRangeR_vp;
  cv::Mat imageL_cv, imageR_cv, imageRangeR_cv;

  robotVisInterface.getLeftImage(&imageL_cv);
  robotVisInterface.getRightImage(&imageR_cv);
  imageL_cv.convertTo(imageL_cv, CV_8U); //TODO check if necessary convert in 8U
  imageR_cv.convertTo(imageR_cv, CV_8U); //TODO check if necessary convert in 8U

  vpImageConvert::convert(imageL_cv, imageL_vp);
  vpImageConvert::convert(imageR_cv, imageR_vp);

  //DEPTH camera
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (useDepth){

    robotVisInterface.getRangeRightImage(&imageRangeR_cv);
    imageRangeR_cv.convertTo(imageRangeR_cv, CV_16UC1);
    // convert function of visp dont convert from cv to uint16_t visp matrix)
    vpImage<uint16_t> imageRangeR_vp_raw;
    imageRangeR_vp_raw.resize(imageRangeR_cv.rows, imageRangeR_cv.cols);
    for (int i=0; i<imageRangeR_cv.rows; i++){
      for (int j=0; j<imageRangeR_cv.cols; j++){
        imageRangeR_vp_raw[i][j] = imageRangeR_cv.at<unsigned short>(i,j);
      }
    }
    //DCAM::makePointCloud(imageRangeR_vp_raw, pointcloud);
    vpImageConvert::createDepthHistogram(imageRangeR_vp_raw, imageRangeR_vp);

    DCAM::makePointCloud(imageRangeR_cv, pointcloud);
  }


  /// display things
  vpDisplayOpenCV display_left;
  vpDisplayOpenCV display_right;
  vpDisplayOpenCV display_rangeRight;
  display_left.setDownScalingFactor(vpDisplay::SCALE_AUTO);
  display_right.setDownScalingFactor(vpDisplay::SCALE_AUTO);
  display_rangeRight.setDownScalingFactor(vpDisplay::SCALE_AUTO);
  display_left.init(imageL_vp, 100, 100, "Model-based tracker (Left)");
  display_right.init(imageR_vp, 110 + (int)imageL_vp.getWidth(), 100,
                     "Model-based tracker (Right)");
  if (useDepth){
    display_rangeRight.init(imageRangeR_vp, 120+(int)imageL_vp.getWidth()+(int)imageR_vp.getWidth(),
                            100, "Model-based tracker (Range Right)");
    vpDisplay::display(imageRangeR_vp);
    vpDisplay::flush(imageRangeR_vp);
  }

  /// CREATE TRACKERS
  std::vector<std::string> cameraNames(2); //for config files
  cameraNames.at(0) = "left";
  if (!useDepth){
    cameraNames.at(1) = "right";
  } else {
    cameraNames.at(1) = "rangeRight";
  }

  MonoTracker* monoTrackerL;
  MonoTracker* monoTrackerR;
  StereoTracker* stereoTracker;
  std::map<std::string, vpHomogeneousMatrix> mapCameraTransf;
  // note: here it must be putted the transf from RIGTH to LEFT and not viceversa
  mapCameraTransf[cameraNames.at(0)] = vpHomogeneousMatrix();  //identity

  if (!useDepth){
    mapCameraTransf[cameraNames.at(1)] =
      CONV::transfMatrix_eigen2visp(robVisInfo.robotStruct.cRTcL);
  } else {
    mapCameraTransf[cameraNames.at(1)] =
        CONV::transfMatrix_eigen2visp(robVisInfo.robotStruct.cRangeRTcL);
  }

  std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
  mapOfImages[cameraNames.at(0)] = &imageL_vp;
  if (!useDepth){
    mapOfImages[cameraNames.at(1)] = &imageR_vp;
  } else {
    mapOfImages[cameraNames.at(1)] = &imageRangeR_vp;
  }

  // initialize trasformation matrix from camera to object
  std::map<std::string, vpHomogeneousMatrix> mapOfcameraToObj;
  mapOfcameraToObj[cameraNames.at(0)] = vpHomogeneousMatrix();
  mapOfcameraToObj[cameraNames.at(1)] = vpHomogeneousMatrix();

  if (stereo==false){/// Mono
    monoTrackerL = new MonoTracker(robotName, cameraNames.at(0),
                     vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
    monoTrackerR = new MonoTracker(robotName, cameraNames.at(1),
                     vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);

  } else {/// Stereo
    std::vector<int> trackerTypes;
    trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
    if (!useDepth){
      trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
    } else {
      trackerTypes.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER);
    }

    stereoTracker = new StereoTracker(robotName, cameraNames, mapCameraTransf, trackerTypes);
  }

  /// INIT TRACKERS

  std::vector<std::vector<cv::Point>> found4CornersVectorL, found4CornersVectorR; //used if initclick false
  if (initByClick){
    if (stereo == false){
      monoTrackerL->initTrackingByClick(&imageL_vp);
      monoTrackerR->initTrackingByClick(&imageR_vp);
    } else {
      stereoTracker->initTrackingByClick(mapOfImages);
    }


  } else { //find point without click //TODO, parla anche di altri metodi provati

    if (useDepth){
      Detector::findSquare(imageL_cv, &found4CornersVectorL);
      Detector::drawSquares(imageL_cv, found4CornersVectorL, "left");

      append2Dto3Dfile(found4CornersVectorL.at(0), sourcePath);
      stereoTracker->initTrackingByPoint(mapOfImages);

    }

    /// FIND SQUARE METHOD
      // https://docs.opencv.org/3.4/db/d00/samples_2cpp_2squares_8cpp-example.html#a20
    Detector::findSquare(imageL_cv, &found4CornersVectorL);
    Detector::findSquare(imageR_cv, &found4CornersVectorR);
    Detector::drawSquares(imageL_cv, found4CornersVectorL, "left");
    Detector::drawSquares(imageR_cv, found4CornersVectorR, "right");

    /// TEMPLATE MATCHING OPENCV
     // https://docs.opencv.org/3.4.6/de/da9/tutorial_template_matching.html
    //TODO usare diversi template da diverse angolazioni...
    // scrivi che template va anche bene però se simoa storti rispetto
    // al hole non riusciamo a fare un buon contorno perchè la regione visibile
    // non è un quadrato giusto e non possiamo sapere le dimensioni dei lati
    // in pixel nella immagine che si vede dalle camere... ci vorrebbe homography
    // per rectify immagini...
    //TODO rectify?
//    cv::Mat templL1 = cv::imread(sourcePath + templName);
//    cv::Mat templR1 = cv::imread(sourcePath + templName);
//    found4CornersVectorL.resize(1);
//    found4CornersVectorR.resize(1);
//    Detector::templateMatching(imageL_cv, templL1, &(found4CornersVectorL.at(0)));
//    Detector::templateMatching(imageR_cv, templR1, &(found4CornersVectorR.at(0)));


    //TODO  if found4CornersVector contain more than one element, pick the best...
    append2Dto3Dfile(found4CornersVectorL.at(0), found4CornersVectorR.at(0), sourcePath);

    if (stereo == false){
      monoTrackerL->initTrackingByPoint(&imageL_vp);
      monoTrackerR->initTrackingByPoint(&imageR_vp);

    }else {
      stereoTracker->initTrackingByPoint(mapOfImages);

    }
  } // END INIT


/** *********************************************************************************************************+*********
                                   MAIN VISION LOOP
*******************************************************************************************************************/

  //get camera param for display purpose
  vpCameraParameters cam_left, cam_right;
  if(stereo == false){
    monoTrackerL->getCameraParams(&cam_left);
    monoTrackerR->getCameraParams(&cam_right);

  } else {
    std::map<std::string, vpCameraParameters> mapOfCamParams;
    stereoTracker->getCamerasParams(&mapOfCamParams);
    cam_left = mapOfCamParams.at(cameraNames.at(0));
    cam_right = mapOfCamParams.at(cameraNames.at(1));
  }

  std::cout << "[" << robotName << "][MISSION_MANAGER] Starting loop" << std::endl;
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

    if (useDepth){
      robotVisInterface.getRangeRightImage(&imageRangeR_cv);
      imageRangeR_cv.convertTo(imageRangeR_cv, CV_16UC1);


      // convert function of visp dont convert from cv to uint16_t visp matrix)
      vpImage<uint16_t> imageRangeR_vp_raw;
      imageRangeR_vp_raw.resize(imageRangeR_cv.rows, imageRangeR_cv.cols);
      for (int i=0; i<imageRangeR_cv.rows; i++){
        for (int j=0; j<imageRangeR_cv.cols; j++){
          imageRangeR_vp_raw[i][j] = imageRangeR_cv.at<unsigned short>(i,j);
        }
      }
      //DCAM::makePointCloud(imageRangeR_vp_raw, pointcloud);
      vpImageConvert::createDepthHistogram(imageRangeR_vp_raw, imageRangeR_vp);

      DCAM::makePointCloud(imageRangeR_cv, pointcloud);
      vpDisplay::display(imageRangeR_vp);
    }

    vpHomogeneousMatrix cLThole, cRThole;
    if (stereo == false){/// METHOD 1 MONO CAMERAS

      double ransacErrorL = 0.0;
      double elapsedTimeL = 0.0;
      double ransacErrorR = 0.0;
      double elapsedTimeR = 0.0;

      monoTrackerL->monoTrack(&imageL_vp, &cLThole, &ransacErrorL, &elapsedTimeL);
      monoTrackerR->monoTrack(&imageR_vp, &cRThole, &ransacErrorR, &elapsedTimeR);

      monoTrackerL->display(&imageL_vp);
      monoTrackerR->display(&imageR_vp);


    } else{    /// METHOD 2 STEREO

      //update map of img
      mapOfImages.at(cameraNames.at(0)) = &imageL_vp;

      if (!useDepth){
        mapOfImages.at(cameraNames.at(1)) = &imageR_vp;
        stereoTracker->stereoTrack(mapOfImages, &mapOfcameraToObj);
      } else {

        // the image for the second (range) camera is not needed. Instead we
        // provide the pointcloud
        std::map<std::string, pcl::PointCloud< pcl::PointXYZ >::ConstPtr> mapOfPointclouds;
        mapOfPointclouds[cameraNames.at(1)] = pointcloud;
        stereoTracker->stereoTrack(mapOfImages, mapOfPointclouds, &mapOfcameraToObj);
      }
      cLThole = mapOfcameraToObj.at(cameraNames.at(0));

      /// TODO if range used, I think that transformation is not provided for the range camera
      /// (having two different transformation is need only for monotracker?)
      cRThole = mapOfcameraToObj.at(cameraNames.at(1));

      stereoTracker->display(mapOfImages);
    }

    /// method TRACKING DETECTION VISP ***********************************************************
                    ///TODO
    /// method TRACKING DETECTION VISP ***********************************************************


    vpDisplay::displayFrame(imageL_vp, cLThole, cam_left, 0.25, vpColor::none, 2);
    if (!useDepth){
      vpDisplay::displayFrame(imageR_vp, cRThole, cam_right, 0.25, vpColor::none, 2);
    } else {
      vpDisplay::displayFrame(imageRangeR_vp, cRThole, cam_right, 0.25, vpColor::none, 2);
    }

    // store estimation in struct, TODO mean tra right e left for monoTracker?
    Eigen::Matrix4d wTh_estimated_left =
        robVisInfo.robotState.wTv_eigen *
        robVisInfo.robotStruct.vTcameraL *
        CONV::matrix_visp2eigen(cLThole);
    Eigen::Matrix4d wTh_estimated_right =
        robVisInfo.robotState.wTv_eigen *
        robVisInfo.robotStruct.vTcameraR *
        CONV::matrix_visp2eigen(cRThole);



    robVisInfo.transforms.wTh_estimated_eigen = wTh_estimated_left;

    ///debug
    std::cout << "PUBLISHING:\n" << robVisInfo.transforms.wTh_estimated_eigen <<"\n";
    visionInterface.publishHoleTransform(robVisInfo.transforms.wTh_estimated_eigen);


    vpDisplay::displayText(imageL_vp, 30, 10, "A click to exit.", vpColor::red);
    vpDisplay::flush(imageL_vp);
    vpDisplay::flush(imageR_vp);
    if (useDepth){
      vpDisplay::flush(imageRangeR_vp);
    }
    //vpDisplay::getKeyboardEvent(imageL_vp, 'd');
    if (vpDisplay::getClick(imageL_vp, false)) {
      return 1;
    }


    if (pathLog.size() != 0){
      worldInterface.getwT(&(robVisInfo.transforms.wTh_eigen), holeName);
      CMAT::TransfMatrix wThole_cmat =
          CONV::matrix_eigen2cmat(robVisInfo.transforms.wTh_eigen);

      if (stereo == false){

        logger.logCartError(robVisInfo.transforms.wTh_eigen,
                               wTh_estimated_left, "errorMonoL");
        logger.logCartError(robVisInfo.transforms.wTh_eigen,
                               wTh_estimated_right, "errorMonoR");

      } else{

        logger.logCartError(robVisInfo.transforms.wTh_eigen,
                            wTh_estimated_left, "errorStereoL");
        /// TODO check if error l and r are same, they must are when stereo method is used
        logger.logCartError(robVisInfo.transforms.wTh_eigen,
                            wTh_estimated_right, "errorStereoR");
      }
    } //END LOGGING


    ros::spinOnce();
    loop_rate.sleep();
  }


  if (stereo == false){
    delete monoTrackerL;
    delete monoTrackerR;
  } else {
    delete stereoTracker;

  }

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
 * @warning Visp want switched x and y coord for the 2D point. SO this funtion print point as y, x coord
 */
void append2Dto3Dfile(std::vector<cv::Point> found4CornersL, std::vector<cv::Point> found4CornersR,
                      std::string sourcePath){

  // copy original files, if destination exist, it will be overwrite (that is what we want)
  std::ifstream  srcL(sourcePath+"/vision/"+initFileClick+"left.init", std::ios::binary);
  std::ofstream  dstL(sourcePath+"/vision/"+initFile_w2D+"left.init",  std::ios::binary);
  dstL << srcL.rdbuf();
  srcL.close();

  std::ifstream  srcR(sourcePath+"/vision/"+initFileClick+"right.init", std::ios::binary);
  std::ofstream  dstR(sourcePath+"/vision/"+initFile_w2D+"right.init",   std::ios::binary);
  dstR << srcR.rdbuf();
  srcR.close();

  //write on the new file
  dstL << "#Generate by code: append 2D points" << std::endl
       << "4" << std::endl // number of points
       << found4CornersL.at(0).y << " " << found4CornersL.at(0).x << std::endl
       << found4CornersL.at(1).y << " " << found4CornersL.at(1).x << std::endl
       << found4CornersL.at(2).y << " " << found4CornersL.at(2).x << std::endl
       << found4CornersL.at(3).y << " " << found4CornersL.at(3).x << std::endl ;
  dstL.close();
  dstR << "#Generate by code: append 2D points" << std::endl
       << "4" << std::endl // number of points
       << found4CornersR.at(0).y << " " << found4CornersR.at(0).x << std::endl
       << found4CornersR.at(1).y << " " << found4CornersR.at(1).x << std::endl
       << found4CornersR.at(2).y << " " << found4CornersR.at(2).x << std::endl
       << found4CornersR.at(3).y << " " << found4CornersR.at(3).x << std::endl ;
  dstR.close();

  return;
}

/**
 * @brief append2Dto3Dfile function to write 2D point in the file where corresponded 3D point are
 *    version for only left image, used by depth camera method
 * @param found4CornersL the 4 point in the left image
 * @param sourcePath the path of the .cpp file
 * @note there isn't a visp function that permits to init using file for
 *       3D points and vector for 2D points. So we have to write (append) 2D points
 *       in the file of 3D points and use
 *       void vpMbGenericTracker::initFromPoints( const vpImage< unsigned char > &  	I1,
 *                                                 const vpImage< unsigned char > &  	I2,
 *                                               const std::string &  	initFile1,
 *                                               const std::string &  	initFile2 )
 * @warning Visp want switched x and y coord for the 2D point. SO this funtion print point as y, x coord
 */
void append2Dto3Dfile(std::vector<cv::Point> found4CornersL, std::string sourcePath){

  // copy original files, if destination exist, it will be overwrite (that is what we want)
  std::ifstream  srcL(sourcePath+"/vision/"+initFileClick+"left.init", std::ios::binary);
  std::ofstream  dstL(sourcePath+"/vision/"+initFile_w2D+"left.init",  std::ios::binary);
  dstL << srcL.rdbuf();
  srcL.close();

  //write on the new file
  dstL << "#Generate by code: append 2D points" << std::endl
       << "4" << std::endl // number of points
       << found4CornersL.at(0).y << " " << found4CornersL.at(0).x << std::endl
       << found4CornersL.at(1).y << " " << found4CornersL.at(1).x << std::endl
       << found4CornersL.at(2).y << " " << found4CornersL.at(2).x << std::endl
       << found4CornersL.at(3).y << " " << found4CornersL.at(3).x << std::endl ;
  dstL.close();

  return;
}



/**
  Axis in the model are:
  x going out of the plane
  y pointing on the right
  z go up in the plane
  */
