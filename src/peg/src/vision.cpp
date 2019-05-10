#include "header/vision.h"




void MatchingMethod(int match_method, cv::Mat img, cv::Mat templ,
                    cv::Point *bestMatch, double *minMaxVal);

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

  std::vector<int> trackerTypes;
//  trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER);
//  trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER);
//  trackerTypes.push_back(vpMbGenericTracker::KLT_TRACKER );
//  trackerTypes.push_back(vpMbGenericTracker::KLT_TRACKER );
  trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
  trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);


  vpMbGenericTracker tracker(trackerTypes);
  initTracker(&tracker, trackerTypes.size(), robVisInfo.robotStruct.cRTcL);

  vpDisplayOpenCV display_left;
  vpDisplayOpenCV display_right;
  display_left.setDownScalingFactor(vpDisplay::SCALE_AUTO);
  display_right.setDownScalingFactor(vpDisplay::SCALE_AUTO);
  display_left.init(imageL_vp, 100, 100, "Model-based tracker (Left)");
  display_right.init(imageR_vp, 110 + (int)imageL_vp.getWidth(), 100,
                     "Model-based tracker (Right)");

  bool initByClick = false;
  if (initByClick){ //init by click
    switch (trackerTypes.size()){
    case 1:
      tracker.initClick(imageL_vp, initFileClickLeft, true);
      break;
    case 2:
      tracker.initClick(imageL_vp, imageR_vp, initFileClickLeft, initFileClickRight, true);
      break;
    default:
      std::cerr << "WRONG NUMBER OF TRACKERS\n";
    }

  } else { //find point da solo COME??








//    ///cv corner detect TRY srivi che non è buono ***********************************************
//    // coor of rect are top-left corner
//    src = imageL_cv;
//    cv::namedWindow( source_window );
//    cv::imshow( source_window, src );
//    goodFeaturesToTrack_Demo( 0, 0 );
//    cv::waitKey(0);

    /// EDGE detect con canny e hough ****************************************************
//    cv::Mat dst, cdst, cdstP;
//    // Loads an image
//    cv::Mat src = imageL_cv;
//    // Check if image is loaded fine

//    // Edge detection
//    cv::Canny(src, dst, 50, 200, 3);
//    // Copy edges to the images that will display the results in BGR
//    cv::cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);
//    cdstP = cdst.clone();
//    // Standard Hough Line Transform
//    std::vector<cv::Vec2f> lines; // will hold the results of the detection
//    cv::HoughLines(dst, lines, 1, CV_PI/180, 60, 0, 0 ); // runs the actual detection
//    // Draw the lines
//    for( size_t i = 0; i < lines.size(); i++ )
//    {
//        float rho = lines[i][0], theta = lines[i][1];
//        cv::Point pt1, pt2;
//        double a = cos(theta), b = sin(theta);
//        double x0 = a*rho, y0 = b*rho;
//        pt1.x = cvRound(x0 + 1000*(-b));
//        pt1.y = cvRound(y0 + 1000*(a));
//        pt2.x = cvRound(x0 - 1000*(-b));
//        pt2.y = cvRound(y0 - 1000*(a));
//        cv::line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
//    }
//    // Probabilistic Line Transform
//    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
//    cv::HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
//    // Draw the lines
//    for( size_t i = 0; i < linesP.size(); i++ )
//    {
//        cv::Vec4i l = linesP[i];
//        cv::line( cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
//    }
//    // Show results
//    cv::imshow("Source", src);
//    cv::imshow("Standard Hough Line Transform", cdst);
//    cv::imshow("Probabilistic Line Transform", cdstP);
//    // Wait and Exit
//    cv::waitKey();
    /// EDGE detect con canny e hough ****************************************************


    /// BOUNDING BOX DETECTION ***********************************************************************************************
//    src = imageL_cv.clone();
//    //cv::blur( src, src, cv::Size(3,3) );
//    std::string source_window = "Source";

//    cv::namedWindow( source_window );
//    cv::imshow( source_window, src );
//    const int max_thresh = 255;
//    cv::createTrackbar( "Canny thresh:", source_window, &thresh, max_thresh, thresh_callback );
//    cv::createTrackbar("canny thres 2:", source_window, &thresh2, 500, thresh_callback );

//    cv::createTrackbar("poli thresh:", source_window, &thresh_poly, 30, thresh_callback );
//    thresh_callback( 0, 0 );
//    cv::waitKey(0);
    /// BOUNDING BOX DETECTION  https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html********************************************************************


    /// FIND SQUARE METHOD https://docs.opencv.org/3.4/db/d00/samples_2cpp_2squares_8cpp-example.html#a20
//    std::vector<std::vector<cv::Point> > squares, orderedSquares;
//    cv::Mat image;
//    cv::cvtColor(imageL_cv, image, cv::COLOR_GRAY2BGR); //actually not real conv in colors
//    findSquares(image, squares);
//    orderedSquares.resize(squares.size());
//    orderAngles(squares, &orderedSquares);
//    drawSquares(image, squares);
    /// FIND SQUARE METHOD https://docs.opencv.org/3.4/db/d00/samples_2cpp_2squares_8cpp-example.html#a20


    /// TEMPLATE MATCHING OPENCV ********************************************************************************************+++
      // https://docs.opencv.org/3.4.6/de/da9/tutorial_template_matching.html

    const char* image_window = "Source Image";

    cv::Mat templ = cv::imread("/home/tori/UWsim/Peg/templateFrontLittle.jpg", cv::IMREAD_COLOR);
    cv::Mat img;
    //img = cv::imread("/home/tori/UWsim/Peg/source10.png", cv::IMREAD_COLOR);
    cv::cvtColor(imageL_cv, img, cv::COLOR_GRAY2BGR); //actually not real conv in colors

    cv::Mat img_display = img.clone();

    cv::namedWindow( image_window, cv::WINDOW_AUTOSIZE );

    int templ_method = cv::TM_CCOEFF_NORMED;
    std::vector<double>scaleFactors = {1, 0.9, 0.8, 0.7, 0.75, 0.6, 0.65, 0.5,
                                         0.48, 0.45, 0.42, 0.4, 0.38, 0.38, 0.32, 0.3,
                                        0.28, 0.25, 0.22, 0.2, 0.15, 0.1};
    //std::vector<double>scaleFactors = {0.3};


    std::vector<double> minMaxValue(scaleFactors.size()),
                        scaleUpY(scaleFactors.size()),
                        scaleUpX(scaleFactors.size());

    std::vector<cv::Point> bestMatch(scaleFactors.size());
    cv::Mat imgScaled = img.clone();

    //last scaling is the last scaling, then if the for is "break" last scale is modified
    //it is used as index to consider only max/min values until the last scaling factor used
    int lastScale=scaleFactors.size();
    for (int i=0; i<scaleFactors.size(); i++){

      cv::resize(img, imgScaled, cv::Size(), scaleFactors[i], scaleFactors[i]);
      std::cout << "img size y, x: " << img.rows << "  " << img.cols << "\n";
      std::cout << "scaled img y, x: " << imgScaled.rows << "  " <<imgScaled.cols << "\n";
      // actual scale can be approximated so calculate scaleUp in this way
      // is better than 1/scaleFactors[i]
      scaleUpY[i] = ((double)img.rows) / ((double)imgScaled.rows);
      scaleUpX[i] = ((double)img.cols) / ((double)imgScaled.cols);

      //if scaled img little than template, break loop
      if (imgScaled.rows < templ.rows || imgScaled.cols < templ.cols){
        std::cout << "template bigger\n";
        lastScale = i-1;
        break;
      }


      MatchingMethod(templ_method, imgScaled, templ, &(bestMatch[i]), &(minMaxValue[i]));


//      std::cout << "scale factor UPX and UPY  " << scaleUpX << " " << scaleUpY  << "\n";
//      std::cout << "bestMatrch.x: " << bestMatch[i].x << "\n";
//      std::cout << "bestMatrch.y: " << bestMatch[i].y << "\n";
//      std::cout << "topLeft.x: " << topLeft.x << "\n";
//      std::cout << "topLeft.y: " << topLeft.y << "\n";
//      std::cout << "bottomRi.x: " << bottomRight.x << "\n";
//      std::cout << "bottomRi.y: " << bottomRight.y << "\n";
//      std::cout << "temple row col " << templ.rows << "  " << templ.cols << "\n\n";


//      cv::rectangle( imgScaled, bestMatch[i],
//                    cv::Point( bestMatch[i].x + templ.cols , bestMatch[i].y + templ.rows ),
//                     cv::Scalar::all(0), 1, 8, 0 );
//      std::string boh = "asdasda" + std::to_string(i);
//      cv::imshow( boh, imgScaled );
//      cv::Mat otherMat;
//      cv::resize (imgScaled, otherMat, cv::Size(), scaleUpX[i], scaleUpY[i]);
//      cv::imshow( "boh", otherMat );
//      cv::waitKey();



    }

    int indexBest;
    if( templ_method  == cv::TM_SQDIFF || templ_method == cv::TM_SQDIFF_NORMED ){
      indexBest = std::distance(minMaxValue.begin(),
                                  std::min_element(minMaxValue.begin(),
                                                   minMaxValue.begin() + lastScale));

    } else{
      indexBest = std::distance(minMaxValue.begin(),
                                 std::max_element(minMaxValue.begin(),
                                                  minMaxValue.begin() + lastScale));
    }

    std::cout << "BEST ITERATION: scaling factor " << scaleFactors[indexBest]
                 <<"\n VALUE:" << minMaxValue.at(indexBest) << "\n\n";

    cv::Point topLeft, bottomRight;
    topLeft.x = (int)(bestMatch[indexBest].x * scaleUpX[indexBest]);
    topLeft.y = (int)(bestMatch[indexBest].y * scaleUpY[indexBest]);
    bottomRight.x = (int)( (bestMatch[indexBest].x + templ.cols) * scaleUpX[indexBest]);
    bottomRight.y = (int)( (bestMatch[indexBest].y +  templ.rows) * scaleUpY[indexBest]);

    cv::rectangle( img_display, topLeft, bottomRight,
                   cv::Scalar::all(0), 1, 8, 0 );
    cv::imshow( image_window, img_display);
    cv::waitKey(0);


    //original not scaled
   // cv::createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );
    //MatchingMethod( 0, 0 ,);
    //cv::waitKey(0);

    /// TEMPLATE MATCHING OPENCV ********************************************************************************************+++


    /// FEATURE 2 + HOMOGRAPy *******************************************************************************************************+
    /// https://docs.opencv.org/3.4/d7/dff/tutorial_feature_homography.html
    /// NOT works, too parameters to set for all the function, even with canny preprocessing
    /// keypoint are not matched. Maybe a fine setup will work.

    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
//    cv::Mat img_object = cv::imread("/home/tori/UWsim/Peg/templateSideBorder.jpg",
//                                    cv::IMREAD_GRAYSCALE );

//    cv::Mat img_scene = imageL_cv;
//    cv::Canny(img_object, img_object, 50, 200, 3);
//    cv::Canny(img_scene, img_scene, 50, 200, 3);

//    cv::Mat imgd_scene = img_scene; //for intermiadate display
//    cv::Mat imgd_obj = img_object; //for intermiadate display

//    cv::Ptr<cv::xfeatures2d::SURF> detectorSURF = cv::xfeatures2d::SURF::create(300);
//    cv::Ptr<cv::xfeatures2d::SURF> detectorSURF2 = cv::xfeatures2d::SURF::create(
//          100, 4, 3, true);
//    cv::Ptr<cv::xfeatures2d::SIFT> detectorSIFT = cv::xfeatures2d::SIFT::create(
//          0, 3, 0.04, 10);
//    cv::Ptr<cv::xfeatures2d::SIFT> detectorSIFT2 = cv::xfeatures2d::SIFT::create(
//          0, 3, 0.02, 15);
//    std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
//    cv::Mat descriptors_object, descriptors_scene;
//    detectorSURF->detectAndCompute( img_object, cv::noArray(), keypoints_object, descriptors_object );
//    detectorSURF->detectAndCompute( img_scene, cv::noArray(), keypoints_scene, descriptors_scene );
//    std::cout <<"point in template: " << keypoints_object.size() << "\n\n";
//    std::cout <<"point in scene: " << keypoints_scene.size() << "\n\n";
//    cv::drawKeypoints(img_scene, keypoints_scene, imgd_scene, cv::Scalar(255,0,0));
//    cv::drawKeypoints(img_object, keypoints_object, imgd_obj, cv::Scalar(255,0,0));
//    cv::imshow("scena keypoint", imgd_scene);
//    cv::imshow("temmplate keypoint", imgd_obj);
//    cv::waitKey();


//    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
//    // Since SURF is a floating-point descriptor NORM_L2 is used
//    cv::Ptr<cv::DescriptorMatcher> matcher =
//        cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
//    std::vector< std::vector<cv::DMatch> > knn_matches;
//    matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );

//    //-- Filter matches using the Lowe's ratio test
//    const float ratio_thresh = 0.75f;
//    std::vector<cv::DMatch> good_matches;
//    std::cout << "knn_matches matches: " << knn_matches.size() << "\n\n";

//    for (size_t i = 0; i < knn_matches.size(); i++)
//    {
//        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
//        {
//            good_matches.push_back(knn_matches[i][0]);
//        }
//    }

//    std::cout << "goo matches: " << good_matches.size() << "\n\n";

//    //-- Draw matches
//    cv::Mat img_matches;
//    cv::drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
//                     good_matches, img_matches, cv::Scalar::all(-1),
//                     cv::Scalar::all(-1), std::vector<char>(),
//                     cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//    //-- Localize the object
//    std::vector<cv::Point2f> obj;
//    std::vector<cv::Point2f> scene;
//    for( size_t i = 0; i < good_matches.size(); i++ )
//    {
//        //-- Get the keypoints from the good matches
//        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
//        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
//    }
//    std::cout << "to find homog template good match: " << obj.size() << "\n\n";
//    std::cout << "to find homog scene good match: " << scene.size() << "\n\n";

//    cv::Mat H = cv::findHomography( obj, scene, cv::RANSAC, 3 );
//    std::cout << "mat result of findhomog: " << H.size() << "\n\n";
//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    std::vector<cv::Point2f> obj_corners(4);
//    obj_corners[0] = cv::Point2f(0, 0);
//    obj_corners[1] = cv::Point2f( (float)img_object.cols, 0 );
//    obj_corners[2] = cv::Point2f( (float)img_object.cols, (float)img_object.rows );
//    obj_corners[3] = cv::Point2f( 0, (float)img_object.rows );
//    std::vector<cv::Point2f> scene_corners(4);
//    cv::perspectiveTransform( obj_corners, scene_corners, H);
//    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//    cv::line( img_matches, scene_corners[0] + cv::Point2f((float)img_object.cols, 0),
//          scene_corners[1] + cv::Point2f((float)img_object.cols, 0), cv::Scalar(0, 255, 0), 4 );
//    cv::line( img_matches, scene_corners[1] + cv::Point2f((float)img_object.cols, 0),
//          scene_corners[2] + cv::Point2f((float)img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
//    cv::line( img_matches, scene_corners[2] + cv::Point2f((float)img_object.cols, 0),
//          scene_corners[3] + cv::Point2f((float)img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
//    cv::line( img_matches, scene_corners[3] + cv::Point2f((float)img_object.cols, 0),
//          scene_corners[0] + cv::Point2f((float)img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
//    //-- Show detected matches
//    cv::imshow("Good Matches & Object detection", img_matches );
//    cv::waitKey();

    /// FEATURE 2 + HOMOGRAPy *******************************************************************************************************+




    /**  VISP METHOD FOR KEYPOINT DETECTION, CHE NO BUONI*/
    // in comune per tutti i metodi di visp
    //vpDisplayOpenCV d(imageL_vp, 500, 500, "Tracking");
    //vpDisplay::display(imageL_vp);
    //vpDisplay::flush(imageL_vp);

//    /// method tracker *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//    vpKltOpencv tracker;
//    tracker.setMaxFeatures(200);
//    tracker.setWindowSize(10);
//    tracker.setQuality(0.015);
//    tracker.setMinDistance(15);
//    tracker.setHarrisFreeParameter(0.04);
//    tracker.setBlockSize(9);
//    tracker.setUseHarris(1);
//    tracker.setPyramidLevels(3);
//    tracker.initTracking(imageL_cv);

//    /// method keypoint ***********************************************************************
//    vpKeyPoint keypoint_learning;
//    keypoint_learning.loadConfigFile(configFileDetector);

//    while(ros::ok()){
//      /// TRACKING keypoint 1
//      //tracker.track(imageL_cv);
//      //tracker.display(imageL_vp, vpColor::red);

////      /// Tracking keypoint 2
//////      std::vector<cv::KeyPoint> trainKeyPoints;
//////      double elapsedTime;
//////      keypoint_learning.detect(imageL_vp, trainKeyPoints, elapsedTime);
//////      // display found points
//////      vpDisplay::display(imageL_vp);
//////      for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
//////        vpDisplay::displayCross(imageL_vp, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
//////      }
/////


//      vpDisplay::flush(imageL_vp);
//      vpDisplay::getClick(imageL_vp);
//    }



  } ///else not init by click




  //// old tracking method (con inizializ initclick)
  //objDetectionInit(imageL_vp, &tracker);
  //vpKeyPoint keypoint_detection;
  //keypoint_detection.loadConfigFile(configFileDetector);
  //keypoint_detection.loadLearningData(learnData, true);


/** *********************************************************************************************************+*********
                                   MAIN VISION LOOP
*******************************************************************************************************************/
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    robotVisInterface.getwTv(&(robVisInfo.robotState.wTv_eigen));
    worldInterface.getwT(&(robVisInfo.transforms.wTh_eigen), holeName);

    robotVisInterface.getLeftImage(&imageL_cv);
    robotVisInterface.getRightImage(&imageR_cv);
    imageL_cv.convertTo(imageL_cv, CV_8U); //TODO check if necessary convert in 8U
    imageR_cv.convertTo(imageR_cv, CV_8U); //TODO check if necessary convert in 8U
    vpImageConvert::convert(imageL_cv, imageL_vp);
    vpImageConvert::convert(imageR_cv, imageR_vp);

    vpDisplay::display(imageL_vp);
    vpDisplay::display(imageR_vp);

    CMAT::TransfMatrix wThole_cmat =
        CONV::matrix_eigen2cmat(robVisInfo.transforms.wTh_eigen);

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

//    CMAT::TransfMatrix wTholeEstimated_cmat =
//        CONV::matrix_eigen2cmat(robVisInfo.transforms.wTh_estimated_eigen);

//    CMAT::Vect6 swappedError =
//        CMAT::CartError(wThole_cmat, wTholeEstimated_cmat);
//    CMAT::Vect6 error;
//    error.SetFirstVect3(swappedError.GetSecondVect3());
//    error.SetSecondVect3(swappedError.GetFirstVect3());
//    logger.writeCmatMatrix(error, "error");


    /// method TRACKING DETECTION VISP ***********************************************************
                    ///TODO
    /// method TRACKING DETECTION VISP ***********************************************************


    /// METHOD 2 STEREO

    vpHomogeneousMatrix cLThole_st, cRThole_st;
    stereoTracking(imageL_vp, imageR_vp, &tracker, &cLThole_st, &cRThole_st);

    // display
    vpCameraParameters cam_left, cam_right;
    tracker.getCameraParameters(cam_left, cam_right);
    tracker.display(imageL_vp, imageR_vp, cLThole_st, cRThole_st, cam_left, cam_right, vpColor::red, 2);
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


    CMAT::TransfMatrix wTholeEstimated_stereoL_cmat =
        CONV::matrix_eigen2cmat(wTh_estimated_stereo_left);
    CMAT::TransfMatrix wTholeEstimated_stereoR_cmat =
        CONV::matrix_eigen2cmat(wTh_estimated_stereo_right);


    CMAT::Vect6 swappedError2 =
        CMAT::CartError(wThole_cmat, wTholeEstimated_stereoL_cmat);
    CMAT::Vect6 error2;
    error2.SetFirstVect3(swappedError2.GetSecondVect3());
    error2.SetSecondVect3(swappedError2.GetFirstVect3());
    logger.writeCmatMatrix(error2, "errorStereoL");

    /// TODO check if error l and r are same, they must are when stereo method is used
    CMAT::Vect6 swappedError3 =
        CMAT::CartError(wThole_cmat, wTholeEstimated_stereoR_cmat);
    CMAT::Vect6 error3;
    error3.SetFirstVect3(swappedError3.GetSecondVect3());
    error3.SetSecondVect3(swappedError3.GetFirstVect3());
    logger.writeCmatMatrix(error3, "errorStereoR");

    vpDisplay::displayText(imageL_vp, 30, 10, "A click to exit.", vpColor::red);
    vpDisplay::flush(imageL_vp);
    vpDisplay::flush(imageR_vp);
    if (vpDisplay::getClick(imageL_vp, false)) {
      return 1;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

   return 0;
}

int stereoTracking(vpImage<unsigned char> I_left, vpImage<unsigned char> I_right,
                   vpMbGenericTracker *tracker,
                   vpHomogeneousMatrix *cLeftTo, vpHomogeneousMatrix *cRightTo){
  try {

    tracker->track(I_left, I_right);
    tracker->getPose(*cLeftTo, *cRightTo);

  } catch (const vpException &e) {
    std::cerr << "Catch a ViSP exception: " << e.what() << std::endl;
  }
  return 0;
}




int initTracker(vpMbGenericTracker *tracker, int nCameras, Eigen::Matrix4d cRTcL){

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
      // note: here it must be putted the transf from RIGTH to LEFT and not viceversa
      mapCameraTransf["Camera2"] = CONV::transfMatrix_eigen2visp(cRTcL);

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



// returns sequence of squares detected on the image.
void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares,
                         int thresh, int N)
{

  using namespace std;
  using namespace cv;

  squares.clear();
  Mat pyr, timg, gray0(image.size(), CV_8U), gray;
  // down-scale and upscale the image to filter out the noise
  pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
  pyrUp(pyr, timg, image.size());
  vector<vector<Point> > contours;
  // find squares in every color plane of the image
  for( int c = 0; c < 3; c++ )
  {
      int ch[] = {c, 0};
      mixChannels(&timg, 1, &gray0, 1, ch, 1);
      // try several threshold levels
      for( int l = 0; l < N; l++ )
      {
          // hack: use Canny instead of zero threshold level.
          // Canny helps to catch squares with gradient shading
          if( l == 0 )
          {
              // apply Canny. Take the upper threshold
              // and set the lower to 0 (which forces edges merging)
              Canny(gray0, gray, 0, thresh, 5);
              // dilate canny output to remove potential
              // holes between edge segments
              dilate(gray, gray, Mat(), Point(-1,-1));
          }
          else
          {
              // apply threshold if l!=0:
              //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
              gray = gray0 >= (l+1)*255/N;
          }
          // find contours and store them all as a list
          findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
          vector<Point> approx;
          // test each contour
          for( size_t i = 0; i < contours.size(); i++ )
          {
              // approximate contour with accuracy proportional
              // to the contour perimeter
              approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);
              // square contours should have 4 vertices after approximation
              // relatively large area (to filter out noisy contours)
              // and be convex.
              // Note: absolute value of an area is used because
              // area may be positive or negative - in accordance with the
              // contour orientation
              if( approx.size() == 4 &&
                  fabs(contourArea(approx)) > 1000 &&
                  isContourConvex(approx) )
              {
                  double maxCosine = 0;
                  for( int j = 2; j < 5; j++ )
                  {
                      // find the maximum cosine of the angle between joint edges
                      double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                      maxCosine = MAX(maxCosine, cosine);
                  }
                  // if cosines of all angles are small
                  // (all angles are ~90 degree) then write quandrange
                  // vertices to resultant sequence
                  if( maxCosine < 0.3 )
                      squares.push_back(approx);
              }
          }
      }
  }
}


// the function draws all the squares in the image
void drawSquares( cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares,
                         const char* wndname)
{

  using namespace std;
  using namespace cv;
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        std::cout << "number of point:" << n<< "\n";
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);
        circle(image, squares[i][0], 5, Scalar(0,0,0), FILLED);
        circle(image, squares[i][1], 5, Scalar(255,255,0), FILLED);
        circle(image, squares[i][2], 5, Scalar(0,255,255), FILLED);
        circle(image, squares[i][3], 5, Scalar(255,255,255), FILLED);

    }
    imshow(wndname, image);
}

/**
 * @brief angle // helper function:
 * finds a cosine of angle between vectors
 * from pt0->pt1 and from pt0->pt2
 * @param pt1
 * @param pt2
 * @param pt0
 * @return
 */
double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

int orderAngles(std::vector<std::vector<cv::Point>> angles, std::vector<std::vector<cv::Point>> *orderedAngles){
  for (int i=0; i< angles.size(); i++){
    if (angles.at(i).size() != 4){
      std::cerr << "Angles must be 4\n";
      return -1;
    }
    orderAngles(angles.at(i), &(orderedAngles->at(i)));
  }
  return 0;
}

int orderAngles(std::vector<cv::Point> angles, std::vector<cv::Point> *orderedAngles){

  if (angles.size() != 4){
    std::cerr << "Angles must be 4\n";
    return -1;
  }

  cv::Point center = getCenter(angles);

  orderedAngles->resize(4);
  for (int i=0; i<4; i++){

    if (angles.at(i).x < center.x){
      if (angles.at(i).y < center.y){ //top left corner
        orderedAngles->at(0) = angles.at(i);
      } else { // bottom left corner
        orderedAngles->at(3) = angles.at(i);
      }

    } else {
      if (angles.at(i).y < center.y){ //top right corner
        orderedAngles->at(1) = angles.at(i);
      } else { // bottom rigth corner
        orderedAngles->at(2) = angles.at(i);
      }
    }
  }

}

cv::Point getCenter(std::vector<cv::Point> points){

    cv::Point A = points.at(0);
    cv::Point B = points.at(1);
    cv::Point C = points.at(2);
    cv::Point D = points.at(3);

    // Line AB represented as a1x + b1y = c1
    double a1 = A.x - A.y;
    double b1 = A.x - B.x;
    double c1 = a1*(A.x) + b1*(A.y);

    // Line CD represented as a2x + b2y = c2
    double a2 = D.y - C.y;
    double b2 = C.x - D.x;
    double c2 = a2*(C.x)+ b2*(C.y);

    double determinant = a1*b2 - a2*b1;

    if (determinant == 0)
    {
        /// TODO error, lines parallel
    }

    double x = (b2*c1 - b1*c2)/determinant;
    double y = (a1*c2 - a2*c1)/determinant;
    return cv::Point(x, y);

}


void MatchingMethod(int match_method, cv::Mat img, cv::Mat templ,
                    cv::Point *bestMatch, double *minMaxVal){
  using namespace std;
  using namespace cv;

  Mat result;

  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  result.create( result_rows, result_cols, CV_32FC1 );

  matchTemplate( img, templ, result, match_method);

  double minVal; double maxVal; Point minLoc; Point maxLoc;
  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc);
  if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED ) {
    *bestMatch = minLoc;
    *minMaxVal = minVal;
  } else {
    *bestMatch = maxLoc;
    *minMaxVal = maxVal;
  }


  return;
}


/**
 * @brief thresh_callback
 *
 * Also with this, difficult to select the right rectangle.
 *
 * global vvar required:
 * cv::Mat src;
int thresh = 10;
int thresh2 = 30;
int thresh_poly = 3;
cv::RNG rng(12345);
void thresh_callback(int, void* );
 *
 */
//void thresh_callback(int, void* )
//{
//  using namespace std;
//  using namespace cv;

//  Mat canny_output;
//  Canny( src, canny_output, thresh, thresh2, 3);

//  vector<vector<Point> > contours;
//  findContours( canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
//  vector<vector<Point> > contours_poly( contours.size() );
//  vector<Rect> boundRect( contours.size() );
//  vector<Point2f>centers( contours.size() );
//  vector<float>radius( contours.size() );
//  for( size_t i = 0; i < contours.size(); i++ )
//  {
//      approxPolyDP( contours[i], contours_poly[i], thresh_poly, true );
//      boundRect[i] = boundingRect( contours_poly[i] );
//  }
//  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
//  std::cout<< "find " <<  contours.size() << " countors\n";
//  for( size_t i = 0; i< contours.size(); i++ )
//  {
//      Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
//      //drawContours( drawing, contours_poly, (int)i, color );
//      rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
//  }
//  imshow( "Contours", drawing );

//}



/**
 * @brief goodFeaturesToTrack_Demo
 *
 * /// corner detect not useful, too many corners and the best one are not the corner
/// of the square
 *
 * var globali necessarie :
 * cv::Mat src;
int maxCorners = 20;
cv::RNG rng(12345);
const char* source_window = "Image";
void goodFeaturesToTrack_Demo( int, void* );
 */
//void goodFeaturesToTrack_Demo( int, void* )
//{
//    maxCorners = MAX(maxCorners, 1);
//    std::vector<cv::Point2f> corners;
//    double qualityLevel = 0.018;
//    double minDistance = 50;
//    int blockSize = 3, gradientSize = 3;
//    bool useHarrisDetector = false;
//    double k = 0.04;
//    cv::Mat copy = src.clone();
//    cv::goodFeaturesToTrack( src,
//                         corners,
//                         maxCorners,
//                         qualityLevel,
//                         minDistance,
//                         cv::Mat(),
//                         blockSize,
//                         gradientSize,
//                         useHarrisDetector,
//                         k );
//    std::cout << "** Number of corners detected: " << corners.size() << std::endl;
//    int radius = 4;
//    for( size_t i = 0; i < corners.size(); i++ )
//    {
//        cv::circle( copy, corners[i], radius, cv::Scalar(
//                      rng.uniform(0,255), rng.uniform(0, 256), rng.uniform(0, 256)), cv::FILLED );
//    }
//    cv::namedWindow( source_window );
//    cv::imshow( source_window, copy );
//}

//int objDetectionInit(vpImage<unsigned char> I, vpMbGenericTracker *tracker){

//  try {

//  vpCameraParameters cam;
//  vpHomogeneousMatrix cMo;

//  vpDisplayX display;

//  display.init(I, 300, 300, "Model-based edge tracker");

//  tracker->getCameraParameters(cam);

//  tracker->track(I);

//  vpKeyPoint keypoint_learning;
//  keypoint_learning.loadConfigFile(configFileDetector);

//  std::vector<cv::KeyPoint> trainKeyPoints;
//  double elapsedTime;
//  keypoint_learning.detect(I, trainKeyPoints, elapsedTime);
//  // display found points
//  vpDisplay::display(I);
//  for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
//    vpDisplay::displayCross(I, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
//  }
//  vpDisplay::displayText(I, 10, 10, "All Keypoints...", vpColor::red);
//  vpDisplay::displayText(I, 30, 10, "Click to continue with detection...", vpColor::red);
//  vpDisplay::flush(I);
//  vpDisplay::getClick(I, true);

//  std::vector<vpPolygon> polygons;
//  std::vector<std::vector<vpPoint> > roisPt;
//  std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair
//      = tracker->getPolygonFaces(false);
//  polygons = pair.first;
//  roisPt = pair.second;
//  std::vector<cv::Point3f> points3f;
//  tracker->getPose(cMo);
//  vpKeyPoint::compute3DForPointsInPolygons(cMo, cam, trainKeyPoints,
//                                           polygons, roisPt, points3f);
//  keypoint_learning.buildReference(I, trainKeyPoints, points3f);
//  keypoint_learning.saveLearningData(learnData, true);

//  // display found points
//  vpDisplay::display(I);
//  for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
//    vpDisplay::displayCross(I, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
//  }
//  vpDisplay::displayText(I, 10, 10, "Keypoints only on block...", vpColor::red);
//  vpDisplay::displayText(I, 30, 10, "Click to continue with detection...", vpColor::red);
//  vpDisplay::flush(I);
//  vpDisplay::getClick(I, true);


//  } catch (vpException &e) {
//    std::cout << "Catch an exception: " << e << std::endl;
//  }

//  return 0;
//}


/////OBJECT DETECTIONN
//int objDetection(vpImage<unsigned char> I, vpMbGenericTracker *tracker,
//                 vpKeyPoint *keypoint_detection, vpHomogeneousMatrix *cMo,
//                 double *error, double* elapsedTime){

//  try {

//    vpCameraParameters cam;
//    tracker->getCameraParameters(cam);

//    if (keypoint_detection->matchPoint(I, cam, *cMo, *error, *elapsedTime)) {
//      tracker->setPose(I, *cMo);
//      tracker->display(I, *cMo, cam, vpColor::red, 2);
//    }

//  } catch (vpException &e) {
//    std::cout << "Catch an exception: " << e << std::endl;
//  }

//  return 0;

//}






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







/**  FOR TEMPLATE MATCHINGGGG
 * TODO MULTISCALE APPROACH...
 * @brief MatchingMethod ORIGINAL FROM OPENCV
 *
 * gloabl var:
 * /// glob for template matching
bool use_mask = false;
cv::Mat img; cv::Mat templ; cv::Mat mask; cv::Mat result;
const char* image_window = "Source Image";
const char* result_window = "Result window";
int match_method;
int max_Trackbar = 5;
void MatchingMethod( int, void* );

 *
 */
//void MatchingMethod( int, void*)
//{
//  using namespace std;
//  using namespace cv;

//  Mat img_display;
//  img.copyTo( img_display );

//  int result_cols =  img.cols - templ.cols + 1;
//  int result_rows = img.rows - templ.rows + 1;

//  result.create( result_rows, result_cols, CV_32FC1 );

//  bool method_accepts_mask = (TM_SQDIFF == match_method || match_method == TM_CCORR_NORMED);
//  if (use_mask && method_accepts_mask)
//    { matchTemplate( img, templ, result, match_method, mask); }
//  else
//    { matchTemplate( img, templ, result, match_method); }

//  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
//  double minVal; double maxVal; Point minLoc; Point maxLoc;
//  Point matchLoc;
//  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
//  if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
//    { matchLoc = minLoc; }
//  else
//    { matchLoc = maxLoc; }

//  rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
//  rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
//  imshow( image_window, img_display );
//  imshow( result_window, result );
//  return;
//}







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
