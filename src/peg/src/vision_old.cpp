#include <ros/ros.h>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char **argv){

  std::string left_im = "/home/tori/UWsim/Peg/cameraL_front.png";
  std::string right_im = "/home/tori/UWsim/Peg/cameraR_front.png";

  cv::Mat left = cv::imread(left_im , cv::IMREAD_COLOR);
  if ( left.empty() )
  {
      std::cout<<"Cannot read image file: "<<left_im;
      return -1;
  }

  cv::Mat right = cv::imread(right_im, cv::IMREAD_COLOR);
  if ( right.empty() )
  {
      std::cout<<"Cannot read image file: "<<right_im;
      return -1;
  }


  //convert from colors to single channel
  cv::Mat right_HSV;
  cv::Mat right_GRAY;
  cv::Mat right_single;
  //cv::cvtColor(right, right_HSV, cv::COLOR_BGR2HSV);
  cv::cvtColor(right, right_single, cv::COLOR_BGR2GRAY);
  cv::cvtColor(right, right_GRAY, cv::COLOR_BGR2GRAY);

  cv::Mat left_HSV;
  cv::Mat left_GRAY;
  cv::Mat left_single;
  //cv::cvtColor(left, left_HSV, cv::COLOR_BGR2HSV);
  cv::cvtColor(left, left_single, cv::COLOR_BGR2GRAY);
  cv::cvtColor(left, left_GRAY, cv::COLOR_BGR2GRAY);


  //Apply a Median blur to reduce noise and avoid false circle detection:
  cv::medianBlur(right_single, right_single, 3);
  cv::medianBlur(left_single, left_single, 3);

  cv::imshow("blurredRight", right_single);
  cv::waitKey(0);
  cv::imshow("blurredLeft", left_single);
  cv::waitKey(0);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(right_single, circles, cv::HOUGH_GRADIENT, 1,
               right_single.rows/16,  // change this value to detect circles with different distances to each other
               100, 15, 1, 30 // change the last two parameters
          // (min_radius & max_radius) to detect larger circles
  );


  for( size_t i = 0; i < circles.size(); i++ )
  {
      std::cout << "detected" << "\n";
      cv::Vec3i c = circles[i];
      cv::Point center = cv::Point(c[0], c[1]);
      // circle center
      cv::circle( right_single, center, 1, cv::Scalar(0,100,100), 1, cv::LINE_AA);
      // circle outline
      int radius = c[2];
      cv::circle( right_single, center, radius, cv::Scalar(255,0,255), 1, cv::LINE_AA);
  }
  cv::imshow("detected circles right", right_single);
  cv::waitKey();

  std::vector<cv::Vec3f> circles2;
  cv::HoughCircles(left_single, circles2, cv::HOUGH_GRADIENT, 1,
               left_single.rows/16,  // change this value to detect circles with different distances to each other
               100, 15, 1, 30 // change the last two parameters
          // (min_radius & max_radius) to detect larger circles
  );
  for( size_t i = 0; i < circles2.size(); i++ )
  {
     std::cout << "detected" << "\n";
      cv::Vec3i c = circles2[i];
      cv::Point center = cv::Point(c[0], c[1]);
      // circle center
      cv::circle( left_single, center, 1, cv::Scalar(0,100,100), 1, cv::LINE_AA);
      // circle outline
      int radius = c[2];
      cv::circle( left_single, center, radius, cv::Scalar(255,0,255), 1, cv::LINE_AA);
  }
  cv::imshow("detected circles left", left_single);
  cv::waitKey();


  int numDisparities=48;
  int blockSize=std::atoi(argv[1]);
  cv::Ptr<cv::StereoMatcher> stereo = cv::StereoBM::create(numDisparities, blockSize);
  cv::Mat disparity;
  stereo->compute(left_GRAY, right_GRAY, disparity);

  /// stereo rectify, TODO all'inizio
  cv::Mat intrParam = (cv::Mat_<double>(3,3) <<
                       257.34082279179285, 0.0,               160.0,
                       0.0,               257.34083046114705, 120.0,
                       0.0,               0.0,                1.0);

  cv::Mat lRr = cv::Mat::eye(3,3, CV_64F);
  cv::Mat lDistr = (cv::Mat_<double>(3,1) << 0.19999999999999984, 0.0, 0.0);
  cv::Mat distorsionParam = cv::Mat::zeros(5,1, CV_64F) ;
  cv::Size imageSize(right_single.rows, right_single.cols);


  // output matrices of stereorect
  cv::Mat lRectR = cv::Mat::zeros(3,3, CV_64F);
  cv::Mat rRectR = cv::Mat::zeros(3,3, CV_64F);
  cv::Mat lProject = cv::Mat::zeros(3,4, CV_64F); //the 3x4 P matrix
  cv::Mat rProject = cv::Mat::zeros(3,4, CV_64F);
  cv::Mat disp2depthMap = cv::Mat::zeros(4,4, CV_64F);
  cv::stereoRectify(intrParam, distorsionParam, intrParam,  distorsionParam,
                    imageSize, lRr, lDistr,
                    lRectR, rRectR, lProject, rProject ,
                    disp2depthMap);

  //project center of circle in common
  std::cout << "lRectR:\n" << lRectR << "\n\n";
  std::cout << "rRectR:\n" << rRectR << "\n\n";
  std::cout << "lProject:\n" << lProject << "\n\n";
  std::cout << "rProject:\n" << rProject << "\n\n";
  std::cout << "disp2depthMap (Q):\n" << disp2depthMap << "\n\n";


  /// show circles on disparity map
  cv::Mat disp8U;
  disparity.convertTo(disp8U, CV_8U);
  cv::imshow("disparity", disp8U);
  cv::waitKey(0);


  cv::Mat disp8UFilt;
  cv::medianBlur(disp8U, disp8UFilt, 3);
  cv::imshow("disp blurred", disp8UFilt);
  cv::waitKey(0);
  cv::Mat disp8UCirc = disp8UFilt;
  for( size_t i = 0; i < circles2.size(); i++ )
  {
      cv::Vec3i c = circles2[i];
      cv::Point center = cv::Point(c[0], c[1]);
      // circle center
      cv::circle( disp8UCirc, center, 1, cv::Scalar(0,100,100), 1, cv::LINE_AA);
      // circle outline
      int radius = c[2];
      cv::circle( disp8UCirc, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
  }
  cv::imshow("disparityCirc", disp8UCirc);
  cv::waitKey(0);



//  /// reporject imaget to 3D method
//  cv::Mat _3dImage(disparity.size(), CV_64FC3); //C3 stand for 3channels
//  cv::reprojectImageTo3D(disparity, _3dImage, disp2depthMap);


//  cv::imshow("final", _3dImage);
//  cv::waitKey(0);

//  cv::Vec3i c = circles2[0];
//  std::cout << c[0] << "\n";
//  std::cout << "3D POIMT  " << _3dImage.at<cv::Vec3d>(c[1], c[0]) << std::endl;

//  std::cout << "3D corner bottom left" << _3dImage.at<cv::Vec3d>(196, 106) << std::endl;



  /// triangulation method
  cv::Mat pnts3D;
  //cv::Point2f centerL = cv::Point(circles2[0][0], circles2[0][1]);
  //cv::Point2f centerR = cv::Point(circles[0][0], circles[0][1]);
  cv::Mat centerL(1,1,CV_32FC2);
  cv::Mat centerR(1,1,CV_32FC2);
  centerL.at<cv::Vec2f>(0,0)[0] = circles2[0][0];
  centerL.at<cv::Vec2f>(0,0)[1] = circles2[0][1];
  centerR.at<cv::Vec2f>(0,0)[0] = circles[0][0];
  centerR.at<cv::Vec2f>(0,0)[1] = circles[0][1];


  //std::cout << centerL << "\n";

  cv::triangulatePoints( lProject, rProject, centerL, centerR, pnts3D );


  cv::Mat t = pnts3D.t();
  cv::Mat pnts3DT = cv::Mat( 1, 1, CV_32FC4, t.data );

  cv::Mat resultPoints;
  cv::convertPointsFromHomogeneous( pnts3DT, resultPoints );
  std::cout << "3D POIMT triangulat  " << pnts3DT << std::endl;


  /// metodo tizion stackoverflow
  std::vector<cv::Vec3f> surfacePoints, realSurfacePoints;

  unsigned int N = 1;
  for(int i=0;i<N;i++) {
      double d, disparity;
      // since you have stereo vision system in which cameras lays next to
      // each other on OX axis, disparity is measured along OX axis
      //d = T.at<double>(0,0);
      disparity = circles2[0][0] - circles[0][0];
      std::cout << "asdas\n" << disparity << "\n\n";

      surfacePoints.push_back(cv::Vec3f(circles2[0][0], circles2[0][1], disparity));
  }

  cv::perspectiveTransform(surfacePoints, realSurfacePoints, disp2depthMap);
  std::cout << "3D POIMT tizio  " << realSurfacePoints.at(0) << std::endl;



  /// pNp probl
  // corners per le imaggine front_LEFT
  // bottom left x,y = 106,196
  //bottom right x,y = 252, 202


  return 0;
}

/**
  OPENCV matrix are ROW MAJOR

  cameras intrinsic param  (same for both)
  257.34082279179285, 0.0,               160.0,
  0.0,               257.34083046114705, 120.0,
  0.0,               0.0,                1.0

  trasf matrix between LEFT and RIGHT (lTr)
  0.19999999999999984, 0.0, 0.0 posiz
  identity for rotation


  foto _3  g500_C to hole
- Translation: [3.924, -0.826, 0.867]
- Rotation: in Quaternion [0.038, -0.015, 0.954, 0.298]
            in RPY (radian) [-0.006, -0.082, 2.536]
            in RPY (degree) [-0.326, -4.708, 145.280]


 */
