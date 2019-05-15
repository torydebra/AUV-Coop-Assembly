#include "header/robotVisionInterface.h"


/**
 * @brief RobotVisionInterface::RobotInterfaceVision Constructor
 * @param argc the standard argument of c++ main needed for ros::init
 * @param argv the standard argument of c++ main needed for ros::init and for know robot names:
 * argv[1]=robotName, argv[2] = otherRobotName
 * @param toolName name of the peg in the xml file of the scene
 */
RobotVisionInterface::RobotVisionInterface(ros::NodeHandle nh, std::string robotName)
{

  this->robotName = robotName;
  this->topicRoot = "/uwsim/" + robotName + "/";

  std::cout << "[" << robotName <<"][ROBOT_VISION_INTERFACE] Start"<<std::endl;

  pubTwist = nh.advertise<geometry_msgs::TwistStamped>((topicRoot + "twist_command"),1);

  image_transport::ImageTransport it(nh);
  subLeftImage = it.subscribe(topicRoot + "cameraL", 1, &RobotVisionInterface::imageLeftCallback, this);
  subRightImage = it.subscribe(topicRoot + "cameraR", 1, &RobotVisionInterface::imageRightCallback, this);
}


int RobotVisionInterface::init(){

  if(!ros::ok()){
    return -1;
  }

  //Wait to transform wTv to be ready (or fail if wait more than 3 sec)
  std::string topic = "/" + robotName;
  tfListener.waitForTransform("world", topic, ros::Time(0), ros::Duration(3.0));

  //Wait for images to be ready (ie : the callbacks are called at least once)
  ros::Rate loop_rate(100); //100Hz
  while (leftImage == NULL) {

    std::cout << "[" << robotName <<"][ROBOT_VISION_INTERFACE] No left images yet..." << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

  while (rightImage == NULL) {

    std::cout << "[" << robotName <<"][ROBOT_VISION_INTERFACE] No right images yet..." << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }


  std::cout << "[" << robotName <<"][ROBOT_VISION_INTERFACE] Init done" << std::endl;

  return 0;
}

int RobotVisionInterface::getwTv(Eigen::Matrix4d* wTv_eigen){

  if(!ros::ok()){
    return -1;
  }

  tf::StampedTransform wTv_tf;

  std::string topic = "/" + robotName;
  try {
    tfListener.lookupTransform("world", topic, ros::Time(0), wTv_tf);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  *wTv_eigen = CONV::transfMatrix_tf2eigen(wTv_tf);

  return 0;
}


int RobotVisionInterface::sendyDot(std::vector<double> yDot){

  if(!ros::ok()){
    return -1;
  }
  if (yDot.size() != 6){
    std::cout << "[" << robotName <<"][ROBOT_VISION_INTERFACE] yDot must be of size 6 instead of "
              << yDot.size() << std::endl;
    return -2;
  }

  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x=yDot.at(0);
  twist.twist.linear.y=yDot.at(1);
  twist.twist.linear.z=yDot.at(2);
  twist.twist.angular.x=yDot.at(3);
  twist.twist.angular.y=yDot.at(4);
  twist.twist.angular.z=yDot.at(5);

  pubTwist.publish(twist);
  return 0;
}



/**
 * @brief RobotVisionInterface::getLeftImage for now convert in 8bit grayscale image
 * @param imageCV
 * @return
 */
int RobotVisionInterface::getLeftImage(cv::Mat *imageCV){


  cv_bridge::CvImagePtr imageCVPtr =
      cv_bridge::toCvCopy(leftImage, sensor_msgs::image_encodings::MONO8);
  *imageCV = imageCVPtr.get()->image;
  return 0;

}

int RobotVisionInterface::getRightImage(cv::Mat *imageCV){

  cv_bridge::CvImagePtr imageCVPtr =
      cv_bridge::toCvCopy(rightImage, sensor_msgs::image_encodings::MONO8);
  *imageCV = imageCVPtr.get()->image;
  return 0;

}

/**
 * @brief RobotVisionInterface::imageLeftCallback store in member class the image arrived,
 * process it only when really needed (that are, functions getImageL/R)
 * @param msg
 */
void RobotVisionInterface::imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
  leftImage = msg;
}

void RobotVisionInterface::imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
  rightImage = msg;
}
