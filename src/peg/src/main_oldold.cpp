#include <ros/ros.h>
#include <Eigen/Core>
#include <tf/transform_listener.h>

#include "../support/support.h"
#include "../support/defines.h"

#include "../header/publisher.h"
#include "../header/controller.h"

#include "../header/transforms.h"



int main(int argc, char **argv)
{
  ROS_INFO("[MAIN] Start");
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;

  std::string topicTwist = "/uwsim/g500_A/twist_command_A";
  ros::Publisher pubTwist = nh.advertise<geometry_msgs::TwistStamped>(topicTwist,1);


  Publisher pubClassTwist(pubTwist);
  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x=0;
  twist.twist.linear.y=0;
  twist.twist.linear.z=0;
  twist.twist.angular.x=0;
  twist.twist.angular.y=0;
  twist.twist.angular.z=0;


  /// GOAL VEHICLE
  double goalLinearVect[] = {-0.287, -0.062, 7.424};
  Eigen::Matrix4d wTgoal_eigen = Eigen::Matrix4d::Identity();

  //rot part
  wTgoal_eigen.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();
  //trasl part
  wTgoal_eigen(0, 3) = goalLinearVect[0];
  wTgoal_eigen(1, 3) = goalLinearVect[1];
  wTgoal_eigen(2, 3) = goalLinearVect[2];
  //TRANSFORM LISTENER things
  tf::TransformListener tfListener;
  tf::StampedTransform wTv_tf;

  //struct transforms to pass them to Controller class
  struct Transforms transf;
  transf.wTgoal_eigen = wTgoal_eigen;
  transf.wTv_eigen = CONV::transfMatrix_tf2eigen(wTv_tf);

  //Controller
  Controller controller;

  //Wait to transform to be ready (or fail if wait more than 3 sec)
  tfListener.waitForTransform("world", "/girona500_A", ros::Time(0), ros::Duration(3.0));


  ros::Rate r(1000); //1Hz
  while(ros::ok()){
    try {

    tfListener.lookupTransform("world", "/girona500_A", ros::Time(0), wTv_tf);

    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    transf.wTv_eigen = CONV::transfMatrix_tf2eigen(wTv_tf);
    controller.updateTransforms(&transf);

    CMAT::Matrix qDot = controller.execAlgorithm();


    twist.twist.angular.x=qDot(5);
    twist.twist.angular.y=qDot(6);
    twist.twist.angular.z=qDot(7);
    twist.twist.linear.x=qDot(8);
    twist.twist.linear.y=qDot(9);
    twist.twist.linear.z=qDot(10);

    pubClassTwist.publish(twist);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}






















