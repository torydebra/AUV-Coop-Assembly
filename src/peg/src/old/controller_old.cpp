#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "../header/publisher.h"
#include <eigen3/Eigen/Geometry>
#include <cmat/cmat.h>

typedef Eigen::Matrix<double, 3, 3> JacobianVehiclePose;

int main(int argc, char **argv)
{
  ROS_INFO("[CONTROLLER] Start");
  ros::init(argc, argv, "controller");
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

  // coord vehicle near the peg [-0.287, -0.062, 7.424]
  tf::Vector3 goalPoint = tf::Vector3(-5.287, -0.062, 7.424); //respect world

  tf::Transform goal = tf::Transform(tf::Matrix3x3(1, 0, 0,  0, 1, 0,  0, 0, 1), goalPoint);

  tf::Vector3 qDot = tf::Vector3(0, 0, 0);
  tf::Vector3 xDot = tf::Vector3(0, 0, 0);

  //TRANSFORM LISTENER ROBE
  tf::TransformListener tfListener;
  tf::StampedTransform wTv;

  //Wait to transform to be ready (or fail if wait more than 3 sec)
  tfListener.waitForTransform("world", "/girona500_A", ros::Time(0), ros::Duration(3.0));

  ros::Rate r(1000); //1Hz
  while(ros::ok()){
    try {

    tfListener.lookupTransform("world", "/girona500_A", ros::Time(0), wTv);

    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    //simple difference controller
    //double k = 0.05;
    //qDot = k * (goalPoint - wTv.getOrigin());

    /** with jacobian */
    //reference
    double k = 0.2;
    xDot = k * (goalPoint - wTv.getOrigin());

    double vect[16];
    vect[0] = wTv.getBasis().getRow(0).getX();
    vect[0] = wTv.getBasis().getRow(0).getY();

    //cmat_wTv.SetRotMatrix(wTv.getBasis());
    //cmat_wTv.Matrix[1] = 1;
    //cmat_wTv.GetTrasl().PrintMtx();
    //JacobianVehiclePose Jveh = JacobianVehiclePose();
    //Jveh.
    tf::Matrix3x3 jacobianVehiclePose = wTv.getBasis();

    qDot = (jacobianVehiclePose.inverse() * xDot);


    twist.twist.linear.x=qDot.getX();
    twist.twist.linear.y=qDot.getY();
    twist.twist.linear.z=qDot.getZ();

    pubClassTwist.publish(twist);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
