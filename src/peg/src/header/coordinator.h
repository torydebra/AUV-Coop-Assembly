#ifndef COORDINATOR_H
#define COORDINATOR_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

bool readyRob1;
bool readyRob2;


void readyRob1SubCallback(const std_msgs::Bool::ConstPtr& start);
void readyRob2SubCallback(const std_msgs::Bool::ConstPtr& start);

#endif // COORDINATOR_H
