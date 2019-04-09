#include "header/coordinator.h"


int main(int argc, char **argv){

  if (argc < 3){
    std::cout << "[COORDINATOR] Please insert the two robots name"
              << std::endl;
    return -1;
  }
  readyRob1 = false;
  readyRob2 = false;

  std::string robotName1 = argv[1];
  std::string robotName2 = argv[2];

  ros::init (argc, argv, "Coordinator");
  std::cout << "[COORDINATOR] Start" << std::endl;
  ros::NodeHandle nh;

  std::string topicReady1 = "/uwsim/" + robotName1+"_MissionManager"  + "/ready";
  std::string topicReady2 = "/uwsim/" + robotName2+"_MissionManager"  + "/ready";
  ros::Subscriber readyRob1Sub = nh.subscribe(topicReady1, 1, readyRob1SubCallback);
  ros::Subscriber readyRob2Sub = nh.subscribe(topicReady2, 1, readyRob2SubCallback);

  std::string topicStart = "/uwsim/Coord/StartStopBoth";
  ros::Publisher startBothPub = nh.advertise<std_msgs::Bool>(topicStart, 1);

  std_msgs::Bool start;
  start.data = false;
  int ms = 100;
  boost::asio::io_service io;

  while(ros::ok()){

    start.data = false;
    while(!(readyRob1 && readyRob2)){
      boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(ms));
      startBothPub.publish(start);
      ros::spinOnce();
      loopRater.wait();
    }

    std::cout << "[COORDINATOR] Now both robot are ready" << std::endl;


    start.data = true;
    while(readyRob1 && readyRob2){
      boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(ms));
      startBothPub.publish(start);
      ros::spinOnce();
      loopRater.wait();
    }

    std::cout << "[COORDINATOR] Now one robot is not ready anymore" << std::endl;


  }

}

void readyRob1SubCallback(const std_msgs::Bool::ConstPtr& start){
  readyRob1 = start->data;
}
void readyRob2SubCallback(const std_msgs::Bool::ConstPtr& start){
  readyRob2 = start->data;
}