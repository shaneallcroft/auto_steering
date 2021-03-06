#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define MIN_DIST 0.8

class SubAndPub
{
  public:
  SubAndPub() {
      pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

      sub = n.subscribe("/scan",10, &SubAndPub::cmdVelCallback, this);
  }
  void cmdVelCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
};

void SubAndPub::cmdVelCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  geometry_msgs::Twist velMsg;
  bool shouldTurn = false;
  double sumOfLeftSide = 0.0;
  double sumOfRightSide = 0.0;
  for (int i = scan_msg->ranges.size() / 3; i < (scan_msg->ranges.size() - (scan_msg->ranges.size() / 3)); ++i) {
      if (scan_msg->ranges[i] < MIN_DIST) {
          shouldTurn = true;
       }
      if(i < scan_msg->ranges.size()>>1){
          sumOfLeftSide += scan_msg->ranges[i];
      }
      else{
          sumOfRightSide += scan_msg->ranges[i];
      }
  }
  
  if(shouldTurn && sumOfLeftSide > sumOfRightSide ){
    ROS_INFO("TURN LEFT!!!");
    velMsg.linear.x = 0.0;
    velMsg.angular.z = .3;
  }
  else if(shouldTurn && sumOfLeftSide <= sumOfRightSide){
      ROS_INFO("TURN RIGHT!!!");
      velMsg.linear.x = 0.0;
      velMsg.angular.z = -.3;
  }
  else {
    ROS_INFO("FORWARD");
    velMsg.linear.x = .3;
    velMsg.angular.z = 0.0;
  }

  pub.publish(velMsg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collisionAvoidence");

  SubAndPub substerAndPubster;
  ros::spin();

  return 0;
}
