#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

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

  if(scan_msg->ranges[540] < 1.0 &&
     scan_msg->ranges[539] < 1.0 &&
     scan_msg->ranges[538] < 1.0 &&
     scan_msg->ranges[541] < 1.0) {
    ROS_INFO("TURN!!!");
    velMsg.linear.x = 0.0;
    velMsg.angular.z = .8;
  } else {
    ROS_INFO("FORWARD");
    velMsg.linear.x = .5;
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
