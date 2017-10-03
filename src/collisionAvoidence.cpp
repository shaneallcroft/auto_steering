#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class SubAndPub
{
public:
	SubAndPub()
	{
		pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

		sub = n.subscribe("/scan",10, &SubAndPub::cmdVelCallback, this);
	}
	void cmdVelCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
	{
		geometry_msgs::Twist velMsg;
		if(scan_msg->ranges[540] < 2)
		{
			velMsg.linear.x = 0.0;
			velMsg.angular.z = 1.8;
		}
		else
		{
			velMsg.linear.x = 1.5;
			velMsg.angular.z = 1.8;
		}
		pub.publish(velMsg);

	}
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
};
	int main(int argc, char **argv)
	{

		ros::init(argc, argv, "collisionAvoidence");

		SubAndPub substerAndPubster;
		ros::spin();

		return 0;



	}
