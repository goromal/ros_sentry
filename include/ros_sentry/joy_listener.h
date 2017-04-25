#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>

#include <sstream>

class JoyListener
{
public:
	JoyListener();
private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle nh_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;
};
