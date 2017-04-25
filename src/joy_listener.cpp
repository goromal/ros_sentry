#include "ros_sentry/joy_listener.h"

JoyListener::JoyListener()
{
	// for testing purposes
	vel_pub_ = nh_.advertise<std_msgs::String>("my_chatter", 1000);
	// for listening to the joy parameters
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyListener::joyCallback, this);
}

void JoyListener::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	std_msgs::String msg;
	std::stringstream ss;
	ss << "Axes: " << joy->axes[0] << " " << joy->axes[1] << " " << joy->axes[2] << std::endl;
	ss << "Buttons: " << joy->buttons[0] << "..." << std::endl;
	msg.data = ss.str();
	vel_pub_.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "joy_listener"); // name of the node, I'm pretty sure...
	JoyListener joy_listener;

	ros::spin();
	return 0;
}
