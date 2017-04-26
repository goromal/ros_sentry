#include "ros_sentry/joy_listener.h"

JoyListener::JoyListener() : active_LED (0), first_iteration (true)
{
	// for publishing movement data (in 1D, as of right now...)
	vel_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("controller_data", 100);
	// for listening to the joy parameters
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 50, &JoyListener::joyCallback, this);

	speeds[0] = 1.0;
	speeds[1] = 0.5;
	speeds[2] = 0.25;
	speeds[3] = 0.1;
	speeds[4] = 0.05;
}

void JoyListener::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	/*
	 * Array information:
	 * 		array[0]:  0 = no power (from joy->buttons[0])
	 * 				   1 = give power
	 * 		array[1]:  0-9 active LED (from joy->axes[0])
	 */

	static double prev_seconds = 0.0;
	static double dt = 0.0;

	std_msgs::Int32MultiArray array;
	array.data.clear();
	std_msgs::Header h = joy->header;

	array.data.push_back(joy->buttons[0]); // Button press info

	double multiplier = -1.0 * joy->axes[0]; // for calculating speed

	if (!first_iteration)
		dt += 0.05 * h.seq - prev_seconds;
	else
		first_iteration = false;

	prev_seconds = 0.05 * h.seq;

	int cent_val = sgn(multiplier) * 100 * multiplier; // guaranteed to be positive
	cent_val /= 25; // should give a number between 0 and 4

	if (dt >= speeds[cent_val] || dt < 0) // only proceed if requisite time has passed, or seq has reset somehow
	{
		dt = 0.0;
		iterate_active_LED(sgn(multiplier));
		array.data.push_back(active_LED);
		vel_pub_.publish(array);
	}
}

void JoyListener::iterate_active_LED(int iter)
{
	if (active_LED + iter > MAX_LED)
		active_LED = MIN_LED;
	else if (active_LED + iter < MIN_LED)
		active_LED = MAX_LED;
	else
		active_LED += iter;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "joy_listener"); // name of the node
	JoyListener joy_listener;

	ros::spin();
	return 0;
}
