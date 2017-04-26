#include <ros/ros.h>
#include "std_msgs/Int32MultiArray.h"
#include <sensor_msgs/Joy.h>

#define MIN_LED		0
#define MAX_LED		9

class JoyListener
{
public:
	JoyListener();
private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void iterate_active_LED(int iter);
	int active_LED;
	double speeds[5];
	bool first_iteration;
	ros::NodeHandle nh_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;
};

template <typename T> int sgn(T val) { // returns -1, 0, 1
    return (T(0) < val) - (val < T(0));
}
