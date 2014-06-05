#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "imageProcesser/imageProcesser.h"

using namespace std;

const string CENTER_CAM_IMG_TOPIC_NAME = "/usb_cam/center_image_raw";
const string SIDE_CAM_IMG_TOPIC_NAME = "/usb_cam/side_image_raw";


void chatterCallback()


int main(int argc, char **argv)
{

	ros::init(argc, argv, "imageProcesser");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<std_msgs::String>("processedCamImage", 1000);
	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		publisher.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}


	return 0;
}
