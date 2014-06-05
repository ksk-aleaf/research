#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include <sstream>
#include "imageProcesser/imageProcesser.hpp"
#include <opencv/cv.h>
//#include <opencv2/cv.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

const string CENTER_CAM_IMG_TOPIC_NAME = "/usb_cam_center/image_raw";
const string SIDE_CAM_IMG_TOPIC_NAME = "/usb_cam_side/image_raw";
const string PUBLISH_IMG_TOPIC_NAME = "/usb_cam/processed_image";
const int WHOLE_IMAGE_WID = 2560;
const int WHOLE_IMAGE_HT = 480;
const int CENTER_IMAGE_WID = 1280;
const int SIDE_IMAGE_WID = 640;

cv_bridge::CvImagePtr centerCvPtr;
cv_bridge::CvImagePtr sideCvPtr;
//IplImage* centerIplImage;
//IplImage* sideIplImage;


void centerImageCallback(const sensor_msgs::ImageConstPtr& msg){
	centerCvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

void sideImageCallback(const sensor_msgs::ImageConstPtr& msg){
	sideCvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
}

void processImage(){
	cv::Mat wholeMat;
	wholeMat.create(Size(WHOLE_IMAGE_WID,WHOLE_IMAGE_HT),CV_8UC3);
}


int main(int argc, char **argv)
{


	ros::init(argc, argv, "imageProcesser");
	ros::NodeHandle center_nh;
	ros::NodeHandle side_nh;
	image_transport::ImageTransport centerIt(center_nh);
	image_transport::ImageTransport sideIt(side_nh);
	image_transport::ImageTransport pubIt(side_nh);
	image_transport::Subscriber center_sub = centerIt.subscribe(CENTER_CAM_IMG_TOPIC_NAME, 1, centerImageCallback);
	image_transport::Subscriber side_sub = sideIt.subscribe(SIDE_CAM_IMG_TOPIC_NAME, 1, sideImageCallback);
	image_transport::Publisher publisher = pubIt.advertise(PUBLISH_IMG_TOPIC_NAME, 1);



//	ros::Publisher publisher = n.advertise<std_msgs::String>("processedCamImage", 1000);
	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
		sensor_msgs::Image msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		//ROS_INFO("%s", msg.data.c_str());

		publisher.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}


	return 0;
}
