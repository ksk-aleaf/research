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
const string PUBLISH_CENTER_IMG_TOPIC_NAME = "/usb_cam/processed_image/center";
const string PUBLISH_RIGHT_IMG_TOPIC_NAME = "/usb_cam/processed_image/right";
const string PUBLISH_LEFT_IMG_TOPIC_NAME = "/usb_cam/processed_image/left";
const int WHOLE_IMG_WID = 2560;
const int IMG_HT = 480;
const int CENTER_IMAGE_WID = 1280;
const int SIDE_IMG_WID = 640;
const Rect LEFT_ROI(SIDE_IMG_WID,0,SIDE_IMG_WID,IMG_HT);
const Rect RIGHT_ROI(0,0,SIDE_IMG_WID,IMG_HT);

cv_bridge::CvImagePtr centerCvPtr;
cv_bridge::CvImagePtr sideCvPtr;




void centerImageCallback(const sensor_msgs::ImageConstPtr& msg){
	centerCvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
}

void sideImageCallback(const sensor_msgs::ImageConstPtr& msg){
	sideCvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
}

void processSideImage(){
	cv::Mat leftMat;
	centerCvPtr.get()->image(LEFT_ROI);
}

void publishSideImg(image_transport::Publisher &leftImgPublisher,image_transport::Publisher &rightImgPublisher){
	static cv::Mat leftCvMat;
	static cv::Mat rightCvMat;

	if(sideCvPtr != 0 && sideCvPtr.get() != 0){
			//cv::flip(sideCvPtr.get()->image,sideCvPtr.get()->image,1);
			cv::Mat leftCvMat = sideCvPtr.get()->image(LEFT_ROI);
			cv::Mat rightCvMat = sideCvPtr.get()->image(RIGHT_ROI);
			sideCvPtr.get()->image = leftCvMat;
			leftImgPublisher.publish(	sideCvPtr.get()->toImageMsg());
			sideCvPtr.get()->image = rightCvMat;
			rightImgPublisher.publish( sideCvPtr.get()->toImageMsg());
	}
}

void publishCenterImg(image_transport::Publisher &centerImgPublihser){
	if(centerCvPtr != 0 && centerCvPtr.get() != 0){
		centerImgPublihser.publish(centerCvPtr.get()->toImageMsg());
	}
}


int main(int argc, char **argv)
{
	cv::Mat leftCvMat;
	cv::Mat rightCvMat;


	ros::init(argc, argv, "imageProcesser");
	ros::NodeHandle center_nh;
	ros::NodeHandle side_nh;
	image_transport::ImageTransport centerIt(center_nh);
	image_transport::ImageTransport sideIt(side_nh);
	image_transport::ImageTransport pubCenterIt(center_nh);
	image_transport::ImageTransport pubLeftIt(side_nh);
	image_transport::ImageTransport pubRightIt(side_nh);
	image_transport::Subscriber centerImgSubscriber = centerIt.subscribe(CENTER_CAM_IMG_TOPIC_NAME, 1, centerImageCallback);
	image_transport::Subscriber sideImgSubscriber = sideIt.subscribe(SIDE_CAM_IMG_TOPIC_NAME, 1, sideImageCallback);
	image_transport::Publisher centerImgPublisher = pubCenterIt.advertise(PUBLISH_CENTER_IMG_TOPIC_NAME, 1);
	image_transport::Publisher leftImgPublisher = pubLeftIt.advertise(PUBLISH_LEFT_IMG_TOPIC_NAME, 1);
	image_transport::Publisher rightImgPublisher = pubRightIt.advertise(PUBLISH_RIGHT_IMG_TOPIC_NAME, 1);

	//leftCvPtr.get() = *cv_bridge::CvImage();
	//rightCvPtr.get() = *cv_bridge::CvImage();

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		//processCenterImage();
		publishSideImg(leftImgPublisher,rightImgPublisher);
		publishCenterImg(centerImgPublisher);

		ros::spinOnce();

		loop_rate.sleep();
	}


	return 0;
}
