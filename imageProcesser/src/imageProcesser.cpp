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

//const string CENTER_CAM_IMG_TOPIC_NAME = "/usb_cam_center/image_raw/decompressed";
//const string SIDE_CAM_IMG_TOPIC_NAME = "/usb_cam_side/image_raw/decompressed";
const string CENTER_CAM_IMG_TOPIC_NAME = "/usb_cam_center/image_raw";
const string SIDE_CAM_IMG_TOPIC_NAME = "/usb_cam_side/image_raw";
const string PUBLISH_CENTER_IMG_TOPIC_NAME = "/usb_cam/processed_image/center";
const string PUBLISH_RIGHT_IMG_TOPIC_NAME = "/usb_cam/processed_image/right";
const string PUBLISH_LEFT_IMG_TOPIC_NAME = "/usb_cam/processed_image/left";
const int WHOLE_IMG_WID = 2560;
const int IMG_HEIGHT = 480;
const int CAM_IMG_WID = 1280;
const int CENTER_IMG_WID = 1280;
const int SIDE_IMG_WID = 640;
const double RESIZE_SCALE = 0.5;
//const Rect LEFT_ROI(CAM_IMG_WID / 2 ,0,SIDE_IMG_WID  ,IMG_HEIGHT );
//const Rect RIGHT_ROI( 0, 0, SIDE_IMG_WID ,IMG_HEIGHT );
const Rect LEFT_ROI(int(CAM_IMG_WID * RESIZE_SCALE / 2) ,0,int(SIDE_IMG_WID  * RESIZE_SCALE) ,int(IMG_HEIGHT  * RESIZE_SCALE) );
const Rect RIGHT_ROI( 0, 0, int(SIDE_IMG_WID * RESIZE_SCALE) ,int(IMG_HEIGHT  * RESIZE_SCALE) );
const string CENTER_CAM_IMG_ENCORDING = sensor_msgs::image_encodings::RGB8;
const string SIDE_CAM_IMG_ENCORDING = sensor_msgs::image_encodings::RGB8;


//subscribe or publish image pointer
cv_bridge::CvImage leftCvImage;
cv_bridge::CvImage rightCvImage;
cv_bridge::CvImagePtr centerCvImagePtr;
cv_bridge::CvImagePtr sideCvImagePtr;
cv_bridge::CvImagePtr leftCvImagePtr(new cv_bridge::CvImage);
cv_bridge::CvImagePtr rightCvImagePtr(new cv_bridge::CvImage);
const cv::Mat centerSrcMat;
const cv::Mat sideSrcMat;
cv::Mat resizeCenterImageMat(CENTER_IMG_WID * RESIZE_SCALE , IMG_HEIGHT * RESIZE_SCALE , CV_8UC3);
cv::Mat resizeSideImageMat(SIDE_IMG_WID * RESIZE_SCALE , IMG_HEIGHT * RESIZE_SCALE , CV_8UC3);
bool resizeFlag = false;




void centerImageCallback(const sensor_msgs::ImageConstPtr& msg){
	centerCvImagePtr = cv_bridge::toCvCopy(msg, CENTER_CAM_IMG_ENCORDING);
}

void sideImageCallback(const sensor_msgs::ImageConstPtr& msg){
	sideCvImagePtr = cv_bridge::toCvCopy(msg, SIDE_CAM_IMG_ENCORDING);
}

void resizeImage(){
	if(centerCvImagePtr != 0 && centerCvImagePtr.get() != 0){
		cv::resize(centerCvImagePtr->image,resizeCenterImageMat,cv::Size(),RESIZE_SCALE,RESIZE_SCALE,cv::INTER_LINEAR);
		centerCvImagePtr->image = resizeCenterImageMat;
	}

	if(sideCvImagePtr != 0 && sideCvImagePtr.get() != 0){
		cv::resize(sideCvImagePtr->image,resizeSideImageMat,cv::Size(),RESIZE_SCALE,RESIZE_SCALE,cv::INTER_LINEAR);
		sideCvImagePtr->image = resizeSideImageMat;
	}
}

void processCenterImage(){

	if (centerCvImagePtr == NULL || centerCvImagePtr.get() == NULL){
		return;
	}

	cv::Mat mat = centerCvImagePtr.get()->image;
	int mat_channnel = mat.channels();
	int pixVal = 0;
	int pixValRem = 0;

	cvtColor(mat,mat,CV_RGB2HSV);

	Mat_<Vec3b>& img = (Mat_<Vec3b>&)mat; // 画像の 3 チャンネルポインタ

	for(int y = 0; y < mat.rows; y++){
		for(int x = 0; x < 300; x++){
			for(int channel = 0; channel < mat_channnel; channel++){
				pixVal = img(y,x)[channel];
				pixValRem = pixVal % 16;
				img(y,x)[channel] = pixVal-pixValRem;
				//img(y,x)[channel] = 255;
			}
		}
	}

	centerCvImagePtr.get()->image = mat;
}



void processSideImg(){
	if(sideCvImagePtr != 0 && sideCvImagePtr.get() != 0){
		if(	sideCvImagePtr.get()->image.rows >= LEFT_ROI.height && sideCvImagePtr.get()->image.cols >= (LEFT_ROI.width + LEFT_ROI.x)){
			if(leftCvImagePtr != 0 && leftCvImagePtr.get() != 0){
				leftCvImagePtr.get()->image = sideCvImagePtr.get()->image(LEFT_ROI);
			}

			if(rightCvImagePtr != 0 && rightCvImagePtr.get() != 0){
				rightCvImagePtr.get()->image = sideCvImagePtr.get()->image(RIGHT_ROI);
			}
		}
	}
}

void publishSideImg(image_transport::Publisher &leftImgPublihser,image_transport::Publisher &rightImgPublihser){
	if(leftCvImagePtr != 0 && leftCvImagePtr.get() != 0){
		leftImgPublihser.publish(leftCvImagePtr.get()->toImageMsg());
	}

	if(rightCvImagePtr != 0 && rightCvImagePtr.get() != 0){
		rightImgPublihser.publish(rightCvImagePtr.get()->toImageMsg());
	}
}


void publishCenterImg(image_transport::Publisher &centerImgPublihser){
	if(centerCvImagePtr != 0 && centerCvImagePtr.get() != 0){
		centerImgPublihser.publish(centerCvImagePtr.get()->toImageMsg());
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

	leftCvImagePtr->encoding = SIDE_CAM_IMG_ENCORDING;
	rightCvImagePtr->encoding = SIDE_CAM_IMG_ENCORDING;

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		resizeImage();
		//processCenterImage();
		processSideImg();
		publishSideImg(leftImgPublisher,rightImgPublisher);
		publishCenterImg(centerImgPublisher);

		ros::spinOnce();

		loop_rate.sleep();
	}


	return 0;
}
