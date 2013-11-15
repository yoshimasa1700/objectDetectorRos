#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <HFMD_core/CRForest.h>
#include <HFMD_core/util.h>
#include <HFMD_core/CDataset.h>

#include <boost/filesystem.hpp>

#include <objectDetector/Detect.h>
#include <objectDetector/Pos.h>

namespace enc = sensor_msgs::image_encodings;

class ImageConverter
{
public:
ImageConverter() : image_trans_(nh_){
    
std::cerr << boost::filesystem::current_path() << std::endl;


conf.loadConfig("config.xml");
conf.demoMode = 1;
conf.tsukubaMode = 1;
forest = NULL;
forest = new CRForest(conf);
forest->loadForest();

image_pub_ = image_trans_.advertise("out", 1);
image_sub_ = image_trans_.subscribe("image_raw", 1, &ImageConverter::image_callback, this);

// init opencv windows
cv::namedWindow("Image window");
//cv::namedWindow("mask");
//cv::namedWindow("result");

detect_pub = nh_.advertise<objectDetector::Detect>("objectdetector/detect",1);
blue_pub = nh_.advertise<objectDetector::Pos>("objectdetector/blue",1);
orange_pub = nh_.advertise<objectDetector::Pos>("objectdetector/orange",1);
sign_pub = nh_.advertise<objectDetector::Pos>("objectdetector/sign",1);

}

~ImageConverter()
{
cv::destroyAllWindows();
}

void image_callback(const sensor_msgs::ImageConstPtr& msg){

cv_bridge::CvImagePtr cv_ptr;
try{
cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
}
 catch (cv_bridge::Exception& e){
ROS_ERROR("cv_bridge exception: %s", e.what());
return;
}

cv::Mat inputImage(cv_ptr->image);
// cv::Mat img_mask;
// cv::Mat img_bkgmodel;
// cv::Mat dest = cv::Mat::zeros(inputImage.rows, inputImage.cols, inputImage.type());
// cv::Rect bb;
  
CTestDataset seqImg;
CDetectionResult detectR;

 cv::Mat resizeImg;

 cv::resize(inputImage, resizeImg, cv::Size(inputImage.cols/2, inputImage.rows/2), 0 , 0);

seqImg.img.push_back(&resizeImg);
detectR = forest->detection(seqImg);

objectDetector::Detect detectResult;
detect_pub.publish(detectResult);
objectDetector::Pos pos;
blue_pub.publish(pos);
orange_pub.publish(pos);
sign_pub.publish(pos);

 
 cv::Point sbord = detectR.detectedClass.at(0).centerPoint;
 cv::circle(inputImage, sbord, 5,cv::Scalar(255,0,0),2);

 cv::Point sbord = detectR.detectedClass.at(1).centerPoint;
 cv::circle(inputImage, sbord, 5,cv::Scalar(0,0,0),2);

 cv::Point sbord = detectR.detectedClass.at(2).centerPoint;
 cv::circle(inputImage, sbord, 5,cv::Scalar(255,0,0),2);
    
cv::imshow("Image window", inputImage);	
	
int key = cv::waitKey(1);

switch(key){
 case 'q':
exit(0);
break;
	  
// case 't':
//   cv::imwrite("test.png", inputImage);
//   break;

// case 's':
//   takeFlag = 1;
//   break;

// case 'p':
//   takeFlag = 0;
//   break;

defalut:
break;
	
}
//ここまで処理

//処理後の画像をpublish
image_pub_.publish(cv_ptr->toImageMsg());
}

ros::NodeHandle nh_;
image_transport::ImageTransport image_trans_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;

ros::Publisher detect_pub;
ros::Publisher blue_pub;
ros::Publisher orange_pub;
ros::Publisher sign_pub;

CRForest *forest;
CConfig conf;

// IBGS *bgs;
// cv::Mat bg;
// int imageNum;
// int takeFlag;
// std::ofstream dl;

};

int main(int argc, char** argv){


ros::init(argc, argv, "image_converter");
ImageConverter ic;
ros::spin();
return 0;
}











