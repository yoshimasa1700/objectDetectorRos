#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "package_bgs/FrameDifferenceBGS.h"
#include "package_bgs/StaticFrameDifferenceBGS.h"
#include "package_bgs/WeightedMovingMeanBGS.h"
#include "package_bgs/WeightedMovingVarianceBGS.h"
#include "package_bgs/MixtureOfGaussianV1BGS.h"
#include "package_bgs/MixtureOfGaussianV2BGS.h"
#include "package_bgs/AdaptiveBackgroundLearning.h"
#if CV_MAJOR_VERSION >= 2 && CV_MINOR_VERSION >= 4 && CV_SUBMINOR_VERSION >= 3
#include "package_bgs/GMG.h"
#endif

namespace enc = sensor_msgs::image_encodings;
#define IMAGEMAX 100


std::string hName;
std::string color;
std::string dist;
std::stringstream dirpath;

cv::Rect getBB(const cv::Mat& input){
  cv::Rect bb;
  cv::Mat v;
      
  cv::reduce(input, v, 0,CV_REDUCE_SUM);
  for(int i = input.cols; i > 0; --i)
    if(v.at<double>(0,i) != 0)
      bb.x = i;

  cv::reduce(input, v, 1,CV_REDUCE_SUM);
  for(int i = input.rows; i > 0; --i)
    if(v.at<double>(i,0) != 0)
      bb.y = i;

  cv::reduce(input, v, 0,CV_REDUCE_SUM);
  for(int i = 0; i < input.cols; ++i)
    if(v.at<double>(0,i) != 0)
      bb.width = i - bb.x;

  cv::reduce(input, v, 1,CV_REDUCE_SUM);
  for(int i = 0; i < input.rows; ++i)
    if(v.at<double>(i,0) != 0)
      bb.height = i - bb.y;

  return bb;
}

class ImageConverter
{
public:
  ImageConverter() : image_trans_(nh_){

    image_pub_ = image_trans_.advertise("out", 1);
    image_sub_ = image_trans_.subscribe("image_raw", 1, &ImageConverter::image_callback, this);
    imageNum = 1;
    takeFlag = 0;

    // init opencv windows
    cv::namedWindow("Image window");
    cv::namedWindow("mask");
    cv::namedWindow("result");
	
    
    // init bgs engine
    //bgs = new FrameDifferenceBGS;
    //bgs = new StaticFrameDifferenceBGS;
    //bgs = new WeightedMovingMeanBGS;
    bgs = new WeightedMovingVarianceBGS;
    //bgs = new MixtureOfGaussianV1BGS;
    //bgs = new MixtureOfGaussianV2BGS;
    //bgs = new AdaptiveBackgroundLearning;
    //bgs = new GMG;

    
    // read back ground image
    bg = cv::imread("bg.png");
    cv::Mat img_mask;
    cv::Mat img_bkgmodel;
    bgs->process(bg, img_mask, img_bkgmodel);


    std::stringstream dlpath;
    dlpath << dirpath.str();
    dlpath << "/imageList.txt";
    // create data list
    dl.open(dlpath.str().c_str());

    dl << IMAGEMAX << std::endl;
  }

  ~ImageConverter()
  {
    dl.close();
    delete bgs;
    cv::destroyAllWindows();
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(imageNum > IMAGEMAX)
      exit(0);

    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);//cv_ptr型に変換.cv_ptr->imageがcv::Mat
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }

    //ここから処理
    cv::Mat inputImage(cv_ptr->image);
    cv::Mat img_mask;
    cv::Mat img_bkgmodel;
    cv::Mat dest = cv::Mat::zeros(inputImage.rows, inputImage.cols, inputImage.type());
    cv::Rect bb;
       
    cv::imshow("Image window", inputImage);	
    // calc bgs
    bgs->process(bg, img_mask, img_bkgmodel);
    bgs->process(inputImage, img_mask, img_bkgmodel);
	
    // erase gomashio noise
    cv::erode(img_mask, img_mask, cv::Mat(), cv::Point(), 5);
    cv::dilate(img_mask, img_mask, cv::Mat(), cv::Point(), 10);

    // calc bounding box
    img_mask.convertTo(img_mask,CV_64F);
    cv::Rect ashikiri(0,0,img_mask.cols, img_mask.rows - 170);
    std::cout << ashikiri << std::endl;
    cv::Mat ashikun = img_mask(ashikiri);	
    bb = getBB(ashikun);

    // calc view
    img_mask.convertTo(img_mask,inputImage.type());
    
    std::stringstream imageName;

    if(takeFlag){
      imageName << hName << "_" << color << "_" << dist << "_" << imageNum << "_";

      cv::imwrite(dirpath.str()+"/"+ imageName.str() + "crop.png", inputImage(bb));
      cv::imwrite(dirpath.str()+"/"+ imageName.str() + "maskcrop.png", img_mask(bb));

      dl << imageName.str() + "crop.png " << "nodata " << imageName.str() + "maskcrop.png " 
	 << color + "Jacket " << 0 << " " << 0 << " " << 0 << std::endl;
      std::cout << imageName.str() << std::endl;
      imageNum++;
    }

    cv::cvtColor(img_mask, img_mask, CV_GRAY2RGB);	
    std::cout << img_mask.type() << " " << img_mask.channels() << std::endl;
    dest = cv::min(inputImage, img_mask);
    //cv::rectangle(dest, bb, cv::Scalar(255, 0, 0), 3);


    cv::imshow("mask", img_mask);
    cv::imshow("result", dest(bb));
    //cv::imshow("bkg", img_bkgmodel);
    //}

	
    int key = cv::waitKey(1);

    switch(key){
    case 'q':
      exit(0);
      break;
	  
    case 't':
      cv::imwrite("test.png", inputImage);
      break;

    case 's':
      takeFlag = 1;
      break;

    case 'p':
      takeFlag = 0;
      break;

    defalut:
      break;
	
    }
    //ここまで処理

    //処理後の画像をpublish
    image_pub_.publish(cv_ptr->toImageMsg());//cv_ptr->toImageMsg()でsensor_msgs::Imageを返す
  }

  ros::NodeHandle nh_;
  image_transport::ImageTransport image_trans_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  IBGS *bgs;
  cv::Mat bg;
  int imageNum;
  int takeFlag;
  std::ofstream dl;

};

int main(int argc, char** argv)
{

  std::cout << "person name: ";
  std::cin >> hName;
  std::cout << "color of jacket: ";
  std::cin >> color;
  std::cout << "distance from robot: ";
  std::cin >> dist;

  dirpath << "dataset/" << hName << "/" << color << "/" << dist;

  std::string execstr = "mkdir -p ";
  execstr += dirpath.str();
  system( execstr.c_str() );

  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}











