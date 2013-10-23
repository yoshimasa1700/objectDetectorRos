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

namespace enc = sensor_msgs::image_encodings;

class ImageConverter
{
public:
  ImageConverter() : image_trans_(nh_){
    
    conf.loadConfig("config.xml");
    conf.demoMode = 1;
    forest = NULL;
    forest = new CRForest(conf);
    forest->loadForest();

    image_pub_ = image_trans_.advertise("out", 1);
    image_sub_ = image_trans_.subscribe("image_raw", 1, &ImageConverter::image_callback, this);

    // init opencv windows
    cv::namedWindow("Image window");
    //cv::namedWindow("mask");
    //cv::namedWindow("result");



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

    seqImg.img.push_back(&inputImage);
    detectR = forest->detection(seqImg);

    
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











