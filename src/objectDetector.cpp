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

#define graphRatio 2.0

class ImageConverter
{
public:
  ImageConverter() : image_trans_(nh_){
    
    std::cerr << boost::filesystem::current_path() << std::endl;

    th.clear();

    std::ifstream ifs("threshold.txt");

    for(int i = 0; i < 3; ++i){
      double tth;
      ifs >> tth;
      th.push_back(tth);
    }

    ifs.close();

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
  }

  ~ImageConverter()
  {
    cv::destroyAllWindows();
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg){

    std::cout<< msg->header.stamp<<std::endl;
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
    
    int face[] = {cv::FONT_HERSHEY_SIMPLEX, cv::FONT_HERSHEY_PLAIN, cv::FONT_HERSHEY_DUPLEX, cv::FONT_HERSHEY_COMPLEX, 
		  cv::FONT_HERSHEY_TRIPLEX, cv::FONT_HERSHEY_COMPLEX_SMALL, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 
		  cv::FONT_HERSHEY_SCRIPT_COMPLEX, cv::FONT_ITALIC};


    CTestDataset seqImg;
    CDetectionResult detectR;

    cv::Mat resizeImg;

    cv::resize(inputImage, resizeImg, 
	       cv::Size(inputImage.cols/graphRatio, inputImage.rows/graphRatio), 0 , 0);

    seqImg.img.push_back(&resizeImg);
    detectR = forest->detection(seqImg);

    objectDetector::Detect detectResult;
    objectDetector::Pos p_blue, p_orange, p_sign;
 
    detectResult.header = msg->header;

    std::vector<bool> fP(3,0);
    bool ff = 0;


    detectResult.s_blue = detectR.detectedClass[0].score;
    if(detectR.detectedClass.at(0).score > th[0]){
      cv::Point bluej = detectR.detectedClass.at(0).centerPoint;
      bluej = bluej * graphRatio;
      cv::circle(inputImage, bluej, 5,cv::Scalar(255,0,0),2);
      cv::putText(inputImage, "Blue", cv::Point(10,30), face[4]|face[8], 1.2, cv::Scalar(255,0,0), 2, CV_AA);
      fP[0] = 1;
      detectResult.Blue = 1;
      p_blue.x = bluej.x;
      p_blue.y = bluej.y;
    }

    detectResult.s_orange = detectR.detectedClass[1].score;
    if(detectR.detectedClass.at(1).score > th[1]){
      cv::Point orangej = detectR.detectedClass.at(1).centerPoint;
      orangej = orangej * graphRatio;
      cv::circle(inputImage, orangej, 5,cv::Scalar(0,69,255),2);
      cv::putText(inputImage, "Orange", cv::Point(10,80), face[4]|face[8], 1.2, cv::Scalar(0,69,255), 2, CV_AA);
      fP[1] = 1;
      detectResult.Orange = 1;
      p_orange.x = orangej.x;
      p_orange.y = orangej.y;
    }

    detectResult.s_signboard = detectR.detectedClass[2].score;
    if(detectR.detectedClass.at(2).score > th[2]){
      cv::Point sbord = detectR.detectedClass.at(2).centerPoint;
      sbord = sbord * graphRatio;
      cv::circle(inputImage, sbord * 2, 5,cv::Scalar(0,255,255),2);
      cv::putText(inputImage, "Signboard", cv::Point(10,130), face[4]|face[8], 1.2, cv::Scalar(0,255,255), 2, CV_AA);
      fP[2] = 1;
      detectResult.Signboard = 1;
      p_sign.x = sbord.x;
      p_sign.y = sbord.y;
    }

    if((fP[0] || fP[1]) && fP[2]){
      ff = 1;
      detectResult.Found = 1;
      cv::rectangle(inputImage, cv::Point(), cv::Point(inputImage.cols, inputImage.rows), cv::Scalar(255,0,0), 10);
    }

    detectResult.p_blue = p_blue;
    detectResult.p_orange = p_orange;
    detectResult.p_signboard = p_sign;

    detect_pub.publish(detectResult);


    cv::imshow("Image window", inputImage);	
	
    int key = cv::waitKey(1);

    switch(key){
    case 'q':
      exit(0);
      break;
	  
    case 't':
      cv::imwrite("test.png", inputImage);
      break;

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

    cv_ptr->image = inputImage;
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  ros::NodeHandle nh_;
  image_transport::ImageTransport image_trans_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher detect_pub;

  CRForest *forest;
  CConfig conf;

  std::vector<double> th;

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











