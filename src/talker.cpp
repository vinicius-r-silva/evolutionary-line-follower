#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#define HoughLineTH 40
#define ERROR -1000

typedef struct{
  float shift;
  float angle;
} delta;

cv::Mat canny;
cv::Mat raw_img;
cv::Mat fliped_img;
cv::Mat HLines_img;

using namespace std;
using namespace cv;


Vec4i chooseLine(vector<Vec4i> linesP){
  Vec4i result;
  result[0] = 1000;
  result[1] = 1000;
  result[2] = 1000;
  result[3] = 1000;

  for( size_t i = 0; i < linesP.size(); i++ )
  {
      Vec4i l = linesP[i];
      if(l[0] < result[0]){
        result[0] = l[0];
        result[1] = l[1];
        result[2] = l[2];
        result[3] = l[3];
      }
  }

  return result;
}

delta getError(Vec4i line){
  delta result = {ERROR,ERROR};
  float dx = line[0] - line[2];
  float dy = line[1] - line[3];
  if(dx < 0.0001)
    return result;

  result.angle = atan(dy / dx);
  result.shift = line[1] - line[0]*result.angle;
  cout << "angle: " << result.angle << ", shift: " << result.shift << endl;
  return result;
}

void getImage(const sensor_msgs::Image::ConstPtr& msg)
{
  raw_img =  cv_bridge::toCvShare(msg, "bgr8")->image;

  flip(raw_img,fliped_img, 0);
  Canny(fliped_img, canny, 50, 200, 3);
  cvtColor(canny,HLines_img,COLOR_GRAY2BGR);   

  // Probabilistic Line Transform
  vector<Vec4i> linesP; 
  Vec4i choosenLine;
  HoughLinesP(canny, linesP, 1, CV_PI/180, HoughLineTH, 30, 10 );
  if(linesP.size() > 0){
    choosenLine = chooseLine(linesP);
    line( HLines_img, Point(choosenLine[0], choosenLine[1]), Point(choosenLine[2], choosenLine[3]), Scalar(255,0,0), 3, LINE_AA);
    getError(choosenLine);  
  }


  // for( size_t i = 0; i < linesP.size(); i++ )
  // {
  //     Vec4i l = linesP[i];
  //     if(l[0] != choosenLine[0] && l[1] != choosenLine[1] && l[2] != choosenLine[2] && l[3] != choosenLine[3])
  //       line( HLines_img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
  // }

  // Show results
  // imshow("orginal", fliped_img);
  imshow("Probabilistic", HLines_img);
  waitKey(1);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  namedWindow("Probabilistic", WINDOW_AUTOSIZE); // Create Window

  ros::Subscriber sub = n.subscribe("image", 10, getImage);
  ros::spin();

  return 0;
}

