#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

cv::Mat raw_img;
cv::Mat fliped_img;

using namespace std;
using namespace cv;

int alpha_slider;
int alpha_slider_max = 200;
int th = 50;

static void on_trackbar( int, void* )
{
   th = alpha_slider;
}


void getImage(const sensor_msgs::Image::ConstPtr& msg)
{
  Mat cds, cdst, cdstP; 
  raw_img =  cv_bridge::toCvShare(msg, "bgr8")->image;
  // if(raw_img.empty())
  //   return; 

  // cvtColor(raw_img,fliped_img,CV_BGR2GRAY);   
  // if(fliped_img.empty())
  //   return; 
  flip(raw_img,fliped_img, 0);
  // string ty =  type2str( fliped_img.type() );
  // printf("Matrix: %s %dx%d \n", ty.c_str(), fliped_img.cols, fliped_img.rows );
  // imshow("orginal", fliped_img);
  // waitKey(1);
  // return;
  // cds = fliped_img.clone();
  // Mat test(500, 1000, CV_8UC3, Scalar(0,0, 100));
  // Mat test2(500, 1000, CV_8UC1, Scalar(100));    // Edge detection
  Canny(fliped_img, cds, 50, 200, 3);
  cvtColor(cds,cdst,COLOR_GRAY2BGR);   
  // Canny(fliped_img, cds, 50, 200, 3);
  // cvtColor(fliped_img,cds,CV_BGR2GRAY);   
  // cdst  = cds.clone();
  cdstP = cdst.clone();


  // Standard Hough Line Transform
  vector<Vec2f> lines; // will hold the results of the detection
  HoughLines(cds, lines, 1, CV_PI/180, th, 0, 0 ); // runs the actual detection
  // // Draw the lines
  for( size_t i = 0; i < lines.size(); i++ )
  {
      float rho = lines[i][0], theta = lines[i][1];
      Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      line( cdst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
  }
  // // Probabilistic Line Transform
  vector<Vec4i> linesP; // will hold the results of the detection
  HoughLinesP(cds, linesP, 1, CV_PI/180, th, 50, 10 ); // runs the actual detection
  // Draw the lines
  for( size_t i = 0; i < linesP.size(); i++ )
  {
      Vec4i l = linesP[i];
      line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
  }

  // Show results
  imshow("orginal", fliped_img);
  imshow("Detected Lines - Standard Hough Line Transform", cdst);    
  imshow("Probabilistic", cdstP);

  waitKey(1);

  //ROS_INFO("I heard: [%s]", msg->data.data());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  namedWindow("Probabilistic", WINDOW_AUTOSIZE); // Create Window

  char TrackbarName[50];
  sprintf( TrackbarName, "Alpha x %d", alpha_slider_max );
  createTrackbar( TrackbarName, "Probabilistic", &alpha_slider, alpha_slider_max, on_trackbar );
  on_trackbar( alpha_slider, 0 );

  ros::Subscriber sub = n.subscribe("image", 10, getImage);
  ros::spin();

  return 0;
}

