#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/UInt8MultiArray.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

//--------------------------------------------DEFINES--------------------------------------------//
#define HoughLineTH 40
#define ERROR -1000


//--------------------------------------------GLOBALS--------------------------------------------//
typedef struct{
  float shift;
  float angle;
} delta;

typedef struct{
  float x;
  float y;
  float theta;
} robot_pos;

typedef struct{
  uint8_t Ve;
  uint8_t Vd;
} robot_vel;

cv::Mat canny;
cv::Mat raw_img;
cv::Mat fliped_img;
cv::Mat HLines_img;

ros::NodeHandle n;
ros::Publisher pub;
std_msgs::UInt8MultiArray msg;


//-------------------------------------------FUNCTIONS--------------------------------------------//
//sends a message to the robot be reset
void reset_robot();


//giving a vector of lines, choose the better one to be followed
//the best line is the where the first coordiante is the futher away from the image
Vec4i chooseLine(vector<Vec4i> linesP);


//given a line, find the robot error
//the erro is given by the line's angle in rads and by where the line touchs the botton of the image
delta getError(Vec4i line);


//given the left and right motors velocities, send it
void sendSpeed(robot_vel robotVel);


//callback from getImage topic
//receives the image from the robot onboard camera
//detects the black line in the image, if there is any
void getImage_callback(const sensor_msgs::Image::ConstPtr& msg);



//----------------------------------------------MAIN---------------------------------------------//
int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  namedWindow("Probabilistic", WINDOW_AUTOSIZE); // Create Window

  ros::Subscriber sub = n.subscribe("image", 1, getImage_callback); //subscrive to \image topic
  pub = n.advertise<std_msgs::UInt8MultiArray>("robot_vel", 1);     //create publisher to \robot_vel topic (sets robot motor's velocity)

  //create the message to carry the robot motor's velocity
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = 2;
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label = "robot_velocity";

  ros::spin();
  return 0;
}


//-------------------------------------------FUNCTIONS--------------------------------------------//
//sends a message to the robot be reset
void reset_robot(){
  ros::Publisher reset_pub;
  reset_pub = n.advertise<std_msgs::Bool>("robot_reset", 1);

  std_msgs::Bool msg;
  msg.data = true;
  reset_pub.publish(msg);   
}


//giving a vector of lines, choose the better one to be followed
//the best line is the where the first coordiante is the futher away from the image
Vec4i chooseLine(vector<Vec4i> linesP){
  Vec4i result;
  result[0] = linesP[0][0];
  result[1] = linesP[0][1];
  result[2] = linesP[0][2];
  result[3] = linesP[0][3];

  Vec4i l;
  for( size_t i = 0; i < linesP.size(); i++ )
  {
      l = linesP[i];
      if(l[0] < result[0]){
        result[0] = l[0];
        result[1] = l[1];
        result[2] = l[2];
        result[3] = l[3];
      }
  }

  return result;
}


//given a line, find the robot error
//the erro is given by the line's angle in rads and by where the line touchs the botton of the image
delta getError(Vec4i line){
  delta result = {ERROR,ERROR}; //init the error variable
  int X1 = line[1];             //sets the point's values
  int X2 = line[3];
  int Y1 = line[0];
  int Y2 = line[2];
  int dx = (Y2 - Y1);
  int dy = (X2 - X1);
  float a = 0;
  float b = 0;

  if(fabs(dx) < 0.00001)       //if dx is to low, the following calculation wont work, thus return error
    return result;

  a = (float)dy / (float)dx;   //gets the angular coeficient
  b = (Y2*X1 - Y1*X2)/dx;      //gets the linear coeficient

  result.angle = atan2(dy , -dx); //calculates the angle of the line
  if(result.angle < 0)            
    result.angle += M_PI;         //if the angle is negative, make it positive

  result.shift = (128-b)/a - 128; //calculates where the line touch the botton of the image (y = a*x + b, where y = 128)

  //for debugging, print the error
  cout << "a: " << a  << ",  b: " << b << ",  angle: " << result.angle << ", shift: " << result.shift << endl;
  return result;
}


//given the left and right motors velocities, send it
void sendSpeed(robot_vel robotVel){
  msg.data.clear();                //creates the msg...
  msg.data.push_back(robotVel.Ve);
  msg.data.push_back(robotVel.Vd);
  pub.publish(msg);                //send it
}


//callback from getImage topic
//receives the image from the robot onboard camera
//detects the black line in the image, if there is any
void getImage_callback(const sensor_msgs::Image::ConstPtr& msg){
  raw_img =  cv_bridge::toCvShare(msg, "bgr8")->image; //get the image

  robot_vel robotVel = {0,0};                //later used to send the robot motor's speed
  delta     robot_error;

  flip(raw_img,fliped_img, 0);               //since the raw image is fliped, the unflip it
  Canny(fliped_img, canny, 50, 200, 3);      //apply canny filter to after aply the HoughLines algorithm
  cvtColor(canny,HLines_img,COLOR_GRAY2BGR); //HLines_img is the image where is going to b draw the lines detected in the canny image

  vector<Vec4i> linesP;                      //stores all the detected lines
  Vec4i choosenLine;                         //store the image choosen to be followed by the robot
  HoughLinesP(canny, linesP, 1, CV_PI/180, HoughLineTH, 30, 10 ); //apply the HoughLines algorithm to detect the lines inside the image

  if(linesP.size() > 0){                     //if there is at least one line detect, then...
    choosenLine = chooseLine(linesP);        //choose the better line to be followed (gives preference to the line followed in the previously frame)
    line( HLines_img, Point(choosenLine[0], choosenLine[1]), Point(choosenLine[2], choosenLine[3]), Scalar(255,0,0), 3, LINE_AA); //print the choosen line
    robot_error = getError(choosenLine);     //given the choosen line, get the robot error

    if(robot_error.angle != ERROR){
      sendSpeed(robotVel);                     //send the motor speed to the robot
    }
  }

  imshow("Probabilistic", HLines_img);       //shows the image with the choosen line printed on it
  waitKey(1);
}