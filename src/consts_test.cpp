#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

//--------------------------------------------DEFINES--------------------------------------------//
#define HoughLineTH 40
#define ERROR -1000

#define MAX_SPEED 40

#define QUADRANT_1 1
#define QUADRANT_2 2
#define QUADRANT_3 3
#define QUADRANT_4 4

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

typedef struct{
  uint8_t v0;
  float linear_kp;
  float angular_kp;
} robot_consts;

cv::Mat canny;
cv::Mat raw_img;
cv::Mat fliped_img;
cv::Mat HLines_img;

ros::Publisher speed_pub;
std_msgs::UInt8MultiArray msg;

robot_pos robotPos;


const int alpha_slider_v0_max = 252;
const int alpha_slider_linear_max = 10000;
const int alpha_slider_angular_max = 10000;
int alpha_slider_v0;
int alpha_slider_linear;
int alpha_slider_angular;

static void on_trackbar_v0( int, void* )
{
}

static void on_trackbar_linear( int, void* )
{
}

static void on_trackbar_angular( int, void* )
{
}



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


//callback from robot_pos topic
//updates the current positon of the robot
void getPosition_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

robot_vel getMotorsVelocity(delta error, robot_consts consts){
  robot_vel result;
  float errorSum = consts.linear_kp * error.shift  +  consts.angular_kp * error.angle;

  float Ve = consts.v0 + errorSum;
  float Vd = consts.v0 - errorSum;

  if(Ve > MAX_SPEED)
    Ve = MAX_SPEED;
  else if(Ve < -MAX_SPEED)
    Ve = -MAX_SPEED;

  if(Vd > MAX_SPEED)
    Vd = MAX_SPEED;
  else if(Vd < -MAX_SPEED)
    Vd = -MAX_SPEED;

  cout << "Ve: " << Ve << ",  Vd: " << Vd << ",  linear: " << consts.linear_kp * error.shift << ",  angular" << consts.angular_kp * error.angle << endl;

  result.Ve = Ve;
  result.Vd = Vd;
  return result;
}

int getQuadrant(){
  
  return QUADRANT_1;
}

bool isTheRobotInReverse(){


  return false;
}

//----------------------------------------------MAIN---------------------------------------------//
int main(int argc, char **argv){
  ros::init(argc, argv, "main");
  // namedWindow("Probabilistic", WINDOW_AUTOSIZE); // Create Window
  namedWindow("Probabilistic", WINDOW_NORMAL); // Create Window
  resizeWindow("Probabilistic", 500, 500); // Create Window


  ros::NodeHandle n;
  robotPos.x = 0;
  robotPos.y = 0;
  robotPos.theta = 0;

  ros::Subscriber image_sub = n.subscribe("image", 1, getImage_callback);  //subscrive to \image topic
  ros::Subscriber position_sub = n.subscribe("robot_pos", 1, getPosition_callback);  //subscrive to \robot_pos topic (gets the robot current position)
  speed_pub = n.advertise<std_msgs::UInt8MultiArray>("robot_vel", 1);     //create publisher to \robot_vel topic (sets robot motor's velocity)

  //create the message to carry the robot motor's velocity
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = 2;
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label = "robot_velocity";


  alpha_slider_v0 = 126;
  char TrackbarName_v0[50];
  sprintf( TrackbarName_v0, "Alpha v0 x %d", alpha_slider_v0_max );
  createTrackbar( TrackbarName_v0, "Probabilistic", &alpha_slider_v0, alpha_slider_v0_max, on_trackbar_v0 );
  on_trackbar_v0( alpha_slider_v0, 0 );

  alpha_slider_linear = 5000;
  char TrackbarName_linear[50];
  sprintf( TrackbarName_linear, "Alpha linear x %d", alpha_slider_linear_max );
  createTrackbar( TrackbarName_linear, "Probabilistic", &alpha_slider_linear, alpha_slider_linear_max, on_trackbar_linear );
  on_trackbar_linear( alpha_slider_linear, 0);

  alpha_slider_angular = 5000;
  char TrackbarName_angular[50];
  sprintf( TrackbarName_angular, "Alpha angular x %d", alpha_slider_angular_max );
  createTrackbar( TrackbarName_angular, "Probabilistic", &alpha_slider_angular, alpha_slider_angular_max, on_trackbar_angular );
  on_trackbar_angular( alpha_slider_angular, 0 );


  ros::spin();
  return 0;
}


//-------------------------------------------FUNCTIONS--------------------------------------------//
//sends a message to the robot be reset
void reset_robot(){
  ros::NodeHandle n;
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

  result.angle -= M_PI/2;
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
  speed_pub.publish(msg);                //send it
}


//callback from getImage topic
//receives the image from the robot onboard camera
//detects the black line in the image, if there is any
void getImage_callback(const sensor_msgs::Image::ConstPtr& msg){
  raw_img =  cv_bridge::toCvShare(msg, "bgr8")->image; //get the image

  robot_consts consts = {(uint8_t)(alpha_slider_v0-126),((float)(alpha_slider_linear-5000))/100,((float)(alpha_slider_angular-5000))/100};
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
      robotVel = getMotorsVelocity(robot_error, consts);
      sendSpeed(robotVel);                     //send the motor speed to the robot
    }
  }

  imshow("Probabilistic", HLines_img);       //shows the image with the choosen line printed on it
  waitKey(1);
}


//callback from robot_pos topic
//updates the current positon of the robot
void getPosition_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
  robotPos.x = msg->data[0];
  robotPos.y = msg->data[1];
  robotPos.theta = msg->data[2];
}