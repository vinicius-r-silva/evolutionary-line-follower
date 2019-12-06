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

#define TAM_POPULATION 32
#define TAM_BEST 3

#define LIMIT_VALUE_V0 250
#define LIMIT_VALUE_LINEAR_KP 10
#define LIMIT_VALUE_ANGULAR_KP 220

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
  int16_t Ve;
  uint8_t Vd;
} robot_vel;

typedef struct{
  uint8_t v0;
  float linear_kp;
  float angular_kp;
  uint8_t qtdQuadrantes;     //Soma 1 toda vez que avança quadrante (2 voltas), sub 1 toda vez que volta quadrante
  uint8_t ultimoQuarante;    //
  uint8_t maxQtdQuadrante;

  uint64_t tempoNoQuadrante; //Soma 
  uint64_t framesPerdidos;   //Soma 1 toda vez que existe um frame sem linha, reseta quando encontra linha

} robot_consts;

cv::Mat canny;
cv::Mat raw_img;
cv::Mat fliped_img;
cv::Mat HLines_img;

ros::Publisher speed_pub;
std_msgs::UInt8MultiArray msg;

robot_pos robotPos;


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

void initBestPopulation(robot_consts **indivBest);
void initPopulation(robot_consts **indiv);

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
  result.Ve = 0;
  result.Vd = 0;
  return result;
}

//----------------------------------------------MAIN---------------------------------------------//
int main(int argc, char **argv){
  ros::init(argc, argv, "main");

  robot_consts *indv[TAM_POPULATION];
  robot_consts *best_ind[TAM_BEST];

  initPopulation(indv);
  initBestPopulation(best_ind);

  cout<< best_ind[0]->v0 << endl;;




  namedWindow("Probabilistic", WINDOW_AUTOSIZE); // Create Window


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

  ROS_INFO("%d", best_ind[0]->v0);
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

  robot_consts consts = {0,0.00,-5};
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
  else{

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

  //pega estacao
  //ve se morreu
  //se morreu, troca numero da estacao

}


//Inicio AG
//Iniciando vetores de inidividuos(populacao) e de melhores(best)
//returns -1 <= x <= 1
double randomize(){
  double randon = rand() % 2000;
  return (randon/1000) - 1;
}

void initBestPopulation(robot_consts **indivBest){
  srand(time(0));
  for(int i = 0; i < TAM_BEST; i++){
    indivBest[i] = (robot_consts*)malloc(sizeof(robot_consts));
    indivBest[i]->v0 = (int) (((double)LIMIT_VALUE_V0) * randomize());
    indivBest[i]->linear_kp = (float) LIMIT_VALUE_LINEAR_KP * randomize();
    indivBest[i]->angular_kp = (float) LIMIT_VALUE_ANGULAR_KP * randomize();
    ROS_INFO("%f", randomize());
  }
}

void initPopulation(robot_consts **indiv){
  for(int i = 0; i < TAM_BEST; i++){
    indiv[i] = (robot_consts*)malloc(sizeof(robot_consts));
    indiv[i]->v0 = (int) 0.0;
    indiv[i]->linear_kp = 0.0;
    indiv[i]->angular_kp = 0.0;
  }
}

void calc_fitness(){
}

void cross(){
}

