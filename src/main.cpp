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

#define TAM_POPULATION 36
#define TAM_BEST 12
#define CHANCE_MUTACAO 4

#define MAX_VALUE_V0 250
#define MAX_VALUE_LINEAR_KP 10
#define MAX_VALUE_ANGULAR_KP 220

#define MAX_FRAMES_POR_QUADRANTE 0
#define MAX_FRAMES_SEM_LINHA 0

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

//struct individuo
typedef struct{
  
  //Cromossomo
  uint8_t v0;       //Velocidade
  float linear_kp;  //Quao perto o individuo se mantem no centro da linha
  float angular_kp; //Quao sensivel o individuo realiza as curvas

  //Check kill
  uint8_t qtdQuadrantes;    //Soma 1 toda vez que avança quadrante (2 voltas), sub 1 toda vez que volta quadrante
  uint8_t ultimoQuarante;   //Ultimo quadrante que o individuo estava (para comparacao)
  uint8_t maxQtdQuadrante;  //Valor do maior quadrante que o individuo chegou
  uint64_t tempoNoQuadrante;  //Soma frames no mesmo quadrante, reseta apenas qndo maxQtdQuadrante e' atualizado
  uint64_t framesPerdidos;    //Soma 1 toda vez que existe um frame sem linha, reseta quando encontra linha
  
  //Calculo do fitness
  uint64_t framesTotal;         //Soma total de frames (equiv ao tempoTotal)
  uint64_t distanciaPercorrida; //Soma do calculo das pequenas distancias pto a pto 
  uint64_t fitness; //Contem o fitness do individuo para ordenacao e escolher os individuos para reproducao

} robot_consts; 

cv::Mat canny;
cv::Mat raw_img;
cv::Mat fliped_img;
cv::Mat HLines_img;

ros::Publisher speed_pub;
std_msgs::UInt8MultiArray msg;

robot_pos robotPos;

//Vetor de individuos (Populacao)
robot_consts *indv[TAM_POPULATION];
int pos_indv_atual;

//Vetor dos melhores individuos sera utilizado para a reproducao (Best)
robot_consts *best_ind[TAM_BEST];



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


//AG
//Inicia a populacao de individuos
void initBestPopulation(robot_consts **indivBest);


//Inicia randomicamente a populacao de melhores
void initPopulation(robot_consts **indiv);


//Verifica se individuo deve morrer
bool check_kill_indiv(robot_consts *indiv);


//Calcula o fitness do individuo
void calc_fitness(robot_consts *indiv);


//Realiza o cross(cruzamento de cromossomos) de 2 individuos best para formar 4 indivios novos
void cross(robot_consts *pai, robot_consts *mae, robot_consts **filhos);


//retorna valor entre inicio_range e final_range com precisao de x casas 
double randomize(int inicio_range, int final_range, int casas_precisao);


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
  srand(time(0));

  initPopulation(indv);
  initBestPopulation(best_ind);

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

  ROS_INFO("%.3f", best_ind[0]->angular_kp);
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

  uint8_t station = 1;//pega estacao do individuo
  bool kill_indv = check_kill_indiv(indv[pos_indv_atual]); //ver se deve morrer
  
  if(kill_indv){
    //reset_robot();
    calc_fitness(indv[pos_indv_atual++]); // calcula o fitness do robo atual (que morreu)
    
    //se morreu, troca numero da estacao
    if(pos_indv_atual < TAM_POPULATION){
      //marcar pos_indv_atual na estacao
      //iniciar indv[pos_indv_atual]
    }else{
      //marcar -1 na estacao
    }
  }
}


double randomize(int inicio_range, int final_range, int casas_precisao){
  int div_value = pow(10, casas_precisao) * final_range;
  int total_ran = final_range - inicio_range;
  double randon = rand() % (total_ran * div_value);
  return (randon / div_value) + inicio_range;
}


void initBestPopulation(robot_consts **indivBest){
  for(int i = 0; i < TAM_BEST; i++){
    indivBest[i] = (robot_consts*)malloc(sizeof(robot_consts));
    indivBest[i]->v0=(uint8_t)((float) MAX_VALUE_V0         * randomize(-1, 1, 3));
    indivBest[i]->linear_kp  = (float) MAX_VALUE_LINEAR_KP  * randomize(-1, 1, 3);
    indivBest[i]->angular_kp = (float) MAX_VALUE_ANGULAR_KP * randomize(-1, 1, 3);
  }
}


void initPopulation(robot_consts **indiv){
  for(int i = 0; i < TAM_BEST; i++){
    indiv[i] = (robot_consts*)malloc(sizeof(robot_consts));
    indiv[i]->v0         = 0;
    indiv[i]->linear_kp  = 0.0;
    indiv[i]->angular_kp = 0.0;
  }
}


void calc_fitness(robot_consts *indiv){
  //calc fitness do sublime - ver regra de fitness
  indiv->fitness = 0;
}


void cross(robot_consts *pai, robot_consts *mae, robot_consts **filhos){
  int rand = randomize(1, 100, 0);  //ver mutacao
  filhos[0]->v0         = pai->v0;
  filhos[0]->linear_kp  = pai->linear_kp;
  filhos[0]->angular_kp = mae->angular_kp;
  
  rand = randomize(1, 100, 0);      //ver mutacao
  filhos[1]->v0         = pai->v0;
  filhos[1]->linear_kp  = mae->linear_kp;
  filhos[1]->angular_kp = mae->angular_kp;

  rand = randomize(1, 100, 0);      //ver mutacao
  filhos[2]->v0         = mae->v0;
  filhos[2]->linear_kp  = pai->linear_kp;
  filhos[2]->angular_kp = pai->angular_kp;

  rand = randomize(1, 100, 0);      //ver mutacao
  filhos[2]->v0         = mae->v0;
  filhos[2]->linear_kp  = mae->linear_kp;
  filhos[2]->angular_kp = pai->angular_kp;
}


bool check_kill_indiv(robot_consts *indiv){
  //ind
  return false;
}