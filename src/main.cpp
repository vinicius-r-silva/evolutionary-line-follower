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

#include "headers/globals.h"
#include "headers/image_processing.h"
#include "headers/robot_control.h"
#include "headers/ag.h"

using namespace std;
using namespace cv;


//-------------------------------------------GLOBALS--------------------------------------------//
cv::Mat canny;
cv::Mat raw_img;
cv::Mat fliped_img;
cv::Mat HLines_img;

ros::Publisher *reset_pub;
ros::Publisher *speed_pub;
std_msgs::UInt8MultiArray msg;

robot_pos *robotPos[TAM_POPULATION];

int estacao2robot[TAM_ESTACOES];

//Vetor de individuos (Populacao)
robot_consts *indiv[TAM_POPULATION];
int pos_indv_atual;

//Vetor dos melhores individuos sera utilizado para a reproducao (Best)
robot_consts *indivBest[TAM_BEST];

//------------------------------------------------------FUNCTIONS------------------------------------------------------//

//callback from robot_pos topic
//updates the current positon of the robot
void getPosition_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);


//--------------------------------------------------------MAIN--------------------------------------------------------//
int main(int argc, char **argv){
  int i = 0;
  ros::init(argc, argv, "main");

  srand(time(0));
  initPopulation(indiv);
  initBestPopulation(indivBest);
  
  ROS_INFO("Iniciar Individuos Geracao\n");

  robot_consts **vet_aux;
  for(int j = 0; j < 6; j++){
    vet_aux = (robot_consts**) malloc(6 * sizeof(robot_consts*));
    vet_aux[0] = indiv[6*j];
    vet_aux[1] = indiv[(6*j)+1];
    vet_aux[2] = indiv[(6*j)+2];
    vet_aux[3] = indiv[(6*j)+3];
    vet_aux[4] = indiv[(6*j)+4];
    vet_aux[5] = indiv[(6*j)+5];
    indiv[36 + (2*j)] = indivBest[2*j];
    indiv[37 + (2*j)] = indivBest[(2*j)+1];
    cross(indivBest[2*j], indivBest[(2*j)+1], vet_aux);
    free(vet_aux);
  }

  for(int i = 0; i < TAM_POPULATION; i++){
    ROS_INFO("INDIV[%d]: %d | %.2f | %.2f\n",i, indiv[i]->v0, indiv[i]->linear_kp, indiv[i]->angular_kp);    
  }

  char windowName[] = "estacaoX";
  for(i = 0; i < TAM_ESTACOES; i++){
    windowName[7] = i + '0';
    namedWindow(windowName, WINDOW_AUTOSIZE); // Create Window
    moveWindow(windowName, 20 +(i%2) * 310,20 + (i/2)*200);
  }

  pos_indv_atual = 1;
  for(i = 0; i < TAM_ESTACOES; i++){
    estacao2robot[i] = i;
    pos_indv_atual++;
  }

  ros::NodeHandle n;
  for(i = 0; i < TAM_POPULATION; i++){
    robotPos[i] = (robot_pos*)malloc(sizeof(robot_pos));
    robotPos[i]->x = 0;
    robotPos[i]->y = 0;
    robotPos[i]->theta = 0;
  }

  ros::Subscriber image_sub = n.subscribe("image", 10, getImage_callback);  //subscrive to \image topic
  ros::Subscriber position_sub = n.subscribe("robot_pos", 10, getPosition_callback);  //subscrive to \robot_pos topic (gets the robot current position)
  

  char speed_pub_topic[] = "robotX_vel";
  char reset_pub_topic[] = "robotX_reset";
  reset_pub = (ros::Publisher*)calloc(TAM_ESTACOES, sizeof(ros::Publisher));
  speed_pub = (ros::Publisher*)calloc(TAM_ESTACOES, sizeof(ros::Publisher));
  for(i = 0; i < TAM_ESTACOES; i++){
    speed_pub_topic[5] = i + '0';
    reset_pub_topic[5] = i + '0';
    speed_pub[i] = n.advertise<std_msgs::UInt8MultiArray>(speed_pub_topic, 10); //create publisher to \robot_vel topic (sets robot motor's velocity)
    reset_pub[i] = n.advertise<std_msgs::Bool>(reset_pub_topic, 10);
  }

  //create the message to carry the robot motor's velocity
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = 2;
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label = "robot_velocity";

  ros::spin();
  return 0;
}


//------------------------------------------------------FUNCTIONS------------------------------------------------------//

//callback from robot_pos topic
//updates the current positon of the robot
void getPosition_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
  int estacao = msg->data[0]; //pega estacao do individuo
  int robot = estacao2robot[estacao];

  if(robot == -1)
    return;

  if(estacao != 0)
    return;

  int quadrante = msg->data[1];

  robotPos[robot]->x = msg->data[2];
  robotPos[robot]->y = msg->data[3];
  robotPos[robot]->theta = msg->data[4];
  robotPos[robot]->quadrante = quadrante;


  // sendSpeed()
  reset_robot(0);
  
  if(check_kill_indiv(robot)){
    reset_robot(estacao);
    calc_fitness(indiv[pos_indv_atual]); // calcula o fitness do robo atual (que morreu)
    
    //calcular somente os que ainda nao possuem fitness
    // while(pos_indv_atual < TAM_POPULATION && indiv[pos_indv_atual]->fitness != -1){
    //   pos_indv_atual++;
    // }

    //se morreu, troca numero da estacao
    if(pos_indv_atual < TAM_POPULATION){
      estacao2robot[estacao] = pos_indv_atual;
      pos_indv_atual++;
    }else{
      estacao2robot[estacao] = -1;
    }
  }
}