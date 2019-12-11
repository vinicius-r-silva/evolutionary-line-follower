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

estacao estacao2robot[TAM_ESTACOES];

//Vetor de individuos (Populacao)
vector<robot_consts*> indiv(TAM_POPULATION);
int pos_indv_atual;

//Vetor dos melhores individuos sera utilizado para a reproducao (Best)
//vector<robot_consts*> indivBest(TAM_BEST);

//------------------------------------------------------FUNCTIONS------------------------------------------------------//

//callback from robot_pos topic
//updates the current positon of the robot
void getPosition_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);


//--------------------------------------------------------MAIN--------------------------------------------------------//
int main(int argc, char **argv){
  ROS_INFO("INIT LINE FOLLOWER");
  
  int i = 0;
  ros::init(argc, argv, "main");

  srand(time(0));
  initPopulation();

  char windowName[] = "estacaoX";
  for(i = 0; i < TAM_ESTACOES; i++){
    windowName[7] = i + '0';
    namedWindow(windowName, WINDOW_AUTOSIZE); // Create Window
    moveWindow(windowName, 20 +(i%2) * 310,20 + (i/2)*200);
  }

  pos_indv_atual = 1;
  for(i = 0; i < TAM_ESTACOES; i++){
    estacao2robot[i].robot_station = i;
    estacao2robot[i].id_quadrante = 0;
    ini_quadrantes(i);
    pos_indv_atual++;
  }

  ros::NodeHandle n;
  for(i = 0; i < TAM_POPULATION; i++){
    robotPos[i] = (robot_pos*)malloc(sizeof(robot_pos));
    robotPos[i]->x = 0;
    robotPos[i]->y = 0;
    robotPos[i]->theta = 0;
    robotPos[i]->quadrante = 1;
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


bool compareFitness(robot_consts* robot1, robot_consts* robot2){
  return robot1->fitness < robot2->fitness;
}

//------------------------------------------------------FUNCTIONS------------------------------------------------------//

//callback from robot_pos topic
//updates the current positon of the robot
void getPosition_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){

  int i = 0;
  int estacao = msg->data[0];
  int robot = estacao2robot[estacao].robot_station;

  if(robot == -1)
    return;
  
  int quadrante = msg->data[1];
  float posX = msg->data[2];
  float posY = msg->data[3];

  robotPos[robot]->x = posX;
  robotPos[robot]->y = posY;
  robotPos[robot]->theta = msg->data[4];

  //contadores
  indiv[robot]->tempoNoQuadrante++;
  indiv[robot]->framesTotal++;
  
  bool atualizar_quadrante = robotPos[robot]->quadrante < quadrante || (robotPos[robot]->quadrante == 4 && quadrante == 1);
  if(atualizar_quadrante){
    atualizar_dist(robot, quadrante, 0.0, 0.0);
    
    robotPos[robot]->quadrante = quadrante;
    estacao2robot[estacao].id_quadrante = quadrante;
    
    indiv[robot]->ultimoQuarante = quadrante;
    indiv[robot]->maxQtdQuadrante = quadrante;
    indiv[robot]->tempoNoQuadrante = 0;
    indiv[robot]->qtdQuadrantes++;

  }else if(robotPos[robot]->quadrante > quadrante){
    indiv[robot]->qtdQuadrantes--;
  }

  if(check_kill_indiv(robot)){
    atualizar_dist(robot, quadrante, posX, posY);
    calc_fitness(robot);
    ROS_INFO("Estacao:%d Robot[%02d] Dist:%4.2f Time:%04ld Fit:%4.2f",estacao, robot, indiv[robot]->distanciaPercorrida,indiv[robot]->framesTotal, indiv[robot]->fitness);

    if(pos_indv_atual < TAM_POPULATION){
      estacao2robot[estacao].robot_station = pos_indv_atual;
      pos_indv_atual++;
    }else{
      estacao2robot[estacao].robot_station = -1;
    }
    reset_robot(estacao);

    if(isGenerationEnded()){
      std::sort(indiv.begin(), indiv.end(), compareFitness);
      int qtd_pais = (TAM_POPULATION - TAM_BEST) / RAZAO_PAIS_FILHOS;

      robot_consts *vet_aux[RAZAO_PAIS_FILHOS];
      int pai_index;
      int mae_index;
      
      for(int j = 0; j < qtd_pais; j++){
        for(int k = 0; k < RAZAO_PAIS_FILHOS; k++){
          vet_aux[k] = indiv[TAM_BEST + (RAZAO_PAIS_FILHOS*j)+k];
        }
        pai_index = rand() % TAM_BEST;
        mae_index = rand() % TAM_BEST;
        while(mae_index == pai_index){
          mae_index = rand() % TAM_BEST;
        }
        cross(indiv[pai_index], indiv[mae_index], vet_aux);
      }

      pos_indv_atual = TAM_BEST;
      for(i = 0; i < TAM_ESTACOES; i++){
        estacao2robot[i].robot_station = pos_indv_atual;
        pos_indv_atual++;
      }
    }
  }
}