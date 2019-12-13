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
int ind_next_robot;

int generation;

int sameKing;
double mutationValue;

double sumFitness;
double maxFitnessGen;
double maxFitnessTotal;
vector<double> maxFitnessVec;
vector<double> medFitnessVec;

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
  generation = 0;

  ros::init(argc, argv, "main");

  srand(time(0));
  initPopulation();

  char windowName[] = "estacaoX";
  for(i = 0; i < TAM_ESTACOES; i++){
    windowName[7] = i + '0';
    namedWindow(windowName, WINDOW_AUTOSIZE); // Create Window
    moveWindow(windowName, 20 +(i%2) * 310,20 + (i/2)*200);
  }
  namedWindow("graph", WINDOW_AUTOSIZE); // Create Window
  moveWindow("graph", 640,20);

  ind_next_robot = 0;
  for(i = 0; i < TAM_ESTACOES; i++){
    estacao2robot[i].robot_station = i;
    estacao2robot[i].id_quadrante = 1;
    estacao2robot[i].robotLinha = false;
    ini_quadrantes(i);
    ind_next_robot++;
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

  sumFitness = 0;
  maxFitnessGen = 0;
  maxFitnessTotal = 0;

  mutationValue = INITIAL_MUTATION;
  ros::spin();
  return 0;
}


bool compareFitness(robot_consts* robot1, robot_consts* robot2){
  return robot1->fitness > robot2->fitness;
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
  double fitness;
  
  robotPos[robot]->x = posX;
  robotPos[robot]->y = posY;
  robotPos[robot]->theta = msg->data[4];

  //contadores
  indiv[robot]->tempoNoQuadrante++;
  indiv[robot]->framesTotal++;

  bool movimento_re = robotPos[robot]->quadrante == 1 && quadrante > 2;

  //bool cond_quadrante0 = estacao2robot[estacao].robotLinha;
  bool cond_quadrante0 = true;
  bool cond_quadrante1 = robotPos[robot]->quadrante < quadrante && !movimento_re; //robot avancou quadrante 
  bool cond_quadrante2 = robotPos[robot]->quadrante == 4 && quadrante == 1;       //robot terminou volta
  bool cond_quadrante3 = robotPos[robot]->quadrante > quadrante || movimento_re;  //cond retroceder
  bool terminou_volta = false;

  if(cond_quadrante0 && (cond_quadrante1 || cond_quadrante2)){
    atualizar_dist(robot, 0, quadrante, 0.0, 0.0, false);
    
    robotPos[robot]->quadrante = quadrante;
    estacao2robot[estacao].id_quadrante = quadrante;
    
    indiv[robot]->ultimoQuadrante = quadrante;
    indiv[robot]->tempoNoQuadrante = 0;
    indiv[robot]->qtdQuadrantes++;
    if(indiv[robot]->qtdQuadrantes == 4){
      terminou_volta = true;
    }
  }else if(cond_quadrante3){
    indiv[robot]->qtdQuadrantes--;
  }

  if(check_kill_indiv(robot) || movimento_re){
    atualizar_dist(robot, estacao, quadrante, posX, posY, terminou_volta);
    calc_fitness(robot);
    
    //Debug
    ROS_INFO("Robot[%02d] Estacao:%d Quad:%d Dist:%.2f Time:%ld Fit:%.2f", robot, estacao, quadrante, indiv[robot]->distanciaPercorrida, indiv[robot]->framesTotal, indiv[robot]->fitness);
    fitness = indiv[robot]->fitness;
    
    if(!isinf(fitness)){
      sumFitness += fitness;
      if(fitness > maxFitnessGen){
        maxFitnessVec.push_back(fitness);
        maxFitnessGen = fitness;

        if(fitness > maxFitnessTotal)
          maxFitnessTotal = fitness;

        sameKing = 0;
        updateFitnessGraph();
      }

      if(fitness == 0){
        reset_contadores(robot);
        reset_consts(robot);
      }
    }
    
    if(ind_next_robot < TAM_POPULATION){
      estacao2robot[estacao].robot_station = ind_next_robot;
      ind_next_robot++;
    }else{
      estacao2robot[estacao].robot_station = -1;
    }
    reset_robot(estacao);

    if(isGenerationEnded()){
      generation++;
      ROS_INFO("----------------- Generation %d -----------------", generation);
      
      std::sort(indiv.begin(), indiv.end(), compareFitness);

      maxFitnessVec.push_back(PLOT_NEW_GENERATION);
      medFitnessVec.push_back(sumFitness/TAM_POPULATION);
      updateFitnessGraph();

      if (sameKing){
        mutationValue += MUTATION_STEP;

        if(mutationValue > MUTATION_MAX)
          mutationValue = MUTATION_MAX;

        ROS_INFO("same King, mutation: %lf", mutationValue);
      }
      else{
        mutationValue = INITIAL_MUTATION;
      }

      if(indiv[0]->fitness > 0){
        initCross();
        // bestFit();
        // torneio();
      }
      else{ 
        ROS_INFO("reseting pop");
        ind_next_robot = 0;
        for(i = 0; i < TAM_ESTACOES; i++){
          estacao2robot[i].robot_station = ind_next_robot;
          ind_next_robot++;
        }

        // for(i = 0; i < TAM_POPULATION; i++){
        //   reset_contadores(i);
        //   reset_consts(i);
        // }

        sumFitness = 0;
        maxFitnessGen = 0;
      }

      sameKing = 1;

      ROS_INFO("\n\n----------------------------------New Gen: MaxFit: %lf\n", maxFitnessGen);
    }
  }
}