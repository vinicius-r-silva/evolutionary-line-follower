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

//--------------------------------------------DEFINES--------------------------------------------//
#define HoughLineTH 40
#define ERROR -1000

#define MAX_SPEED 40

#define QUADRANT_1 1
#define QUADRANT_2 2
#define QUADRANT_3 3
#define QUADRANT_4 4

#define TAM_POPULATION 48 //36 filhos + 12 best(nao eh necessario rodar dnovo os 12 best pois ja posuem fitness)
#define TAM_BEST 12
#define CHANCE_MUTACAO 5

#define MAX_VALUE_V0 250
#define MAX_VALUE_LINEAR_KP 10
#define MAX_VALUE_ANGULAR_KP 220

#define LIMIT_FRAMES_POR_QUADRANTE 0
#define LIMIT_FRAMES_SEM_LINHA 0

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


//-------------------------------------------FUNCTIONS--------------------------------------------//

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


//Realiza o cross(cruzamento de cromossomos) de 2 individuos best para formar 6 individuos novos
void cross(robot_consts *pai, robot_consts *mae, robot_consts **filhos);


//retorna valor entre inicio_range e final_range com precisao de x casas 
double randomize(float inicio_range, float final_range, int casas_precisao);


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


//-------------------------------------------------------MAIN--------------------------------------------------------//
int main(int argc, char **argv){
  int i = 0;
  ros::init(argc, argv, "main");

  srand(time(0));
  initPopulation(indiv);
  initBestPopulation(indivBest);
  
  ROS_INFO("INICIAR INDIVIDUOS\n");

  robot_consts **vet;
  for(int j = 0; j < 6; j++){
    vet = (robot_consts**) malloc(6 * sizeof(robot_consts*));
    vet[0] = indiv[6*j];
    vet[1] = indiv[(6*j)+1];
    vet[2] = indiv[(6*j)+2];
    vet[3] = indiv[(6*j)+3];
    vet[4] = indiv[(6*j)+4];
    vet[5] = indiv[(6*j)+5];
    indiv[36 + (2*j)] = indivBest[2*j];
    indiv[37 + (2*j)] = indivBest[(2*j)+1];
    cross(indivBest[2*j], indivBest[(2*j)+1], vet);
    free(vet);
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

  for(i = 0; i < TAM_ESTACOES; i++){
    estacao2robot[i] = i;
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

  //ROS_INFO("%.3f", indivBest[0]->angular_kp);
  ros::spin();
  return 0;
}


//-------------------------------------------------------FUNCTIONS-------------------------------------------------------//


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
  bool kill_indv = check_kill_indiv(indiv[pos_indv_atual]); //ver se deve morrer
  
  if(kill_indv){
    //reset_robot();
    calc_fitness(indiv[pos_indv_atual]); // calcula o fitness do robo atual (que morreu)
    
    //calcular somente os que ainda nao possuem fitness
    while(pos_indv_atual < TAM_POPULATION && indiv[pos_indv_atual]->fitness != -1){
      pos_indv_atual++;
    }

    //se morreu, troca numero da estacao
    if(pos_indv_atual < TAM_POPULATION){
      //marcar pos_indv_atual na estacao
      //iniciar indv[pos_indv_atual]
    }else{
      //marcar -1 na estacao
    }
  }
}


double randomize(float inicio_range, float final_range, int casas_precisao){
  int div_value = final_range * pow(10, casas_precisao);
  int tot_range = final_range - inicio_range;

  double randon = rand() % (tot_range * div_value);
  
  return (randon / div_value) + inicio_range;
}


void initBestPopulation(robot_consts **indivBest){
  for(int i = 0; i < TAM_BEST; i++){
    indivBest[i] = (robot_consts*)malloc(sizeof(robot_consts));
    indivBest[i]->v0=(uint8_t)((float) MAX_VALUE_V0         * randomize(-1, 1, 3));
    indivBest[i]->linear_kp  = (float) MAX_VALUE_LINEAR_KP  * randomize(-1, 1, 3);
    indivBest[i]->angular_kp = (float) MAX_VALUE_ANGULAR_KP * randomize(-1, 1, 3);
    indivBest[i]->fitness    = -1;
  }
}


void initPopulation(robot_consts **indiv){
  for(int i = 0; i < TAM_POPULATION; i++){
    indiv[i] = (robot_consts*)malloc(sizeof(robot_consts));
    indiv[i]->v0         = 0;
    indiv[i]->linear_kp  = 0.0;
    indiv[i]->angular_kp = 0.0;
    indiv[i]->fitness    = -1;
  }
}


void calc_fitness(robot_consts *indiv){
  //calc fitness do sublime - ver regra de fitness
  indiv->fitness = 0;
}


void cross(robot_consts *pai, robot_consts *mae, robot_consts **filhos){
  int rand;
  int mut_v0, mut_lin, mut_ang;

  //2 pai x 1 mae
  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 0)) : 0;
  filhos[0]->v0         = pai->v0         + mut_v0;
  filhos[0]->linear_kp  = pai->linear_kp  + mut_lin;
  filhos[0]->angular_kp = mae->angular_kp + mut_ang;
  
  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 0)) : 0;
  filhos[1]->v0         = pai->v0         + mut_v0;
  filhos[1]->linear_kp  = mae->linear_kp  + mut_lin;
  filhos[1]->angular_kp = pai->angular_kp + mut_ang;

  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 0)) : 0;
  filhos[2]->v0         = mae->v0         + mut_v0;
  filhos[2]->linear_kp  = pai->linear_kp  + mut_lin;
  filhos[2]->angular_kp = pai->angular_kp + mut_ang;
  
  //1 pai x 2 mae
  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 0)) : 0;
  filhos[3]->v0         = mae->v0         + mut_v0;
  filhos[3]->linear_kp  = mae->linear_kp  + mut_lin;
  filhos[3]->angular_kp = pai->angular_kp + mut_ang;

  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 0)) : 0;
  filhos[4]->v0         = mae->v0         + mut_v0;
  filhos[4]->linear_kp  = pai->linear_kp  + mut_lin;
  filhos[4]->angular_kp = mae->angular_kp + mut_ang;

  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 0)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 0)) : 0;
  filhos[5]->v0         = pai->v0         + mut_v0;
  filhos[5]->linear_kp  = mae->linear_kp  + mut_lin;
  filhos[5]->angular_kp = mae->angular_kp + mut_ang;

}


bool check_kill_indiv(robot_consts *indiv){
  
  if(indiv->framesPerdidos >= LIMIT_FRAMES_SEM_LINHA || indiv->tempoNoQuadrante >= LIMIT_FRAMES_POR_QUADRANTE){
       return true;
  }

  return false;
}