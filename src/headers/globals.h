#ifndef GLOBALS_H
#define GLOBALS_H

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

#define TAM_ESTACOES 6

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
  int quadrante;
} robot_pos;

typedef struct{
  uint8_t Ve;
  uint8_t Vd;
} robot_vel;

//struct individuo
typedef struct{
  //Cromossomo
  int16_t v0;       //Velocidade
  float linear_kp;  //Quao perto o individuo se mantem no centro da linha
  float angular_kp; //Quao sensivel o individuo realiza as curvas

  //Check kill
  uint8_t qtdQuadrantes;    //Soma 1 toda vez que avança quadrante (2 voltas), sub 1 toda vez que volta quadrante
  uint8_t ultimoQuarante;   //Ultimo quadrante que o individuo estava (para comparacao)
  uint8_t maxQtdQuadrante;  //Valor do maior quadrante que o individuo chegou
  uint64_t tempoNoQuadrante;  //Soma frames no mesmo quadrante, reseta apenas qndo maxQtdQuadrante e' atualizado
  uint64_t framesPerdidos;    //Soma 1 toda vez que existe um frame sem linha, reseta quando encontra linha
  
  //Calculo do fitness
  uint64_t tempoTotal;          //Soma total de frames (equiv ao tempoTotal)
  uint64_t distanciaPercorrida; //Soma do calculo das pequenas distancias pto a pto 
  uint64_t fitness; //Contem o fitness do individuo para ordenacao e escolher os individuos para reproducao

} robot_consts; 



#endif