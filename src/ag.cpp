#include "headers/ag.h"

extern robot_pos *robotPos[TAM_POPULATION];

extern estacao estacao2robot[TAM_ESTACOES];

//Vetor de individuos (Populacao)
extern vector<robot_consts*> indiv;
extern int pos_indv_atual;

//Vetor dos melhores individuos sera utilizado para a reproducao (Best)
extern robot_consts *indivBest[TAM_BEST];


double randomize(double inicio_range, double final_range, int casas_precisao){
  double div_value = pow(10, casas_precisao) * final_range;
  double total_ran = final_range - inicio_range;
  double randon = rand() % (int)(total_ran * div_value);
  return (randon / div_value) + inicio_range;
}


void initPopulation(){
  for(int i = 0; i < TAM_POPULATION; i++){
    indiv[i] = (robot_consts*)malloc(sizeof(robot_consts));
    indiv[i]->v0=(int16_t)((float) MAX_VALUE_V0         * randomize(-1, 1, 3));
    indiv[i]->linear_kp  = (float) MAX_VALUE_LINEAR_KP  * randomize(-1, 1, 3);
    indiv[i]->angular_kp = (float) MAX_VALUE_ANGULAR_KP * randomize(-1, 1, 3);
    reset_contadores(indiv[i]);
  }
}


void calc_fitness(int robot){
  double fitness = 0.0;
  double vel_med = (double) (indiv[robot]->distanciaPercorrida / indiv[robot]->framesTotal); 
  fitness += (double) (PESO_DISTANCIA * indiv[robot]->distanciaPercorrida);
  fitness += (double) (PESO_VEL_MED * vel_med);
  indiv[robot]->fitness = fitness;
}


void makeSon(robot_consts *pai, robot_consts *mae, robot_consts *filho){
  double mut_v0, mut_lin, mut_ang;
  mut_v0  = randomize(-0.025*MAX_VALUE_V0, 0.025*MAX_VALUE_V0, 4);
  mut_ang = randomize(-0.025*MAX_VALUE_LINEAR_KP, 0.025*MAX_VALUE_LINEAR_KP, 4);
  mut_lin = randomize(-0.025*MAX_VALUE_ANGULAR_KP, 0.025*MAX_VALUE_ANGULAR_KP, 4);

  int parentSum = 0;
  int parentChoice = (int) randomize(-10, 10, 0);
  if(parentChoice < 0){
    filho->v0 = pai->v0 + mut_v0;
    parentSum++;
  } else {
    filho->v0 = mae->v0 + mut_v0;
    parentSum--;
  }

  parentChoice = (int) randomize(-10, 10, 0);
  if(parentChoice < 0){
    filho->angular_kp = pai->angular_kp + mut_ang;
    parentSum++;
  } else {
    filho->angular_kp = mae->angular_kp + mut_ang;
    parentSum--;
  }

  if(parentSum == 2)
    parentChoice = -10;
  else if(parentSum == -2)
    parentChoice = 10;
  else
    parentChoice = (int) randomize(-10, 10, 0);
    
  if(parentChoice < 0){
    filho->angular_kp = pai->angular_kp + mut_ang;
    parentSum++;
  } else {
    filho->angular_kp = mae->angular_kp + mut_ang;
    parentSum--;
  }
  reset_contadores(filho);
}

void cross(robot_consts *pai, robot_consts *mae, robot_consts **filhos){
  makeSon(pai, mae, filhos[0]);
  makeSon(pai, mae, filhos[1]);
}


bool check_kill_indiv(int robot){
  return  indiv[robot]->tempoNoQuadrante > MAX_FRAMES_POR_QUADRANTE ||
          indiv[robot]->framesPerdidos > MAX_FRAMES_SEM_LINHA ||
          indiv[robot]->qtdQuadrantes == 8;
}

bool isGenerationEnded(){
  int i = 0;
  for(i = 0; i < TAM_ESTACOES; i++){
    if(estacao2robot[i].robot_station != -1)
      return false;
  }

  return true;
}


void atualizar_dist(int robot, int quadrante, int posX, int posY){
  double dist = 0;
  if(indiv[robot]->ultimoQuarante != quadrante){ //Nao morreu
    switch (quadrante){
      case 1: dist = 35; break;
      case 2: dist = 60; break;
      case 3: dist = 40; break;
      case 4: dist = 15; break;
    }
  }else{
    double X = estacao2robot->quadrante[quadrante].posX;
    double Y = estacao2robot->quadrante[quadrante].posY;
    double dx2 = pow((posX - X), 2);
    double dy2 = pow((posY - Y), 2);
    dist = sqrt(dx2 + dy2);
  }

  indiv[robot]->distanciaPercorrida += dist;

}

void reset_contadores(robot_consts *ind_robot){
  ind_robot->framesPerdidos   = 0;
  ind_robot->maxQtdQuadrante  = 0;
  ind_robot->qtdQuadrantes    = 0;
  ind_robot->tempoNoQuadrante = 0;
  ind_robot->framesTotal       = 0;
  ind_robot->ultimoQuarante   = 1;
  ind_robot->distanciaPercorrida = 0;
  ind_robot->fitness          = -1;
}