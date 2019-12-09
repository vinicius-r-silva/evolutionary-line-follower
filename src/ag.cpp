#include "headers/ag.h"

extern robot_pos *robotPos[TAM_POPULATION];

extern int estacao2robot[TAM_ESTACOES];

//Vetor de individuos (Populacao)
extern vector<robot_consts*> indiv;
extern int pos_indv_atual;

//Vetor dos melhores individuos sera utilizado para a reproducao (Best)
extern robot_consts *indivBest[TAM_BEST];


double randomize(int inicio_range, int final_range, int casas_precisao){
  int div_value = pow(10, casas_precisao) * final_range;
  int total_ran = final_range - inicio_range;
  double randon = rand() % (total_ran * div_value);
  return (randon / div_value) + inicio_range;
}


void initPopulation(){
  for(int i = 0; i < TAM_POPULATION; i++){
    indiv[i] = (robot_consts*)malloc(sizeof(robot_consts));
    indiv[i]->v0=(int16_t)((float) MAX_VALUE_V0         * randomize(-1, 1, 3));
    indiv[i]->linear_kp  = (float) MAX_VALUE_LINEAR_KP  * randomize(-1, 1, 3);
    indiv[i]->angular_kp = (float) MAX_VALUE_ANGULAR_KP * randomize(-1, 1, 3);
  }
}


void calc_fitness(int robot){
  float fitness = 0.0;
  fitness += PESO_DISTANCIA * indiv[robot]->distanciaPercorrida;
  fitness += PESO_TEMPO_VIVO * (indiv[robot]->tempoTotal / TX_FPS);
  indiv[robot]->fitness = fitness;
}


void makeSon(robot_consts *pai, robot_consts *mae, robot_consts *filho){
  double mut_v0, mut_lin, mut_ang;
  mut_v0  = randomize(-0.025*MAX_VALUE_V0, 0.025*MAX_VALUE_V0, 4);
  mut_ang = randomize(-0.025*MAX_VALUE_LINEAR_KP, 0.025*MAX_VALUE_LINEAR_KP, 4);
  mut_lin = randomize(-0.025*MAX_VALUE_ANGULAR_KP, 0.025*MAX_VALUE_ANGULAR_KP, 4);

  int parentSum = 0;
  int parentChoice = randomize(-10, 10, 0);
  if(parentChoice < 0){
    filho->v0 = pai->v0 + mut_v0;
    parentSum++;
  } else {
    filho->v0 = mae->v0 + mut_v0;
    parentSum--;
  }

  parentChoice = randomize(-10, 10, 0);
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
    parentChoice = randomize(-10, 10, 0);
    
  if(parentChoice < 0){
    filho->angular_kp = pai->angular_kp + mut_ang;
    parentSum++;
  } else {
    filho->angular_kp = mae->angular_kp + mut_ang;
    parentSum--;
  }
}

void cross(robot_consts *pai, robot_consts *mae, robot_consts **filhos){
  makeSon(pai, mae, filhos[0]);
  makeSon(pai, mae, filhos[1]);
}


bool check_kill_indiv(int robot){
  if(indiv[robot]->tempoTotal > MAX_FRAMES_POR_QUADRANTE || indiv[robot]->framesPerdidos > MAX_FRAMES_SEM_LINHA){
    // return true;
  }
  return false;
}

bool isGenerationEnded(){
  int i = 0;
  for(i = 0; i < TAM_ESTACOES; i++){
    if(estacao2robot[i] != -1)
      return false;
  }

  return true;
}


void atualizar_dist(int robot, float posX, float posY, float newPosX, float newPosY){
  float dx = posX - newPosX;
  float dy = posY - newPosY;
  indiv[robot]->distanciaPercorrida += pow(dx, 2);
  indiv[robot]->distanciaPercorrida += pow(dy, 2);
}