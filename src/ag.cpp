#include "headers/ag.h"

extern robot_pos *robotPos[TAM_POPULATION];

extern int estacao2robot[TAM_ESTACOES];

//Vetor de individuos (Populacao)
extern robot_consts *indiv[TAM_POPULATION];
extern int pos_indv_atual;

//Vetor dos melhores individuos sera utilizado para a reproducao (Best)
extern robot_consts *indivBest[TAM_BEST];


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
  for(int i = 0; i < TAM_POPULATION; i++){
    indiv[i] = (robot_consts*)malloc(sizeof(robot_consts));
    indiv[i]->v0         = 0;
    indiv[i]->linear_kp  = 0.0;
    indiv[i]->angular_kp = 0.0;
  }
}


void calc_fitness(int robot){
  float fitness = 0.0;
  fitness += PESO_DISTANCIA * indiv[robot]->distanciaPercorrida;
  fitness += PESO_TEMPO_VIVO * (indiv[robot]->tempoTotal / TX_FPS);
  indiv[robot]->fitness = fitness;
}


void cross(robot_consts *pai, robot_consts *mae, robot_consts **filhos){
  int rand;
  int mut_v0, mut_lin, mut_ang;

  //2 pai x 1 mae
  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 3)) : 0;
  filhos[0]->v0         = pai->v0         + mut_v0;
  filhos[0]->linear_kp  = pai->linear_kp  + mut_lin;
  filhos[0]->angular_kp = mae->angular_kp + mut_ang;
  
  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 3)) : 0;
  filhos[1]->v0         = pai->v0         + mut_v0;
  filhos[1]->linear_kp  = mae->linear_kp  + mut_lin;
  filhos[1]->angular_kp = pai->angular_kp + mut_ang;

  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 3)) : 0;
  filhos[2]->v0         = mae->v0         + mut_v0;
  filhos[2]->linear_kp  = pai->linear_kp  + mut_lin;
  filhos[2]->angular_kp = pai->angular_kp + mut_ang;
  
  //1 pai x 2 mae
  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 3)) : 0;
  filhos[3]->v0         = mae->v0         + mut_v0;
  filhos[3]->linear_kp  = mae->linear_kp  + mut_lin;
  filhos[3]->angular_kp = pai->angular_kp + mut_ang;

  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 3)) : 0;
  filhos[4]->v0         = mae->v0         + mut_v0;
  filhos[4]->linear_kp  = pai->linear_kp  + mut_lin;
  filhos[4]->angular_kp = mae->angular_kp + mut_ang;

  rand = randomize(1, 100, 0);  //Chance de mutacao
  mut_v0  = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_V0, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_LINEAR_KP, 3)) : 0;
  mut_lin = (rand <= CHANCE_MUTACAO) ? (randomize(0, 0.1*MAX_VALUE_ANGULAR_KP, 3)) : 0;
  filhos[5]->v0         = pai->v0         + mut_v0;
  filhos[5]->linear_kp  = mae->linear_kp  + mut_lin;
  filhos[5]->angular_kp = mae->angular_kp + mut_ang;

}


bool check_kill_indiv(int robot){
  if(indiv[robot]->tempoTotal > MAX_FRAMES_POR_QUADRANTE || indiv[robot]->framesPerdidos > MAX_FRAMES_SEM_LINHA){
    // return true;
  }
  return false;
}


void atualizar_dist(int robot, float posX, float posY, float newPosX, float newPosY){
  float dx = posX - newPosX;
  float dy = posY - newPosY;
  indiv[robot]->distanciaPercorrida += pow(dx, 2);
  indiv[robot]->distanciaPercorrida += pow(dy, 2);
}