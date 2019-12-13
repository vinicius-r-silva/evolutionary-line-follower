#include "headers/ag.h"

extern robot_pos *robotPos[TAM_POPULATION];

extern estacao estacao2robot[TAM_ESTACOES];

//Vetor de individuos (Populacao)
extern vector<robot_consts*> indiv;
extern int ind_next_robot;

//Vetor dos melhores individuos sera utilizado para a reproducao (Best)
extern robot_consts *indivBest[TAM_BEST];

extern double sumFitness;
extern double maxFitnessGen;
extern double maxFitnessTotal;
extern vector<double> maxFitnessVec;
extern vector<double> medFitnessVec;

double randomize(double inicio_range, double final_range, int casas_precisao){
  double div_value = pow(10, casas_precisao) * final_range;
  double total_ran = final_range - inicio_range;
  double randon = rand() % (int)(total_ran * div_value);
  return (randon / div_value) + inicio_range;
}


void initPopulation(bool allocar){
  for(int i = 0; i < TAM_POPULATION; i++){
    if(allocar)
      indiv[i] = (robot_consts*)malloc(sizeof(robot_consts));
    indiv[i]->v0=(int16_t)((float) MAX_VALUE_V0         * randomize(0, 1, 3));
    indiv[i]->linear_kp  = (float) MAX_VALUE_LINEAR_KP  * randomize(-1, 1, 3);
    indiv[i]->angular_kp = (float) MAX_VALUE_ANGULAR_KP * randomize(-1, 1, 3);
    reset_contadores(indiv[i]);
    indiv[i]->fitness = -1;
  }
}


void copyPop(vector<robot_consts*> *tempIndiv){
  for(int i = 0; i < TAM_POPULATION; i++){
    (*tempIndiv)[i] = (robot_consts*)malloc(sizeof(robot_consts));
    (*tempIndiv)[i]->v0= indiv[i]->v0;
    (*tempIndiv)[i]->linear_kp  = indiv[i]->linear_kp;
    (*tempIndiv)[i]->angular_kp = indiv[i]->angular_kp;
    (*tempIndiv)[i]->fitness = indiv[i]->fitness;
  }
}

void calc_fitness(int robot){
  double fitness = 0.0;
  double vel_med = (double) (indiv[robot]->distanciaPercorrida / indiv[robot]->framesTotal); 
  fitness += (double) (PESO_DISTANCIA * indiv[robot]->distanciaPercorrida);
  fitness += (double) (PESO_VEL_MED * vel_med);
  if(isinf(fitness))
    indiv[robot]->fitness = 0;
  else
    indiv[robot]->fitness = fitness;
}


void cross(robot_consts *pai, robot_consts *mae, robot_consts *filho){
  double mut_v0, mut_lin, mut_ang;
  mut_v0  = randomize(-0.025*MAX_VALUE_V0, 0.025*MAX_VALUE_V0, 4);
  mut_ang = randomize(-0.025*MAX_VALUE_ANGULAR_KP, 0.025*MAX_VALUE_ANGULAR_KP, 4);
  mut_lin = randomize(-0.025*MAX_VALUE_LINEAR_KP, 0.025*MAX_VALUE_LINEAR_KP, 4);
  reset_contadores(filho);

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
    filho->linear_kp = pai->linear_kp + mut_lin;
  } else {
    filho->linear_kp = mae->linear_kp + mut_lin;
  }
}

void initCross(){
  int i;
  int pai_index;
  int mae_index;
  int filho_index = TAM_BEST;

  for(i = 0; i < (TAM_POPULATION - TAM_BEST) / 2; i++){
      pai_index = rand() % TAM_BEST;
      mae_index = rand() % TAM_BEST;
      while(mae_index == pai_index){
        mae_index = rand() % TAM_BEST;
      }

      cross(indiv[pai_index], indiv[mae_index], indiv[filho_index]);
      filho_index++;
      cross(indiv[pai_index], indiv[mae_index], indiv[filho_index]);
      filho_index++;
  }


  ind_next_robot = TAM_BEST;
  for(i = 0; i < TAM_ESTACOES; i++){
    estacao2robot[i].robot_station = ind_next_robot;
    ind_next_robot++;
  }

  sumFitness = 0;
  maxFitnessGen = indiv[0]->fitness;
  for(i = 0; i < TAM_BEST; i++){
    sumFitness += indiv[i]->fitness;
  }
}

void bestFit(){
  int i = 0;
  double mut_v0, mut_lin, mut_ang;

  for(i = 1; i < TAM_POPULATION; i++){
    reset_contadores(indiv[i]);
    mut_v0  = randomize(-0.025*MAX_VALUE_V0, 0.025*MAX_VALUE_V0, 4);
    mut_ang = randomize(-0.025*MAX_VALUE_ANGULAR_KP, 0.025*MAX_VALUE_ANGULAR_KP, 4);
    mut_lin = randomize(-0.025*MAX_VALUE_LINEAR_KP, 0.025*MAX_VALUE_LINEAR_KP, 4);

    indiv[i]->v0 = (indiv[0]->v0 + indiv[i]->v0)/2 + mut_v0;
    indiv[i]->linear_kp = (indiv[0]->linear_kp + indiv[i]->linear_kp)/2 + mut_lin;
    indiv[i]->angular_kp = (indiv[0]->angular_kp + indiv[i]->angular_kp)/2 + mut_ang;
  }

  ind_next_robot = 1;
  for(i = 0; i < TAM_ESTACOES; i++){
    estacao2robot[i].robot_station = ind_next_robot;
    ind_next_robot++;
  }

  sumFitness = indiv[0]->fitness;
  maxFitnessGen = indiv[0]->fitness;
}




void torneio(){
  int i;
  int a, b, pai1, pai2;
  double mut_v0, mut_lin, mut_ang;
  vector<robot_consts*> tempIndiv(TAM_POPULATION);

  copyPop(&tempIndiv);

  for (i = 1; i < TAM_POPULATION; i++){
    // Sorteia dois individuos para 1ro torneio
    a = (int)(randomize(0, TAM_POPULATION, 0));
    b = (int)(randomize(0, TAM_POPULATION, 0));

    if (tempIndiv[a]->fitness > tempIndiv[b]->fitness)
        pai1 = a;
    else
        pai1 = b;

    // Sorteia mais dois individuos para 2do torneio
    a = (int)(randomize(0, TAM_POPULATION, 0));
    b = (int)(randomize(0, TAM_POPULATION, 0));

    if (tempIndiv[a]->fitness > tempIndiv[b]->fitness)
        pai2 = a;
    else
        pai2 = b;
    
    mut_v0  = randomize(-0.025*MAX_VALUE_V0, 0.025*MAX_VALUE_V0, 4);
    mut_ang = randomize(-0.025*MAX_VALUE_ANGULAR_KP, 0.025*MAX_VALUE_ANGULAR_KP, 4);
    mut_lin = randomize(-0.025*MAX_VALUE_LINEAR_KP, 0.025*MAX_VALUE_LINEAR_KP, 4);
    
    reset_contadores(indiv[i]);
    indiv[i]->v0 = (tempIndiv[pai1]->v0 + tempIndiv[pai2]->v0)/2 + mut_v0;
    indiv[i]->linear_kp = (tempIndiv[pai1]->linear_kp + tempIndiv[pai2]->linear_kp)/2 + mut_lin;
    indiv[i]->angular_kp = (tempIndiv[pai1]->angular_kp + tempIndiv[pai2]->angular_kp)/2 + mut_ang;
  }

  ind_next_robot = 1;
  for(i = 0; i < TAM_ESTACOES; i++){
    estacao2robot[i].robot_station = ind_next_robot;
    ind_next_robot++;
  }

  sumFitness = indiv[0]->fitness;
  maxFitnessGen = indiv[0]->fitness;
}

bool check_kill_indiv(int robot){
  return  indiv[robot]->tempoNoQuadrante > MAX_FRAMES_POR_QUADRANTE ||
          indiv[robot]->framesPerdidos > MAX_FRAMES_SEM_LINHA ||
          indiv[robot]->qtdQuadrantes > 4 ||
          indiv[robot]->qtdQuadrantes < 0;
}

bool isGenerationEnded(){
  int i = 0;
  for(i = 0; i < TAM_ESTACOES; i++){
    if(estacao2robot[i].robot_station != -1)
      return false;
  }

  return true;
}


void atualizar_dist(int robot, int estacao, int quadrante, int posX, int posY, bool terminou_volta){
  double dist = 0;

  if(indiv[robot]->ultimoQuadrante != quadrante){
    switch (indiv[robot]->ultimoQuadrante){
      case 1: dist = 35; break;
      case 2: dist = 60; break;
      case 3: dist = 40; break;
      case 4: dist = 15; break;
    }
  }else if(!terminou_volta){
    double X = estacao2robot[estacao].quadrante[quadrante].posX;
    double Y = estacao2robot[estacao].quadrante[quadrante].posY;
    double dx2 = pow((posX - X), 2);
    double dy2 = pow((posY - Y), 2);
    dist = 10.0 * sqrt(dx2 + dy2);
  }
  
  if(isinf(dist) || indiv[robot]->qtdQuadrantes < 0 || dist < 30)
    dist = 0;

  indiv[robot]->distanciaPercorrida += dist;

}

void reset_contadores(robot_consts *ind_robot){
  ind_robot->framesPerdidos   = 0;
  ind_robot->qtdQuadrantes    = 0;
  ind_robot->tempoNoQuadrante = 0;
  ind_robot->framesTotal       = 0;
  ind_robot->ultimoQuadrante   = 1;
  ind_robot->distanciaPercorrida = 0;
}