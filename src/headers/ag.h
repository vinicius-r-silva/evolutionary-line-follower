#ifndef AG_H
#define AG_H

#include "globals.h"

#define PESO_DISTANCIA  6
#define PESO_VEL_MED    4
#define TX_FPS 30

//Inicia a populacao de individuos
void initBestPopulation();


//Inicia randomicamente a populacao de melhores
void initPopulation();


//Verifica se individuo deve morrer
bool check_kill_indiv(int robot);


//Calcula o fitness do individuo - Media Ponderada
void calc_fitness(int robot);


//Atualiza a distancia percorrida
void atualizar_dist(int robot, int estacao, int quadrante, int posX, int posY, bool terminou_volta);


//Realiza o cross(cruzamento de cromossomos) de 2 individuos best para formar 1 individuo novo
void cross(robot_consts *pai, robot_consts *mae, robot_consts *filho);

//make cross 
void initCross();

void bestFit();

void torneio();

//retorna valor entre inicio_range e final_range com precisao de x casas 
double randomize(double inicio_range, double final_range, int casas_precisao);

//check if generation was come to a end
bool isGenerationEnded();

//Reset values default
void reset_contadores(robot_consts *ind_robot);


#endif