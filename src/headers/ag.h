#ifndef AG_H
#define AG_H

#include "globals.h"

#define PESO_DISTANCIA  0.8
#define PESO_TEMPO_VIVO 0.2
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
void atualizar_dist(int robot, float posX, float posY, float newPosX, float newPosY);


//Realiza o cross(cruzamento de cromossomos) de 2 individuos best para formar 6 indivios novos
void cross(robot_consts *pai, robot_consts *mae, robot_consts **filhos);


//retorna valor entre inicio_range e final_range com precisao de x casas 
double randomize(int inicio_range, int final_range, int casas_precisao);

//check if generation was come to a end
bool isGenerationEnded();

//Reset values default
void reset_contadores(robot_consts *ind_robot);

#endif