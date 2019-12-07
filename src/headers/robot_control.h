#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "globals.h"

//sends a message to the robot be reset
void reset_robot(int estacao);

//given the left and right motors velocities, send it
void sendSpeed(robot_vel robotVel, int estacao);

robot_vel getMotorsVelocity(delta error, robot_consts consts);

#endif