#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include "globals.h"
#include "robot_control.h"

//callback from getImage topic
//receives the image from the robot onboard camera
//detects the black line in the image, if there is any
void getImage_callback(const sensor_msgs::Image::ConstPtr& msg);

//given a line, find the robot error
//the erro is given by the line's angle in rads and by where the line touchs the botton of the image
delta getError(Vec4i line);

//giving a vector of lines, choose the better one to be followed
//the best line is the where the first coordiante is the futher away from the image
Vec4i chooseLine(vector<Vec4i> linesP);

#endif