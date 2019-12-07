#include "headers/robot_control.h"

extern ros::Publisher *reset_pub;
extern ros::Publisher *speed_pub;
extern std_msgs::UInt8MultiArray msg;

extern robot_pos *robotPos[TAM_POPULATION];

//sends a message to the robot be reset
void reset_robot(int estacao){
  std_msgs::Bool resetMsg;
  resetMsg.data = true;
  reset_pub[estacao].publish(resetMsg);   
}

//given the left and right motors velocities, send it
void sendSpeed(robot_vel robotVel, int estacao){
  msg.data.clear();                   //creates the msg...
  msg.data.push_back(robotVel.Ve);
  msg.data.push_back(robotVel.Vd);
  speed_pub[estacao].publish(msg);  //send it
}

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

  //cout << "Ve: " << Ve << ",  Vd: " << Vd << ",  linear: " << consts.linear_kp * error.shift << ",  angular" << consts.angular_kp * error.angle << endl;

  result.Ve = Ve;
  result.Vd = Vd;
  result.Ve = 0;
  result.Vd = 0;
  return result;
}