#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"


#define TAM_ESTACOES 6

using namespace std;

typedef struct{
  uint8_t Ve;
  uint8_t Vd;
} robot_vel;

ros::Publisher *reset_pub;
ros::Publisher *speed_pub;
std_msgs::UInt8MultiArray msg;

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

void getPosition_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    int i;
    robot_vel velocidades = {0,0};
    for(i = 0; i < TAM_ESTACOES; i++){
        sendSpeed(velocidades, i);
        reset_robot(i);
        sendSpeed(velocidades, i);
    }
}
//--------------------------------------------------------MAIN--------------------------------------------------------//
int main(int argc, char **argv){
    int i;
    ROS_INFO("INIT RESET ROBOTS");
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    char speed_pub_topic[] = "robotX_vel";
    char reset_pub_topic[] = "robotX_reset";
    reset_pub = (ros::Publisher*)calloc(TAM_ESTACOES, sizeof(ros::Publisher));
    speed_pub = (ros::Publisher*)calloc(TAM_ESTACOES, sizeof(ros::Publisher));
    for(i = 0; i < TAM_ESTACOES; i++){
        speed_pub_topic[5] = i + '0';
        reset_pub_topic[5] = i + '0';
        speed_pub[i] = n.advertise<std_msgs::UInt8MultiArray>(speed_pub_topic, 10); //create publisher to \robot_vel topic (sets robot motor's velocity)
        reset_pub[i] = n.advertise<std_msgs::Bool>(reset_pub_topic, 10);
    }
    ros::Subscriber position_sub = n.subscribe("robot_pos", 10, getPosition_callback);  //subscrive to \robot_pos topic (gets the robot current position)

    ros::spin();


    return 0;
}