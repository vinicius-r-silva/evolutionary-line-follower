#include "ros/ros.h"
#include "std_msgs/Bool.h"


using namespace std;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Publisher pub;
    pub = n.advertise<std_msgs::Bool>("robot_reset", 1);

    std_msgs::Bool msg;
    msg.data = true;
    ros::Rate poll_rate(1);

    while (ros::ok())
    {
        pub.publish(msg);   
        ros::spinOnce();
        poll_rate.sleep();
    }
    return 0;
}

