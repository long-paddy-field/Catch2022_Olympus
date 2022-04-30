#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <string>
#include <math.h>

const double pi = 3.1415;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SCARA_sim");
    ros::NodeHandle nh;
    ros::Publisher rviz_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

    ros::Rate loop_rate(10);

    sensor_msgs::JointState rviz_msg;

    int counter = 0;

    rviz_msg.name.resize(4);
    rviz_msg.position.resize(4);

    rviz_msg.name[0] = "stand_arm1";
    rviz_msg.name[1] = "arm1_arm2";
    rviz_msg.name[2] = "arm2_linear";
    rviz_msg.name[3] = "linear_wrist";

    while (ros::ok())
    {
        rviz_msg.header.stamp = ros::Time::now();

        rviz_msg.position[0] = (float)counter / 50;
        rviz_msg.position[1] = (float)counter / 50;
        rviz_msg.position[2] = (float)counter / -1000;
        rviz_msg.position[3] = (float)counter / 50;

        counter++;
        rviz_pub.publish(rviz_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}