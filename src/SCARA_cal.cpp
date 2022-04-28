#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <string>
#include <math.h>

const double pi = 3.1415;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SCARA_cal");
    ros::NodeHandle nh;
    ros::Publisher arm1_pub = nh.advertise<std_msgs::Float64>("my_SCARA/arm1_position_controller/command", 10);
    ros::Publisher arm2_pub = nh.advertise<std_msgs::Float64>("my_SCARA/arm2_position_controller/command", 10);

    ros::Rate loop_rate(10);
    int count = 0;

    std_msgs::Float64 arm_msg;
    arm_msg.data = 0;

    while (ros::ok())
    {
        arm_msg.data = (float)count/200;
        arm1_pub.publish(arm_msg);
        arm2_pub.publish(arm_msg);
        count++;
        ROS_INFO("data: %f", arm_msg.data);
        ros::spinOnce();
        loop_rate.sleep();
    }

    //     while (ros::ok())
    //     {
    //         sensor_msgs::JointState js0;
    //         js0.header.stamp = ros::Time::now();
    //         js0.name.resize(2);
    //         js0.name[0] = "stand_arm1";
    //         js0.name[1] = "arm1_arm2";
    //         js0.position.resize(2);
    //         js0.position[0] = pi/2;
    //         js0.position[1] = -1*pi/2;

    //         ROS_INFO("pub_0: %f" ,js0.position[0]);
    //         ROS_INFO("pub_1: %f" ,js0.position[1]);

    // //        joint_pub.publish(js0);
    //         count++;

    //         ros::spinOnce();
    //         loop_rate.sleep();
    //     }
    return 0;
}