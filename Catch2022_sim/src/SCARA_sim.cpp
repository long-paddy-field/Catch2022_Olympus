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
    ros::Publisher arm1_pub = nh.advertise<std_msgs::Float64>("my_SCARA/arm1_position_controller/command", 10);
    ros::Publisher arm2_pub = nh.advertise<std_msgs::Float64>("my_SCARA/arm2_position_controller/command", 10);
    ros::Publisher linear_pub = nh.advertise<std_msgs::Float64>("my_SCARA/linear_position_controller/command", 10);
    ros::Publisher wrist_pub = nh.advertise<std_msgs::Float64>("my_SCARA/wrist_position_controller/command", 10);

    ros::Rate loop_rate(10);
    int count = 0;

    sensor_msgs::JointState rviz_msg;

    std_msgs::Float64 arm_msg[2];

    float arm1_data = 0;
    float arm2_data = 0;
   
    while (ros::ok())
    {

        arm1_data = (float)count / 100;
        arm2_data = (float)count / -100;

        rviz_msg.header.stamp = ros::Time::now();
        rviz_msg.name.resize(4);
        rviz_msg.position.resize(2);
        rviz_msg.name[0] = "stand_arm1";
        rviz_msg.name[1] = "arm1_arm2";
        rviz_msg.position[0] = arm1_data;
        rviz_msg.position[1] = arm2_data;

        arm_msg[0].data = arm1_data;
        arm_msg[1].data = arm2_data;

        arm1_pub.publish(arm_msg[0]);
        arm2_pub.publish(arm_msg[1]);
        rviz_pub.publish(rviz_msg);
        count++;
        ROS_INFO("data: %f", arm_msg[0].data);
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