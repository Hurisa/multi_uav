#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <cmath>        // std::abs
#include <iostream>

using namespace std;

class Copter //updated z rotation
{
public:

    double x, y, z, w;
    double roll, pitch, yaw;
    double vx, vy;

    void get_yaw(const tf2_msgs::TFMessage& msg);
};

void Copter::get_yaw(const tf2_msgs::TFMessage& msg)
{

    Copter::x=msg.transforms[0].transform.rotation.x;
    Copter::y=msg.transforms[0].transform.rotation.y;
    Copter::z=msg.transforms[0].transform.rotation.z;
    Copter::w=msg.transforms[0].transform.rotation.w;

    tf::Quaternion q(Copter::x, Copter::y, Copter::z, Copter::w);
    tf::Matrix3x3 m(q);


    m.getRPY(Copter::roll, Copter::pitch, Copter::yaw);
}



int main(int argc, char **argv)
{
// Initialize the ROS system and become a node .

    ros::init(argc, argv, "publish_rotation");
    ros::NodeHandle nh;

    Copter copter;
// Create a publisher object .
// ros::Publisher publisher = nh.advertise<geometry_msgs::TwistStamped>("/uav/mavros/setpoint_velocity/cmd_vel", 1000);


    ros::Publisher  yaw_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/uav/mavros/setpoint_raw/attitude",10);
    ros::Subscriber yaw_sub = nh.subscribe("/tf", 10, &Copter::get_yaw, &copter);

// Loop at 2Hz until the node i s shut down.
    int counter(0);

    double error;

    ros::Rate rate(10);
    while (ros::ok())
    {
        //geometry_msgs::PoseStamped poseMsg;
        mavros_msgs::AttitudeTarget poseMsg;
        //nh.subscribe("/uav/mavros/local_position/pose", 10, &Loop::set_yaw, &loop);
        //move 1 (yaw rotation)

        //poseMsg.pose.position.x = 0;
        //poseMsg.pose.position.y = 0;
        //poseMsg.pose.position.z = 2;

        //poseMsg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);  //set initial orientation
        poseMsg.orientation = tf::createQuaternionMsgFromRollPitchYaw(copter.roll,copter.pitch,copter.yaw);  //set initial orientation

        poseMsg.body_rate.z=1;
        //ROS_INFO("#Hello roation read is: %f", copter.yaw);


        if (counter>5)
        {
            double rotation = copter.yaw;
            //ROS_INFO("[INIT] Start rotation is %f radians", copter.yaw);


            //ROS_INFO("Start rotation is %f radians",rotation);
            double angle(45);
            double target_rotation=angle*3.14/180;

            //ROS_INFO("target rotation is %f radians",target_rotation);

            poseMsg.orientation = tf::createQuaternionMsgFromRollPitchYaw (0.0,0.0,target_rotation);
            //ROS_INFO("QUATERNIONS: %f, %f, %f, %f ", poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z, poseMsg.orientation.w);



            poseMsg.header.stamp = ros::Time::now();
            poseMsg.header.frame_id="world";

            error=abs(target_rotation-copter.yaw);
            if (error>0.1)
            {
                ROS_INFO("error: %f", error);
                yaw_pub.publish(poseMsg);
            }
            else
            {
                ROS_INFO("sleeping...");
                ROS_INFO("error: %f", error);
                sleep(30);

            }


            ROS_INFO("error: %f", error);




            //ROS_INFO("current angle : %f", copter.yaw);


            //sleep(120);

            /*for(int i = 157; ros::ok() && i > 0; --i)   //Turn 180 degrees = 3.14 radians
            {

                rotation = rotation + 0.01;
                poseMsg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (0.0,0.0,rotation);
                ROS_INFO("QUATERNIONS: %f, %f, %f, %f ", poseMsg.pose.orientation.x, poseMsg.pose.orientation.y, poseMsg.pose.orientation.z, poseMsg.pose.orientation.w);

                poseMsg.header.stamp = ros::Time::now();
                poseMsg.header.frame_id="map";

                local_pos_pub.publish(poseMsg);

                ROS_INFO("rotating: %f",rotation);
                //ROS_INFO("Sleeping");
                //sleep(2);

                ros::spinOnce();
                //ROS_INFO("bye bye roation read is: %f", loop.yaw);
            }
            ROS_INFO("End rotation is %f radians",rotation);
            ROS_INFO("QUATERNIONS: %f, %f, %f, %f ", poseMsg.pose.orientation.x, poseMsg.pose.orientation.y, poseMsg.pose.orientation.z, poseMsg.pose.orientation.w);*/

        }
        else
        {
            ros::spinOnce();
            //ROS_INFO("Initializing subscription");
            counter++;
        }
        ros::spinOnce();
        rate.sleep();

    }


    return 0;
}



