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
#include "std_msgs/Bool.h"
#include "std_msgs/Int64.h"

#include <sensor_msgs/Imu.h>
using namespace std;

class Copter //updated z rotation
{
public:

    double x, y, z, w;
    double roll, pitch, yaw;
    double vx, vy;

    bool moveOn;

    int new_angle;

    void get_yaw(const sensor_msgs::Imu& msg);
    void rotation_mode(const std_msgs::Bool& msg);
    void read_new_yaw(const std_msgs::Int64& msg);
};

/*void Copter::get_yaw(const tf2_msgs::TFMessage& msg)
{
    Copter::x=msg.transforms[0].transform.rotation.x;
    Copter::y=msg.transforms[0].transform.rotation.y;
    Copter::z=msg.transforms[0].transform.rotation.z;
    Copter::w=msg.transforms[0].transform.rotation.w;

    tf::Quaternion q(Copter::x, Copter::y, Copter::z, Copter::w);
    tf::Matrix3x3 m(q);

    m.getRPY(Copter::roll, Copter::pitch, Copter::yaw);
}*/

void Copter::get_yaw(const sensor_msgs::Imu& msg)
{

    Copter::x=msg.orientation.x;
    Copter::y=msg.orientation.y;
    Copter::z=msg.orientation.z;
    Copter::w=msg.orientation.w;

    tf::Quaternion q(Copter::x, Copter::y, Copter::z, Copter::w);
    tf::Matrix3x3 m(q);

    //double r, p, y;
    m.getRPY(Copter::roll, Copter::pitch, Copter::yaw);

    //Copter::yaw=y;
    //ROS_INFO("yaw is: %f", Copter::yaw*180/3.14);
}

void Copter::read_new_yaw(const std_msgs::Int64& msg)
{

    Copter::new_angle = msg.data;

}

void Copter::rotation_mode(const std_msgs::Bool& msg)
{

    Copter::moveOn = msg.data;

}

int main(int argc, char **argv)
{
// Initialize the ROS system and become a node .

    ros::init(argc, argv, "publish_rotation");
    ros::NodeHandle nh;

    Copter copter;
// Create a publisher object .
// ros::Publisher publisher = nh.advertise<geometry_msgs::TwistStamped>("/uav/mavros/setpoint_velocity/cmd_vel", 1000);



    ros::Subscriber yaw_sub         = nh.subscribe("erlecopter/imu", 1, &Copter::get_yaw, &copter);
    //ros::Subscriber rotation_sub    = nh.subscribe("/update_yaw", 10, &Copter::rotation_mode, &copter);

    //ros::Subscriber new_yaw_sub     = nh.subscribe("/new_yaw", 10, &Copter::read_new_yaw, &copter);
    ros::Subscriber new_yaw_sub     = nh.subscribe("sepangle", 1, &Copter::read_new_yaw, &copter);

    ros::Publisher  yaw_pub      = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",1);
    ros::Publisher  rotation_pub = nh.advertise<std_msgs::Bool>("update_yaw", 1);
    //ros::Publisher  vel_pub      = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);

// Loop at 2Hz until the node i s shut down.
    int counter(0);

    double error;

    std_msgs::Bool _moveOn;
    mavros_msgs::AttitudeTarget poseMsg;

    copter.new_angle=(int)round(copter.yaw);

    ros::Rate rate(5);
    string ns = ros::this_node::getNamespace();

    geometry_msgs::TwistStamped velMsg;
    while (ros::ok())
    {
        //geometry_msgs::PoseStamped poseMsg;

        //std_msgs::Bool moveOn;
        //nh.subscribe("/uav/mavros/local_position/pose", 10, &Loop::set_yaw, &loop);
        //move 1 (yaw rotation)

        //poseMsg.pose.position.x = 0;
        //poseMsg.pose.position.y = 0;
        //poseMsg.pose.position.z = 2;

        //poseMsg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);  //set initial orientation
        poseMsg.orientation = tf::createQuaternionMsgFromRollPitchYaw(copter.roll,copter.pitch,copter.yaw);  //set initial orientation

        //poseMsg.body_rate.z=-2;
        //ROS_INFO("#Hello roation read is: %f", copter.yaw);


        if (counter>5)
        {
            double rotation = copter.yaw;
            //ROS_INFO("[INIT] Start rotation is %f radians", copter.yaw);


            //ROS_INFO("Start rotation is %f radians",rotation);
            //double angle(45);
            double target_rotation=copter.new_angle*3.14/180;
            //ROS_INFO("new angle is: %d", copter.new_angle);

            //ROS_INFO("target rotation is %f radians",target_rotation);

            poseMsg.orientation = tf::createQuaternionMsgFromRollPitchYaw (0.0,0.0,target_rotation);
            //ROS_INFO("QUATERNIONS: %f, %f, %f, %f ", poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z, poseMsg.orientation.w);



            //poseMsg.header.stamp = ros::Time::now();
            poseMsg.header.frame_id="world";

            error=abs(target_rotation-copter.yaw);

            if(target_rotation-copter.yaw>=0)
            {
                poseMsg.body_rate.z=-0.5;
            }
            else
            {
                poseMsg.body_rate.z=0.5;
            }

            //ROS_INFO("target_rotation: %f", target_rotation);
            //ROS_INFO("copter_yaw: %f", copter.yaw);
            cout<<ns<<"target rotation: "<<target_rotation<<endl;
            //cout<<"current orientation [control]: "<<copter.yaw<<endl;
            cout<<ns<<" error: "<<error<<endl;



            if (error>0.1)
            {
                //ROS_INFO("rotating");
                yaw_pub.publish(poseMsg);
                _moveOn.data=false;
                rotation_pub.publish(_moveOn);

                //velMsg.twist.linear.x=0;
                //velMsg.twist.linear.y=0;
                //velMsg.twist.linear.z=0;
                //vel_pub.publish(velMsg);
            }
            else
            {
                //ROS_INFO("sleeping...");
                //ROS_INFO("error is small: %f", error);

                _moveOn.data=true;
                rotation_pub.publish(_moveOn);
                sleep(10);
                copter.new_angle=(int)round(copter.yaw);
            }


            //ROS_INFO("error: %f", error);




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
            //ros::spinOnce();
            //ROS_INFO("Initializing subscription");
            counter++;
        }
        ros::spinOnce();
        rate.sleep();

    }


    return 0;
}



