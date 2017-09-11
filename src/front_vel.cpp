#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <tf2_msgs/TFMessage.h>
#include "geometry_msgs/TransformStamped.h"
#include <stdlib.h>

#include <tf/transform_datatypes.h>

#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Bool.h"
#include <iostream>

#include <math.h>

#include <sensor_msgs/Imu.h>
using namespace std;

class Copter //updated z rotation
{
public:

    double x;
    double y;
    double z;
    double w;

    double roll;
    double pitch;
    double yaw;

    float vx;
    float vy;

    bool moveOn;


    void get_yaw(const sensor_msgs::Imu& msg);
    void rotation_mode(const std_msgs::Bool& msg);

};

void Copter::rotation_mode(const std_msgs::Bool& msg)
{

    Copter::moveOn = msg.data;

}

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
    //ROS_INFO("yaw is: %f", Copter::yaw);
}

/*void Copter::get_yaw(const tf2_msgs::TFMessage& msg)
{

    Copter::x=msg.transforms[0].transform.rotation.x;
    Copter::y=msg.transforms[0].transform.rotation.y;
    Copter::z=msg.transforms[0].transform.rotation.z;
    Copter::w=msg.transforms[0].transform.rotation.w;

    tf::Quaternion q(Copter::x, Copter::y, Copter::z, Copter::w);
    tf::Matrix3x3 m(q);

    //double r, p, y;
    m.getRPY(Copter::roll, Copter::pitch, Copter::yaw);

    //Copter::yaw=y;
    ROS_INFO("yaw is: %f", Copter::yaw);
}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_vel");
    ros::NodeHandle nh;

    Copter copter;

    ros::Publisher  vel_pub         = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
    //ros::Subscriber tf_sub          = nh.subscribe("/tf", 10, &Copter::get_yaw, &copter);
    ros::Subscriber tf_sub          = nh.subscribe("erlecopter/imu", 1, &Copter::get_yaw, &copter);
    ros::Subscriber rotation_sub    = nh.subscribe("update_yaw", 1, &Copter::rotation_mode, &copter);

    double v(0.4);

    ros::Rate rate(8);

    geometry_msgs::TwistStamped velMsg;

    string ns = ros::this_node::getNamespace();

    while (ros::ok())
    {
        copter.vx = sin(copter.yaw)*v;
        copter.vy = cos(copter.yaw)*v;

        velMsg.twist.linear.x=-copter.vx;
        velMsg.twist.linear.y=copter.vy;
        //velMsg.header.stamp = ros::Time::now();
        velMsg.header.frame_id="world";

        if(copter.moveOn)
        {
            cout<<ns<<" Moving on"<<endl;
            //cout<<" Moving on y "<<copter.vy<<endl;
            //cout<<" Moving on x "<<copter.vx<<endl;
            velMsg.twist.linear.x=0;
            velMsg.twist.linear.y=0;
            velMsg.twist.linear.z=0;
            vel_pub.publish(velMsg);

        }
        else
        {
            cout<<ns<<" rotating"<<endl;
            velMsg.twist.linear.x=0;
            velMsg.twist.linear.y=0;
            velMsg.twist.linear.z=0;
            vel_pub.publish(velMsg);
             ros::spinOnce();
        }
        ros::spinOnce();
       rate.sleep();
    }
}
