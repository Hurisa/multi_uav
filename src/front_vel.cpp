#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <tf2_msgs/TFMessage.h>
#include "geometry_msgs/TransformStamped.h"
#include <stdlib.h>

#include <tf/transform_datatypes.h>

#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

#include <iostream>

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

    double vx;
    double vy;

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

    double r, p, y;
    m.getRPY(r, p, y);

    Copter::yaw=y;

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_vel");
    ros::NodeHandle nh;

    Copter copter;

    ros::Publisher  vel_pub  = nh.advertise<geometry_msgs::TwistStamped>("/uav/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber tf_sub   = nh.subscribe("/tf", 10, &Copter::get_yaw, &copter);

    int v(1);

    ros::Rate rate(10);

    geometry_msgs::TwistStamped velMsg;

    while (ros::ok())
    {
    copter.vx = sin(copter.yaw)*v;
    copter.vy = cos(copter.yaw)*v;



    velMsg.twist.linear.x=-copter.vx;
    velMsg.twist.linear.y=copter.vy;
    velMsg.header.stamp = ros::Time::now();
    velMsg.header.frame_id="world";

    vel_pub.publish(velMsg);

    ros::spinOnce();
    rate.sleep();

    }





}
