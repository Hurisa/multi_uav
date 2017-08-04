#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>

#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

#include <iostream>

using namespace std;

class Loop
{
public:
    double yaw;
    void set_yaw(const geometry_msgs::PoseStamped& msg);
};

void Loop::set_yaw(const geometry_msgs::PoseStamped& msg)
{
    //std::cout<<" new variable "<< msg.pose.orientation.z<<endl;
    Loop::yaw=msg.pose.orientation.z;


}




int main(int argc, char **argv)
{
// Initialize the ROS system and become a node .

    ros::init(argc, argv, "publish_rotation");
    ros::NodeHandle nh;

    Loop loop;
// Create a publisher object .
// ros::Publisher publisher = nh.advertise<geometry_msgs::TwistStamped>("/uav/mavros/setpoint_velocity/cmd_vel", 1000);


    ros::Publisher  local_pos_pub  = nh.advertise<geometry_msgs::PoseStamped>("/uav/mavros/setpoint_position/local",10);
    ros::Subscriber local_pose_sub = nh.subscribe("/uav/mavros/local_position/pose", 10, &Loop::set_yaw, &loop);
// Loop at 2Hz until the node i s shut down.
   int counter(0);

    ros::Rate rate(10);
    while (ros::ok())
    {
        geometry_msgs::PoseStamped poseMsg;

        //nh.subscribe("/uav/mavros/local_position/pose", 10, &Loop::set_yaw, &loop);
        //move 1 (yaw rotation)
        poseMsg.pose.position.x = 0;
        poseMsg.pose.position.y = 0;
        poseMsg.pose.position.z = 2;
        poseMsg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);  //set initial orientation

        ROS_INFO("#Hello roation read is: %f", loop.yaw);


        if (counter>5)
        {
            double rotation = loop.yaw;
            ROS_INFO("[INIT] Start rotation is %f radians", loop.yaw);


            ROS_INFO("Start rotation is %f radians",rotation);

            double target_rotation=45*3.14/180;

            ROS_INFO("target rotation is %f radians",target_rotation);

            poseMsg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (0.0,0.0,rotation+target_rotation);
            ROS_INFO("QUATERNIONS: %f, %f, %f, %f ", poseMsg.pose.orientation.x, poseMsg.pose.orientation.y, poseMsg.pose.orientation.z, poseMsg.pose.orientation.w);



            poseMsg.header.stamp = ros::Time::now();
            poseMsg.header.frame_id="map";

            local_pos_pub.publish(poseMsg);


            ROS_INFO("current angle : %f", loop.yaw);


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
        else{
        ros::spinOnce();
        //ROS_INFO("Initializing subscription");
        counter++;
        }
        ros::spinOnce();
        rate.sleep();

    }


    return 0;
}



