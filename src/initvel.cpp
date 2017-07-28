#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <stdlib.h>


int main(int argc, char **argv)
{
// Initialize the ROS system and become a node .

  ros::init(argc, argv, "publish_velocity");

  ros::NodeHandle nh;

// Create a publisher object .
  ros::Publisher publisher = nh.advertise<geometry_msgs::TwistStamped>("/uav/mavros/setpoint_velocity/cmd_vel", 1000);

// Loop at 2Hz until the node i s shut down.
     struct timeval time;
     gettimeofday(&time,NULL);

     // microsecond has 1 000 000
     // Assuming you did not need quite that accuracy
     // Also do not assume the system clock has that accuracy.
     srand((time.tv_sec * 1000) + (time.tv_usec / 1000));

  ros::Rate rate(10);
  while (ros::ok())
  {


        geometry_msgs::TwistStamped msg;

        msg.twist.linear.x=double(rand())/double(RAND_MAX);
        //msg.angular.z=5*(2*double(rand())/double(RAND_MAX)-1);

        msg.twist.linear.y=double(rand())/double(RAND_MAX);
        publisher.publish(msg);

        ROS_INFO_STREAM("Sending random velocity command:"<<" linear_x="<<msg.twist.linear.x<<" linear_y=" <<msg.twist.linear.y);

        rate.sleep();
  }


  return 0;
}



