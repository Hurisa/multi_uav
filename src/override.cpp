#include "ros/ros.h"
#include "std_msgs/String.h"
#include <mavros_msgs/OverrideRCIn.h>
#include <stdlib.h>


int main(int argc, char **argv)
{
// Initialize the ROS system and become a node .

  ros::init(argc, argv, "overrideRC");

  ros::NodeHandle nh;

// Create a publisher object .
  ros::Publisher publisher = nh.advertise<mavros_msgs::OverrideRCIn>("/uav/mavros/rc/override", 1000);

  ros::Rate rate(10);
  while (ros::ok())
  {


        mavros_msgs::OverrideRCIn msg;

        //msg.channels[0] = 1500;
        //msg.channels[1] = 1500;
        //msg.channels[2] = 1500;
        msg.channels[3] = 1400;
        //msg.channels[4] = 1100;
        //msg.channels[5] = 1100;
        //msg.channels[6] = 1100;
        //msg.channels[7] = 1100;

        msg.channels[0]=65535;
        msg.channels[1]=65535;
        msg.channels[2]=65535;
        //msg.channels[3]=1500;
        msg.channels[4]=65535;
        msg.channels[5]=65535;
        msg.channels[6]=65535;
        msg.channels[7]=65535;



        publisher.publish(msg);

        ROS_INFO_STREAM("Overriding RC");

        ros::spinOnce();
        rate.sleep();
  }


  return 0;
}
