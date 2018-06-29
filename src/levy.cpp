#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <random>


#define PI 3.14159265358979323846
class LevyBoid
{
public:
  double x, y, z;
  void getPos(const gazebo_msgs::ModelStates& msg);
};

void LevyBoid::getPos(const gazebo_msgs::ModelStates& msg)
{

            LevyBoid::x=msg.pose[0].position.x;
            LevyBoid::y=msg.pose[0].position.y;


}


int main(int argc, char **argv)
{
// Initialize the ROS system and become a node .

  ros::init(argc, argv, "publish_velocity");
  LevyBoid boid;


  ros::NodeHandle nh;

  ros::Subscriber sep_sub = nh.subscribe("/gazebo/model_states", 1, &LevyBoid::getPos, &boid);

// Create a publisher object .
  ros::Publisher publisher = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1000);

// Loop at 2Hz until the node i s shut down.
     struct timeval time;
     gettimeofday(&time,NULL);

     // microsecond has 1 000 000
     // Assuming you did not need quite that accuracy
     // Also do not assume the system clock has that accuracy.
     srand((time.tv_sec * 1000) + (time.tv_usec / 1000));

     double mu=1.5;
     double sigma=10.0;
     double muOne=mu-1;
     double muTwo=2-mu;
     double x, y, theta, jump;
     double U1, U2, U3, phi, r;
     double vmax=0.5;

     ros::Rate rate(1);
  while (ros::ok())
  {
        
        //static double X1, X2;

        U1=double(rand())/double(RAND_MAX)*2-1;
        U2=double(rand())/double(RAND_MAX)*2-1;
        U3=double(rand())/double(RAND_MAX)*2-1;

        U1 = U1*(PI/2);
        U2 = (U2+1)/2;

        phi = U3 * M_PI;


        r = (sin(muOne * U1) / pow(cos(U1), 1/muOne) ) * pow((cos(muTwo * U1) / U2), (muTwo / muOne));

        x = r * cos(phi);
        y = r * sin(phi);

        theta=atan2(y,x);
        jump=sqrt(pow(x,2)+pow(y,2));

        ROS_INFO_STREAM("JUMP SIZE: "<<jump); 

        geometry_msgs::TwistStamped msg;

        msg.twist.linear.x=vmax*cos(theta);

        //msg.angular.z=5*(2*double(rand())/double(RAND_MAX)-1);

        msg.twist.linear.y=vmax*sin(theta);

        msg.header.stamp = ros::Time::now();

        publisher.publish(msg);

        /*ROS_INFO_STREAM("Sending random velocity command:"<<" linear_x="<<msg.twist.linear.x<<" linear_y=" <<msg.twist.linear.y);*/
        ros::spinOnce();
        rate.sleep();
  }
  return 0;
}

