#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <iostream>
#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include "std_msgs/Int64.h"

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <array>
using namespace std;

class Station //updated z rotation
{
public:

    double x,y,z,w;
    double roll, pitch, yaw;

    double yaw_sum;

    int flock_size;
    int avg_yaw;

    vector<double> orientations;
    vector<double> X;
    vector<double> Y;
    vector<double> Z;




    void get_orientations(const gazebo_msgs::ModelStates& msg);
    void get_positions(const gazebo_msgs::ModelStates& msg);
    void get_velocities(const gazebo_msgs::ModelStates& msg);
};

void Station::get_orientations(const gazebo_msgs::ModelStates& msg)
{

    Station::flock_size=msg.name.size()-1;

    for(int i(0); i<Station::flock_size; i++)
    {
    cout<<msg.name[i+1]<<endl;
        Station::x=msg.pose[i+1].orientation.x;
        Station::y=msg.pose[i+1].orientation.y;
        Station::z=msg.pose[i+1].orientation.z;
        Station::w=msg.pose[i+1].orientation.w;

        tf::Quaternion q(Station::x, Station::y, Station::z, Station::w);
        tf::Matrix3x3 m(q);

        //double r, p, y;
        m.getRPY(Station::roll, Station::pitch, Station::yaw);

        if(Station::orientations.size()==0)
        {
            Station::orientations.push_back(Station::yaw);
        }
        else
        {
            Station::orientations[i]=Station::yaw;
        }
        //ROS_INFO("i is: %d ", i);
        //ROS_INFO("yaw[i] is: %f ", Station::orientations[i]);

    }
    Station::yaw_sum=0;
    for(int i(0); i<Station::flock_size; i++)
    {
        Station::yaw_sum=Station::yaw_sum+Station::orientations[i];
    }
    Station::avg_yaw=(int)round(Station::yaw_sum/Station::flock_size*180/3.14);
    // ROS_INFO("target_yaw is: %d [in callback]", Station::avg_yaw);
}

void Station::get_positions(const gazebo_msgs::ModelStates& msg)
{
    Station::flock_size=msg.name.size()-1;

    for(int i(0); i<Station::flock_size; i++)
    {

        ////////////////////////////////////////////////////
        if(Station::X.size()==0)
        {
            Station::X.push_back(msg.pose[i+1].position.x);
        }
        else
        {
            Station::X[i]=msg.pose[i+1].position.x;
            //ROS_INFO("x: %f",Station::X[i]);
        }
        /////////////////////////////////////////////////////
        if(Station::Y.size()==0)
        {
            Station::Y.push_back(msg.pose[i+1].position.y);
        }
        else
        {
            Station::Y[i]=msg.pose[i+1].position.y;
            //ROS_INFO("y: %f",Station::Y[i]);
        }
        ////////////////////////////////////////////////////
        if(Station::Z.size()==0)
        {
            Station::Z.push_back(msg.pose[i+1].position.z);
        }
        else
        {
            Station::Z[i]=msg.pose[i+1].position.z;
            //ROS_INFO("z: %f",Station::Z[i]);
        }
        /////////////////////////////////////////////////////
    }
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground");
    ros::NodeHandle nh;

    Station station;

    std_msgs::Float64MultiArray PosArr;
    std_msgs::Int64 yawmsg;

    ros::Subscriber rot_sub     = nh.subscribe("/gazebo/model_states", 1, &Station::get_orientations, &station);
    ros::Subscriber pos_sub     = nh.subscribe("/gazebo/model_states", 1, &Station::get_positions, &station);
    ros::Publisher  yaw_pub     = nh.advertise<std_msgs::Int64>("/new_yaw", 1);

    ros::Publisher  posArr_pub  = nh.advertise<std_msgs::Float64MultiArray>("/arrpos", 1);
    int counter(0);

    ros::Rate rate(1);

    while (ros::ok())
    {
        if (counter>5)
        {

            yawmsg.data=station.avg_yaw;
            yaw_pub.publish(yawmsg);
            //ROS_INFO("target_yaw is: %d", station.avg_yaw);
            PosArr.layout.dim.push_back(std_msgs::MultiArrayDimension());
            PosArr.layout.dim[0].size = station.flock_size;
            PosArr.layout.dim[0].stride = station.flock_size*3;
            PosArr.layout.dim[0].label = "unit"; // or whatever name you typically use to index vec1

// copy in the data
            PosArr.layout.dim.push_back(std_msgs::MultiArrayDimension());
            PosArr.layout.dim[1].size = 3;
            PosArr.layout.dim[1].stride = station.flock_size;
            PosArr.layout.dim[1].label = "position"; // or whatever name you typically use to index vec1

// copy in the data
            for(int i(0); i<=station.X.size(); i++) {
                //cout<<i<<endl;
                PosArr.data.push_back(station.X[i]);
            }

            for(int i(0); i<=station.Y.size(); i++) {
                //cout<<i<<endl;
                PosArr.data.push_back(station.Y[i]);
            }

            for(int i(0); i<=station.Z.size(); i++) {
                //cout<<i<<endl;
                PosArr.data.push_back(station.Z[i]);
            }



            posArr_pub.publish(PosArr);
            PosArr.data.clear();

        }
        else
        {
            counter++;
        }

        ros::spinOnce();
        rate.sleep();
    }
}
