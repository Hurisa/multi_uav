#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "std_msgs/Int64.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
#include <vector>
#include <array>


#include <math.h>       /* sqrt */

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
using namespace std;

class Boid //updated z rotation
{
public:

    double x, y, z;
    double avg_yaw;
    vector<vector<double>> posMatrix;

    double separation_angle, cohesion_angle;

    double roll, pitch, yaw;

    int lowlimit;
    int uplimit;

    int rows;
    int columns;

    void get_orientation(const std_msgs::Int64 msg);
    void build_matrix(const std_msgs::Float64MultiArray msg);
    void read_pos(const  geometry_msgs::PointStamped msg);

    void separation(vector<vector<double>> posMatrix);
    void cohesion(vector<vector<double>> posMatrix);

    void get_yaw(const sensor_msgs::Imu& msg);
};

void Boid::get_yaw(const sensor_msgs::Imu& msg)
{

    double x_quat, y_quat, z_quat, w_quat;
    x_quat=msg.orientation.x;
    y_quat=msg.orientation.y;
    z_quat=msg.orientation.z;
    w_quat=msg.orientation.w;

    tf::Quaternion q(x_quat, y_quat, z_quat, w_quat);
    tf::Matrix3x3 m(q);

    //double r, p, y;
    m.getRPY(Boid::roll, Boid::pitch, Boid::yaw);

    //Copter::yaw=y;
    //ROS_INFO("yaw is: %f", Copter::yaw);
}

void Boid::get_orientation(const std_msgs::Int64 msg)
{
    Boid::avg_yaw=msg.data*3.14/180;
    //cout<<"average angle in callback: "<<msg.data<<endl;
}

void Boid::build_matrix(const std_msgs::Float64MultiArray msg)
{


    Boid::rows=msg.data.size()/2;

    for(int i = 0; i < Boid::rows; i++)
    {

        vector<double> row;
        Boid::posMatrix.push_back(row);
        Boid::posMatrix[i].push_back(0);
    }

    int counter(0);
    for (int i(0); i<Boid::rows; i++)
    {
        for (int j(0); j<Boid::columns; j++)
        {

            Boid::posMatrix[i][j]=msg.data[counter];
            counter++;
        }
    }
}

void Boid::separation(vector<vector<double>> posMatrix)
{

    double distance;

    vector<double> x_dist;
    vector<double> y_dist;
    vector<double> z_dist;

    double loop_dist(0);
    int neighbours(0);

    for (int i(0); i<Boid::columns; i++)
    {



        //loop_dist=sqrt(pow(Boid::x-posMatrix[0][i],2)+pow(Boid::y-posMatrix[1][i],2)+pow(Boid::z-posMatrix[2][i],2));
        loop_dist=sqrt(pow(Boid::x-posMatrix[0][i],2)+pow(Boid::y-posMatrix[1][i],2));

        //cout<<posMatrix[0][i]<<" "<<posMatrix[1][i]<<" "<<posMatrix[2][i]<<endl;

        //cout<<"distance: [outloop]"<<loop_dist<<endl;
        double err=1;
        //cout<<"err: "<<err<<endl;
        if (loop_dist<=Boid::lowlimit && loop_dist>err)
        {
            cout<<"distance: [inloop separation]"<<loop_dist<<endl;
            x_dist.push_back(posMatrix[0][i]);
            y_dist.push_back(posMatrix[1][i]);
            z_dist.push_back(posMatrix[2][i]);

            //cout<<"x-mat: "<<posMatrix[0][i]<<endl;
            //cout<<"y-mat: "<<posMatrix[1][i]<<endl;
            //cout<<"x-mat: "<<posMatrix[2][i]<<endl;

            neighbours++;
            //distance.push_back(loop_dist);
            //cout<<"distance: "<<loop_dist<<endl;

            //cout<<" neighbours:" << neighbours<<endl;
        }


    }


    double x_avg(0);
    double y_avg(0);
    double z_avg(0);

    for(int i(0); i<neighbours; i++)
    {
        x_avg=x_avg+x_dist[i];
        y_avg=y_avg+y_dist[i];
        //z_avg=z_avg+z_dist[i];
    }
    x_avg=x_avg/neighbours;
    y_avg=y_avg/neighbours;
    //z_avg=z_avg/neighbours;



    /*cout<<"x "<<Boid::x<<endl;
    cout<<"y "<<Boid::y<<endl;
    cout<<"z "<<Boid::z<<endl;

    cout<<"distance:x "<<pow(Boid::x-x_avg,2)<<endl;
    cout<<"distance:y "<<pow(Boid::y-y_avg,2)<<endl;
    cout<<"distance:z "<<pow(Boid::z-z_avg,2)<<endl;

    cout<<"distance:1 "<<pow(Boid::x-x_avg,2)+pow(Boid::y-y_avg,2)+pow(Boid::z-z_avg,2)<<endl;
    distance=sqrt(pow(Boid::x-x_avg,2)+pow(Boid::y-y_avg,2)+pow(Boid::z-z_avg,2));
    cout<<"distance:2 "<<distance<<endl;

    //cout<<"separation x: "<<Boid::x<<endl;
    //cout<<"separation y: "<<Boid::y<<endl;
    //cout<<"average x: "<<x_avg<<endl;
    //cout<<"average y: "<<y_avg<<endl;*/
    Boid::separation_angle=atan2(Boid::y-y_avg,Boid::x-x_avg);


    //cout<<"separation angle [raw] "<<Boid::separation_angle<<endl;
    //Boid::separation_angle=Boid::separation_angle-3.14;
    //if (Boid::separation_angle<-3.14)
    //{
    //    Boid::separation_angle=Boid::separation_angle+(2*3.14);
    //    cout<<"separation angle [-180 180] "<<Boid::separation_angle<<endl;
    //}
    string ns = ros::this_node::getNamespace();
    cout<<ns<<" separation angle: "<<Boid::separation_angle*180/3.14<<endl;
}


void Boid::cohesion(vector<vector<double>> posMatrix)
{
    double distance;

    vector<double> x_dist;
    vector<double> y_dist;
    vector<double> z_dist;

    double loop_dist(0);
    int neighbours(0);

    for (int i(0); i<Boid::columns; i++)
    {



        //loop_dist=sqrt(pow(Boid::x-posMatrix[0][i],2)+pow(Boid::y-posMatrix[1][i],2)+pow(Boid::z-posMatrix[2][i],2));
        loop_dist=sqrt(pow(Boid::x-posMatrix[0][i],2)+pow(Boid::y-posMatrix[1][i],2));

        //cout<<posMatrix[0][i]<<" "<<posMatrix[1][i]<<" "<<posMatrix[2][i]<<endl;

       // cout<<"distance: [outloop cohesion]"<<loop_dist<<endl;
        //double err=pow(10,-2);
        //cout<<"err: "<<err<<endl;
        if (loop_dist>Boid::lowlimit && loop_dist<Boid::uplimit)
        {
            cout<<"distance: [inloop cohesion]"<<loop_dist<<endl;
            x_dist.push_back(posMatrix[0][i]);
            y_dist.push_back(posMatrix[1][i]);
            z_dist.push_back(posMatrix[2][i]);

            //cout<<"x-mat: "<<posMatrix[0][i]<<endl;
            //cout<<"y-mat: "<<posMatrix[1][i]<<endl;
            //cout<<"x-mat: "<<posMatrix[2][i]<<endl;

            neighbours++;
            //distance.push_back(loop_dist);
            //cout<<"distance: "<<loop_dist<<endl;

            //cout<<" neighbours:" << neighbours<<endl;
        }


    }


    double x_avg(0);
    double y_avg(0);
    double z_avg(0);

    for(int i(0); i<neighbours; i++)
    {
        x_avg=x_avg+x_dist[i];
        y_avg=y_avg+y_dist[i];
        //z_avg=z_avg+z_dist[i];
    }
    x_avg=x_avg/neighbours;
    y_avg=y_avg/neighbours;
    //z_avg=z_avg/neighbours;

    //distance=sqrt(pow(Boid::x-x_avg,2)+pow(Boid::y-y_avg,2)+pow(Boid::z-z_avg,2));
    //cout<<"distance: "<<distance<<endl;

    //cout<<"separation x: "<<Boid::x<<endl;
    //cout<<"separation y: "<<Boid::y<<endl;
    //cout<<"average x: "<<x_avg<<endl;
    //cout<<"average y: "<<y_avg<<endl;
    Boid::cohesion_angle=atan2(Boid::y-y_avg,Boid::x-x_avg);
    //cout<<"cohesion angle [raw] "<<Boid::cohesion_angle<<endl;
    Boid::cohesion_angle=Boid::cohesion_angle-3.14;
    if (Boid::cohesion_angle<-3.14)
    {
        Boid::cohesion_angle=Boid::cohesion_angle+(2*3.14);
        //cout<<"cohesion angle [-180 180] "<<Boid::cohesion_angle<<endl;
    }
    string ns = ros::this_node::getNamespace();
    cout<<ns<<" cohesion angle: "<<Boid::cohesion_angle*180/3.14<<endl;
}

void Boid::read_pos(const geometry_msgs::PointStamped msg)
{
    Boid::x=msg.point.x;
    Boid::y=msg.point.y;
    Boid::z=msg.point.z;

}







int main(int argc, char **argv)
{

    ros::init(argc, argv, "flock");
    ros::NodeHandle nh;

    Boid boid;

    ros::Subscriber yaw_sub        = nh.subscribe("/new_yaw", 1, &Boid::get_orientation, &boid);
    ros::Subscriber pos_sub        = nh.subscribe("/arrpos", 1, &Boid::build_matrix, &boid);
    ros::Subscriber self_pos_sub   = nh.subscribe("erlecopter/ground_truth/position", 1, &Boid::read_pos, &boid);
    ros::Subscriber self_yaw_sub   = nh.subscribe("erlecopter/imu", 1, &Boid::get_yaw, &boid);

    ros::Publisher  sep_pub        = nh.advertise<std_msgs::Int64>("sepangle",1);

    boid.uplimit =10;
    boid.lowlimit=5;
    boid.columns=2;

    ros::Rate rate(7);

    std_msgs::Int64 msg;

    int max_rot=30;

    double weight_cohesion;
    double weight_separation;

    double final_angle;
    string ns = ros::this_node::getNamespace();
    int counter(0);
    while (ros::ok())
    {

        if(counter>5)
        {
            ///////////////////////////////////////////
            boid.separation(boid.posMatrix);
            boid.cohesion(boid.posMatrix);

            if (isnan(boid.separation_angle) && isnan(boid.cohesion_angle))
            {
            //cout<<ns<<" rule1"<<endl;
                final_angle = boid.yaw;
            }
            else if (isnan(boid.separation_angle) && !isnan(boid.cohesion_angle))
            {
            //cout<<ns<<" rule2"<<endl;
                //final_angle=(boid.cohesion_angle + boid.avg_yaw)/2;
                final_angle=(boid.cohesion_angle);
            }
            else if (!isnan(boid.separation_angle) && isnan(boid.cohesion_angle))
            {
            //cout<<ns<<" rule3"<<endl;
                //final_angle=(boid.separation_angle+ boid.avg_yaw)/2;
                final_angle=(boid.separation_angle);

            }


            //cout<<ns<<" delta theta "<<(final_angle-boid.yaw)*180/3.14<<endl;

           if (final_angle-boid.yaw>(max_rot*3.14/180))
            {
                //cout<<ns<<" option1"<<endl;
                //cout<<ns<<" final angle is: "<<final_angle*180/3.14<<endl;
                final_angle=boid.yaw+max_rot*3.14/180;
                msg.data=(int)round(final_angle*180/3.14);
                cout<<ns<<" final angle is: "<<msg.data<<endl;
                //cout<<ns<<" current angle: "<<boid.yaw*180/3.14<<endl;
            }
            else if (final_angle-boid.yaw<-max_rot*3.14/180)
            {
                //cout<<ns<<" option2"<<endl;
                //cout<<ns<<" final angle is: "<<final_angle*180/3.14<<endl;
                final_angle=boid.yaw-max_rot*3.14/180;
                msg.data=(int)round(final_angle*180/3.14);
                cout<<ns<<" final angle is: "<<msg.data<<endl;
                //cout<<ns<<" current angle: "<<boid.yaw*180/3.14<<endl;
            }
            else
            {
                //cout<<ns<<" option3"<<endl;
                //cout<<ns<<" final angle is: "<<msg.data<<endl;
                msg.data=(int)round(final_angle*180/3.14);
                cout<<ns<<" final angle is: "<<msg.data<<endl;
                //cout<<ns<<" current angle: "<<boid.yaw*180/3.14<<endl;
            }
            //msg.data=(int)round(final_angle*180/3.14);
            sep_pub.publish(msg);

        }
        else
        {
            counter++;
        }
        ros::spinOnce();
        rate.sleep();
    }
}
