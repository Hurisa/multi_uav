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

#include <geometry_msgs/TwistStamped.h>

#include <cmath>        // std::abs
using namespace std;



class Boid
{
public:

    string name;
    string ns;


    double x, y, z;
    double vx, vy;

    double vx_sep   , vy_sep;
    double vx_coh   , vy_coh;
    double vx_align , vy_align;

    int id;
    int flock_size;
    int lowlimit;
    int uplimit;

    bool nocoh;
    bool nosep;
    bool noali;

    void get_name(const gazebo_msgs::ModelStates& msg);
    void separation(const gazebo_msgs::ModelStates& msg);
    void cohesion(const gazebo_msgs::ModelStates& msg);
    void alignment(const gazebo_msgs::ModelStates& msg);
    void get_vel(const gazebo_msgs::ModelStates& msg);
};






void Boid::get_name(const gazebo_msgs::ModelStates& msg)
{

    Boid::flock_size=msg.name.size()-1;

    for(int i(0); i<=Boid::flock_size; i++)
    {
        if(!Boid::ns.compare(msg.name[i]))
        {
            Boid::id=i;
            Boid::x=msg.pose[i].position.x;
            Boid::y=msg.pose[i].position.y;
        }
    }
}

void Boid::alignment(const gazebo_msgs::ModelStates& msg)
{
    double distance;
    double vx_align_loc, vy_align_loc;


    double v(0.4);

    int counter(0);
    for(int i(1); i<=Boid::flock_size; i++)
    {

        if (i!=Boid::id)
        {
            distance=sqrt(pow(Boid::x-msg.pose[i].position.x,2)+pow(Boid::y-msg.pose[i].position.y,2));
            //cout<<"distance s"<<distance<<endl;

            if(distance<Boid::uplimit)
            {
                vx_align_loc=vx_align_loc+msg.twist[i].linear.x;
                vy_align_loc=vy_align_loc+msg.twist[i].linear.y;
                counter++;
                //cout<<"read position y s"<<y_sep<<endl;
            }

        }
    }

        if(counter>0)
        {
            //cout<<"position x s"<<Boid::x<<endl;
            //cout<<"position y s"<<Boid::y<<endl;

            //cout<<"average x s"<<x_sep/counter<<endl;
            //cout<<"average y s"<<y_sep/counter<<endl;

            Boid::vx_align=-vy_align_loc/counter*1.5;
            Boid::vy_align=vx_align_loc/counter*1.5;
            cout<<Boid::ns<<endl;
            cout<<"vx alignment: "<< Boid::vx_align<<endl;
            cout<<"vy alignment: "<< Boid::vy_align<<endl;

            //separation_angle=atan2(y_sep,x_sep);

            //cout<<"separation_angle "<<separation_angle*180/3.14<<endl;
            //Boid::vx_sep = sin(separation_angle)*v;
            //Boid::vy_sep = cos(separation_angle)*v;
        }
        else
        {
            Boid::noali=true;
        }

}

void Boid::separation(const gazebo_msgs::ModelStates& msg)
{

    double distance;
    double x_sep, y_sep, z_sep;
    double separation_angle;

    double v(0.8);

    int counter(0);
    for(int i(1); i<=Boid::flock_size; i++)
    {

        if (i!=Boid::id)
        {
            distance=sqrt(pow(Boid::x-msg.pose[i].position.x,2)+pow(Boid::y-msg.pose[i].position.y,2));
            //cout<<"distance s"<<distance<<endl;

            if(distance<Boid::lowlimit)
            {
                x_sep=x_sep+msg.pose[i].position.x;
                y_sep=y_sep+msg.pose[i].position.y;
                counter++;
                cout<<"i "<<i<<endl;
                cout<<"read position x s "<<x_sep<<endl;
                cout<<"read position y s "<<y_sep<<endl;
            }

        }
    }

        if(counter>0)
        {
            //cout<<"position x s"<<Boid::x<<endl;
            //cout<<"position y s"<<Boid::y<<endl;
            cout<<Boid::ns<<endl;
            cout<<"average x s"<<x_sep/counter<<endl;
            cout<<"average y s"<<y_sep/counter<<endl;
            Boid::nosep=false;
            x_sep=Boid::x-x_sep/counter;
            y_sep=Boid::y-y_sep/counter;

            //cout<<"distance x s"<<x_sep<<endl;
            //cout<<"distance y s"<<y_sep<<endl;

            separation_angle=atan2(y_sep,x_sep);

            cout<<"separation_angle "<<separation_angle*180/3.14<<endl;
            Boid::vx_sep = -sin(separation_angle)*v;
            Boid::vy_sep = cos(separation_angle)*v;

            cout<<" vx sep "<<Boid::vx_sep<<endl;
            cout<<" vy sep "<<Boid::vy_sep<<endl;
        }
        else
        {
            Boid::nosep=true;
        }




}

void Boid::cohesion(const gazebo_msgs::ModelStates& msg)
{

    double distance;
    double x_coh, y_coh, z_coh;
    double cohesion_angle;

    double v(0.4);

    int counter(0);
    for(int i(1); i<=Boid::flock_size; i++)
    {
        if (i!=Boid::id)
        {
            distance=sqrt(pow(Boid::x-msg.pose[i].position.x,2)+pow(Boid::y-msg.pose[i].position.y,2));
            //cout<<"distance c"<<distance<<endl;

            if(distance>Boid::lowlimit && distance<Boid::uplimit  )
            {
                x_coh=x_coh+msg.pose[i].position.x;
                y_coh=y_coh+msg.pose[i].position.y;
                counter++;
            }
        }
     }


        if(counter>0)
        {
        cout<<"counter"<<counter<<endl;
            Boid::nocoh=false;
            x_coh=Boid::x-x_coh/counter;
            y_coh=Boid::y-y_coh/counter;

            //cout<<"distance x c"<<x_coh<<endl;
            //cout<<"distance y c"<<y_coh<<endl;

            cohesion_angle=atan2(y_coh,x_coh);


            cohesion_angle=cohesion_angle-3.14;
            if (cohesion_angle<-3.14)
            {
                cohesion_angle=cohesion_angle+(2*3.14);
        //cout<<"cohesion angle [-180 180] "<<Boid::cohesion_angle<<endl;
            }
            cout<<Boid::ns<<endl;
            cout<<"cohesion_angle "<<cohesion_angle*180/3.14<<endl;
            Boid::vx_coh = -sin(cohesion_angle)*v;
            Boid::vy_coh = cos(cohesion_angle)*v;
            cout<<" vx coh "<<Boid::vx_coh<<endl;
            cout<<" vy coh "<<Boid::vy_coh<<endl;

        }
        else
        {
            Boid::nocoh=true;
        }



}

void Boid::get_vel(const gazebo_msgs::ModelStates& msg)
{
    Boid::flock_size=msg.name.size()-1;
    for(int i(1); i<=Boid::flock_size; i++)
    {

        if (i==Boid::id)
        {

            Boid::vx=-msg.twist[i].linear.y;
            Boid::vy=msg.twist[i].linear.x;
        }
    }

}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "flock");
    ros::NodeHandle nh;

    Boid boid;

    boid.uplimit =10;
    boid.lowlimit=5;

    ros::Subscriber name_sub  = nh.subscribe("/gazebo/model_states", 1, &Boid::get_name, &boid);
    ros::Subscriber sep_sub   = nh.subscribe("/gazebo/model_states", 1, &Boid::separation, &boid);
    ros::Subscriber coh_sub   = nh.subscribe("/gazebo/model_states", 1, &Boid::cohesion, &boid);
    ros::Subscriber alig_sub  = nh.subscribe("/gazebo/model_states", 1, &Boid::alignment, &boid);
    ros::Subscriber vel_sub   = nh.subscribe("/gazebo/model_states", 1, &Boid::get_vel, &boid);

    ros::Publisher  vel_pub  = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);

    string namespace_String = ros::this_node::getNamespace();
    namespace_String.erase(0, 1);
    namespace_String.erase(0, 1);
    boid.ns = namespace_String;


    geometry_msgs::TwistStamped velMsg;

     struct timeval time;
     gettimeofday(&time,NULL);

     // microsecond has 1 000 000
     // Assuming you did not need quite that accuracy
     // Also do not assume the system clock has that accuracy.
     srand((time.tv_sec * 1000) + (time.tv_usec / 1000));

    int counter(0);

    ros::Rate rate(10);

    double random_val = 2*double(rand())/double(RAND_MAX)-1;
    cout<<" random_val "<<random_val<<endl;

    velMsg.twist.linear.x = -sin(random_val);
    velMsg.twist.linear.y = cos(random_val);

    velMsg.header.frame_id="world";
                //cout<<" vx_sep "<<velMsg.twist.linear.x<<endl;
                //cout<<" vy_sep "<<velMsg.twist.linear.y<<endl;
    vel_pub.publish(velMsg);
    //cout<<"only separation"<<endl;

    int vmax(1);


    double phi;
    double v;
    while (ros::ok())
    {
        if(counter>5)
        {
            if    (boid.nocoh && !boid.nosep)
            {
                velMsg.twist.linear.x=boid.vx+boid.vx_sep+boid.vx_align;
                velMsg.twist.linear.y=boid.vy+boid.vy_sep+boid.vy_align;


                if (sqrt(pow(velMsg.twist.linear.x,2)+pow(velMsg.twist.linear.y,2)) > vmax)
                {
                    v = vmax;
                }
                else
                {
                     v = sqrt(pow(velMsg.twist.linear.x,2)+pow(velMsg.twist.linear.y,2));
                }


                //double phi_i=atan2(boid.vy,boid.vx);
                double phi=-atan2(boid.vx+boid.vx_sep+boid.vx_align,boid.vy+boid.vy_sep+boid.vy_align);

                //cout<<"boid.vy_sep "<<boid.vy_sep<<endl;
                //cout<<"boid.vy "<<boid.vy<<endl;

                //cout<<"boid.vx_sep "<<boid.vx_sep<<endl;
                //cout<<"boid.vx "<<boid.vx<<endl;


                //cout<<"phi "<<phi*180/3.14<<endl;


                velMsg.twist.linear.x=-v*sin(phi);
                velMsg.twist.linear.y=v*cos(phi);

                velMsg.header.stamp = ros::Time::now();
                velMsg.header.frame_id="world";
                //cout<<" vx_sep "<<velMsg.twist.linear.x<<endl;
                //cout<<" vy_sep "<<velMsg.twist.linear.y<<endl;
                vel_pub.publish(velMsg);
                cout<<"only separation"<<endl;
            }
            else if (!boid.nocoh && boid.nosep)
            {
                velMsg.twist.linear.x=boid.vx+boid.vx_coh+boid.vx_align;
                velMsg.twist.linear.y=boid.vy+boid.vy_coh+boid.vy_align;

                if (sqrt(pow(velMsg.twist.linear.x,2)+pow(velMsg.twist.linear.y,2)) > vmax)
                {
                    v = vmax;
                }
                else
                {
                     v = sqrt(pow(velMsg.twist.linear.x,2)+pow(velMsg.twist.linear.y,2));
                }


                //cout<<"final v "<<v<<endl;

                //double phi=atan2(boid.vy+boid.vy_coh+boid.vy_align,boid.vx+boid.vx_coh+boid.vx_align);

                //phi=phi-3.14;
                //if (phi<-3.14)
                //{
                    //phi=phi+(2*3.14);
        //cout<<"cohesion angle [-180 180] "<<Boid::cohesion_angle<<endl;
                //}

                //cout<<"boid.vy_coh "<<boid.vy_coh<<endl;
                //cout<<"boid.vy "<<boid.vy<<endl;

                //cout<<"boid.vx_coh "<<boid.vx_coh<<endl;
                //cout<<"boid.vx "<<boid.vx<<endl;


                //cout<<"phi "<<phi*180/3.14<<endl;

                double phi_=-atan2(boid.vx+boid.vx_coh+boid.vx_align,boid.vy+boid.vy_coh+boid.vy_align);
                //cout<<"phi_ "<<atan2(boid.vx_coh,boid.vy_coh)*180/3.14<<endl;
                //cout<<"boid vy "<<velMsg.twist.linear.y<<endl;
                //cout<<"boid vx "<<velMsg.twist.linear.x<<endl;

                /*cout<<"cohesion phi "<<phi<<endl;

                cout<<"atan y "<<boid.vy+boid.vy_coh+boid.vy_align<<endl;
                cout<<"atan x "<<boid.vx+boid.vx_coh+boid.vx_align<<endl;

                double phi_f=atan2(boid.vy+boid.vy_coh+boid.vy_align,boid.vx+boid.vx_coh+boid.vx_align);
                cout<<"cohesion phi final "<<phi_i<<endl;
                if( phi_f-phi_i>phi_max)
                {
                    phi = phi_i+phi_max;
                }
                else if ( phi_f-phi_i<-phi_max)
                {
                     phi = phi_i-phi_max;
                }
                else
                {
                     phi = phi_f;
                }/*/


                velMsg.twist.linear.x=-v*sin(phi_);
                velMsg.twist.linear.y=v*cos(phi_);




                velMsg.header.stamp = ros::Time::now();
                velMsg.header.frame_id="world";
                //cout<<" vx_coh "<<velMsg.twist.linear.x<<endl;
                //cout<<" vy_coh "<<velMsg.twist.linear.y<<endl;
                vel_pub.publish(velMsg);
                cout<<"only cohesion"<<endl;
            }
            else if (!boid.nocoh && !boid.nosep)
            {
                velMsg.twist.linear.x=boid.vx+boid.vx_coh+boid.vx_sep+boid.vx_align;
                velMsg.twist.linear.y=boid.vy+boid.vy_coh+boid.vy_sep+boid.vy_align;

                if (sqrt(pow(velMsg.twist.linear.x,2)+pow(velMsg.twist.linear.y,2)) > vmax)
                {
                    v = vmax;
                }
                else
                {
                     v = sqrt(pow(velMsg.twist.linear.x,2)+pow(velMsg.twist.linear.y,2));
                }

                double phi_=-atan2(boid.vx+boid.vx_coh+boid.vx_sep+boid.vx_align,boid.vy+boid.vy_coh+boid.vy_sep+boid.vy_align);

                velMsg.twist.linear.x=-v*sin(phi_);
                velMsg.twist.linear.y=v*cos(phi_);


                velMsg.header.stamp = ros::Time::now();
                velMsg.header.frame_id="world";
                //cout<<" vx_all "<<velMsg.twist.linear.x<<endl;
                //cout<<" vy_all "<<velMsg.twist.linear.y<<endl;

                vel_pub.publish(velMsg);
                cout<<"all rules"<<endl;
            }
            else
            {
                cout<<"something else"<<endl;
            }
        }
        else
        {
            counter++;
            //vel_pub.publish(velMsg);
        }
        ros::spinOnce();
        rate.sleep();
    }
}
