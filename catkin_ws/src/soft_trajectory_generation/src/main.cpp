#include <ros/console.h>
#include <ros/ros.h>
#include <time.h>
#include <cmath>
#include <soft_trajectory_generation/trajectory.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_control");
    ros::Time::init();
    ros::NodeHandle nh("~");
    int rate = 40;
    ros::Rate r(rate);
    //=====================Variables====================

    //=====================Controller===================
    Trajectory *trajectory = new Trajectory(nh);
    //=====================Publishers===================
    //ros::Publisher trajectoryPub = nh.advertise<soft_trajectory_generation::Trajectory>("/trajectory/current", 100);  
    //=====================Subscribers==================
    //ros::Subscriber configureSub = nh.subscribe<std_msgs::Bool>("/web_client/configure", 10, boost::bind(configuration, _1, speed_controller));
    //configuration(NULL, speed_controller);
    //====================Trajectory vertices===========
    //std::vector<float> x_ref{ 2.0, 2.0, -2.0, -2.0,  0.0 }
    //std::vector<float> y_ref{ 0.0, 4.0, 4.0, 0.0,  0.0 }
    //std::vector<float> yaw_ref{ 0.0, M_PI/2.0, M_PI, 3.0*M_PI / 4.0,  2.0*M_PI }
    //std::vector<float> time_ref{ 2.0, 4.0, 4.0, 4.0,  2.0 }
    std::vector<float> p0{0.0, 0.0, 0.0, 0.0};
    std::vector<float> p1{2.0, 0.0, 0.0, 2.0};
    std::vector<float> p2{2.0, 4.0, M_PI/2.0, 6.0};
    std::vector<float> p3{-2.0, 4.0, M_PI, 10.0};
    std::vector<float> p4{-2.0, 0.0, 3.0*M_PI/4.0, 14.0};
    std::vector<float> p5{0.0, 0.0, M_PI*2.0, 16.0};


    std::vector<std::vector<float> > points{p0, p1, p2, p3, p4, p5};
    std::vector<std::vector<float> >::iterator it;
    std::vector<std::vector<float> >::iterator it_prev;
    it_prev = points.begin();
    it = points.begin();
    ++it;
    std::vector<float> point = *it;
    std::vector<float> point_prev = *it_prev;
    ros::Time startTime = ros::Time::now();
    bool isCalculated = false;
    double t0 = 0.0;
    //======================Main loop===================
    while (nh.ok())
    {
        /****ros looping ***/
        if(it != points.end()){
            if( (ros::Time::now()-startTime).toSec() < point.at(3)){
                if(!isCalculated){
                    point = *it;
                    point_prev = *it_prev;
                    trajectory->calculateSpline((point.at(3)-t0), point_prev.at(0),  point.at(0),  0.0f,  0.0f, point_prev.at(1),  point.at(1),  0.0f,  0.0f, point_prev.at(2),  point.at(2),  0.0f,  0.0f );
                    isCalculated = true;
                }
                if(isCalculated){
                    trajectory->publishTrajectory((ros::Time::now()-startTime).toSec() - t0);
                }
            }
            else{
                //ROS_INFO("segmento terminado1");
                isCalculated = false;
                t0 = (double)point.at(3);
                //ROS_INFO("segmento terminado2");
                ++it;
                //ROS_INFO("segmento terminado2.5");
                if(it != points.end())
                    point = *it;
                //ROS_INFO("segmento terminado3");
                ++it_prev;
                //ROS_INFO("segmento terminado4");
                if(it_prev != points.end())
                    point_prev = *it_prev;
                //ROS_INFO("segmento terminado");
            }
        }
        else{
            if(!isCalculated){
                ROS_INFO("Trajectory finished");
                startTime = ros::Time::now();
                t0 = 0.0f;
                it = points.begin();
                ++it;
                it_prev = points.begin();
                point = *it;
                point_prev = *it_prev;
            }
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
