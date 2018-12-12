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
    Trajectory *circle_angle = new Trajectory(nh);
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
    //each point has {xf,yf,yaw0, yawF, vf, time(global)}
    //=============================defininf square trajectory=====================
    Point p0(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    Point p1(2.0, 0.0, 0.0, 0.0, 0.0, 2.0);
    Point p2(2.0, 4.0, M_PI / 2.0, M_PI / 2.0, 0.0, 6.0);
    Point p3(-2.0, 4.0, M_PI, M_PI, 0.0, 10.0);
    Point p4(-2.0, 0.0, 3.0 * M_PI / 2.0, 3.0 * M_PI / 2.0, 0.0, 14.0);
    Point p5(0.0, 0.0, M_PI * 2.0, M_PI * 2.0, 0.0, 16.0);
    //===========================defining circle trajectory=================
    float radius = 2.0f;
    float final_time = 20.0f;
    Point initial_angle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    Point final_angle(2*M_PI, 0.0, 0.0, 0.0, 0.0, final_time);
    circle_angle->calculateSpline(final_angle, initial_angle);
    std::vector<Point> circle_points;
    circle_points.push_back(initial_angle);
    ros::Time startTime = ros::Time::now();
    float yaw_0 = 0.0f;
    for (float t = 0; t <= final_time; t=t+0.1){
        
        float angle = circle_angle->getX(t);
        float x = radius * cos(angle-M_PI/2);
        float y = radius * sin(angle-M_PI/2) + radius;
        float vf = radius * circle_angle->getVx(t);
        Point point(x, y, yaw_0, angle, vf, t);
        yaw_0 = angle;
        circle_points.push_back(point);
        ROS_INFO("t: %0.4f", t);
        ROS_INFO("angle: %0.4f", angle);    
    }

    //std::vector<Point> points{p0, p1, p2, p3, p4, p5};
    std::vector<Point> points = circle_points;
    std::vector<Point>::iterator it;
    std::vector<Point>::iterator it_prev;
    it_prev = points.begin();
    it = points.begin();
    ++it;
    Point point = *it;
    Point point_prev = *it_prev;
    startTime = ros::Time::now();
    bool isCalculated = false;
    double t0 = 0.0;
    //======================Main loop===================
    while (nh.ok())
    {
        /****ros looping ***/
        if (it != points.end())
        {
            if ((ros::Time::now() - startTime).toSec() < point.tf)
            {
                if (!isCalculated)
                {
                    point = *it;
                    point_prev = *it_prev;
                    //void Trajectory::calculateSpline(float tf, float s0_x, float sf_x, float vi_x, float vf_x, float s0_y, float sf_y, float vi_y, float vf_y, float s0_th, float sf_th, float vi_th, float vf_th ){
                    trajectory->calculateSpline(point, point_prev);
                    isCalculated = true;
                }
                if (isCalculated)
                {
                    trajectory->publishTrajectory((ros::Time::now() - startTime).toSec() - t0);
                }
            }
            else
            {
                isCalculated = false;
                t0 = (double)point.tf;
                ++it;
                if (it != points.end())
                    point = *it;
                ++it_prev;
                if (it_prev != points.end())
                    point_prev = *it_prev;
            }
        }
        else
        {
            if (!isCalculated)
            {
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
