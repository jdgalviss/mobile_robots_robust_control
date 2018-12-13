#ifndef TRAJECTORY_H_INCLUDED
#define TRAJECTORY_H_INCLUDED
#include <soft_trajectory_generation/Trajectory.h>
#include <cmath> // std::abs
#include <ros/ros.h>
//each point has {xf,yf,yaw0, yawF, vf, time(global)}
class Point
{
  public:
    Point(float x_f, float y_f, float yaw_0, float yaw_f, float v_f, float yaw_speed_f, float a_f, float yaw_acc_f, float t_f);
    float xf;
    float yf;
    float yaw0;
    float yawf;
    float vf;
    float tf;
    float d_yawf;
    float af;
    float d_d_yawf;
};
class Trajectory
{
  public:
    Trajectory(ros::NodeHandle &nh);
    ~Trajectory();
    void calculateSpline(Point point, Point point_prev);
    void splineParameters(float tf, float sf, float vi, float vf, float ai, float af, float (&a)[5][3]);
    void publishTrajectory(double current_time);
    std::vector<float> splineProfile(double current_time, float (&a)[5][3]);
    soft_trajectory_generation::Trajectory trajectoryMsg;
    float getX(double current_time);
    float getVx(double current_time);
    float getAx(double current_time);

  protected:
    float ax[5][3];
    float ay[5][3];
    float ath[5][3];
    float t1 = 0.0f;
    float t2 = 0.0f;
    float s0x, s0y, s0th;
    ros::NodeHandle n;
    ros::Publisher trajectoryPub;
};

#endif