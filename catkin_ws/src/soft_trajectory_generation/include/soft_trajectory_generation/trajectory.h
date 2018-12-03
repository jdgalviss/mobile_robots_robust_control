#ifndef TRAJECTORY_H_INCLUDED
#define TRAJECTORY_H_INCLUDED
#include <soft_trajectory_generation/Trajectory.h>
#include <cmath> // std::abs
#include<ros/ros.h>
class Trajectory
{
  public:
	Trajectory(ros::NodeHandle &nh);
	~Trajectory();
	void calculateSpline(float tf, float s0_x, float sf_x, float vi_x, float vf_x, float s0_y, float sf_y, float vi_y, float vf_y, float s0_th, float sf_th, float vi_th, float vf_th);
    void splineParameters(float tf, float sf, float vi, float vf, float ai, float af, float (&a)[5][3]);
    void publishTrajectory(double current_time);
    std::vector<float> splineProfile(double current_time, float (&a)[5][3]);
    soft_trajectory_generation::Trajectory trajectoryMsg;

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