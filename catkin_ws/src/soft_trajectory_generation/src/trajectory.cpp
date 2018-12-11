#include <soft_trajectory_generation/trajectory.h>

Trajectory::Trajectory( ros::NodeHandle &nh)
{
    n = nh;
    ROS_INFO("creating Trajectory");	
	if(trajectoryPub==NULL)trajectoryPub = nh.advertise<soft_trajectory_generation::Trajectory>("/trajectory/current", 100);  
	//if(trajectoryMsgPtr==NULL)trajectoryMsgPtr=&trajectoryMsg; 
//    if(skynetControlSub==NULL)skynetControlSub= getNh()->subscribe("/skynet/control", 100, &StreetCrossing::updateSkynetControl,this);
}
//void Trajectory::calculateSpline(float tf, float s0_x, float sf_x, float vi_x, float vf_x, float s0_y, float sf_y, float vi_y, float vf_y, float s0_th, float sf_th, float vi_th, float vf_th ){

void Trajectory::calculateSpline(Point point, Point point_prev){
    //ROS_INFO("tf = %0.4f, s0x=%0.4f, sfx=%0.4f, s0y=%0.4f, sfy=%0.4f", tf, s0_x, sf_x, s0_y, sf_y);
    float tf = point.tf - point_prev.tf;
    float sf_x = point.xf;
    float s0_x = point_prev.xf;
    float vi_x = point_prev.vf * cos(point_prev.yawf);
    float vf_x = point.vf * cos(point.yawf);
    float sf_y = point.yf;
    float s0_y = point_prev.yf;
    float vi_y = point_prev.vf * sin(point_prev.yawf);
    float vf_y = point.vf * sin(point.yawf);
    float sf_th = point.yawf;
    float s0_th = point.yaw0;
    float vi_th = 0.0;
    float vf_th = 0.0;
    splineParameters(tf, sf_x-s0_x, vi_x, vf_x, 0.0f, 0.0f, ax);
    splineParameters(tf, sf_y-s0_y, vi_y, vf_y, 0.0f, 0.0f, ay);
    splineParameters(tf, sf_th-s0_th, vi_th, vf_th, 0.0f, 0.0f, ath);
    s0x = s0_x;
    s0y = s0_y;
    s0th = s0_th;
}

void Trajectory::splineParameters(float tf, float sf, float vi, float vf, float ai, float af, float (&a)[5][3])
{
    //float t2 = tf*(1.0-4.0*m);
    //float v2 = (sf-t1*(vi+vf)+pow(t1,2)*(af-ai)/3.0)/(2*t1+t2);
    //float tf = sf / VX_MEAN;
    float m = 1.0f / 10.0f;
    t1 = tf * m;
    t2 = tf * (1.0 - 4.0 * m);
    float v2 = (sf - 2.0 * t1 * vi / 3.0 - t1 * vi / 3.0 - t1 * vf / 3.0 - 2.0 * t1 * vf / 3.0 - ai * pow(t1, 2.0) / 6.0 + af * pow(t1, 2.0) / 6.0 - ai * t1 * t1 / 6.0 + af * t1 * t1 / 6.0) / (t1 / 3.0 + 2.0 * t1 / 3.0 + t2 + 2.0 * t1 / 3.0 + t1 / 3.0);
    a[0][0] = vi;
    a[0][1] = ai / 2.0;
    a[0][2] = (2.0 * (v2 - vi - ai * t1) - ai * t1) / (6.0 * t1 * (t1 + t1));
    a[1][0] = (2.0 * (v2 * t1 + vi * t1) + ai * t1 * t1) / (2.0 * (t1 + t1));
    a[1][1] = (2.0 * (v2 - vi) - ai * t1) / (2.0 * (t1 + t1));
    a[1][2] = -(2.0 * (v2 - vi) - ai * t1) / (6.0 * t1 * (t1 + t1));
    a[2][0] = v2;
    a[2][1] = 0.0;
    a[2][2] = 0.0;
    a[3][0] = v2;
    a[3][1] = 0.0;
    a[3][2] = (-af * t1 - 2.0 * v2 + 2.0 * vf) / (6.0 * t1 * (t1 + t1));
    a[4][0] = (2.0 * (v2 * t1 + vf * t1) - af * t1 * t1) / (2.0 * (t1 + t1));
    a[4][1] = (2.0 * (vf - v2) - af * t1) / (2.0 * (t1 + t1));
    a[4][2] = (2.0 * (v2 - vf + af * t1) + af * t1) / (6.0 * t1 * (t1 + t1));
}

void Trajectory::publishTrajectory(double current_time){
    //ROS_INFO("current_time= %0.4f", current_time);
    trajectoryMsg.header.stamp = ros::Time::now();
    std::vector<float> profile = splineProfile(current_time, ax);
    trajectoryMsg.x = profile.at(0) + s0x;
    trajectoryMsg.vx = profile.at(1);
    trajectoryMsg.ax = profile.at(2);

    profile = splineProfile(current_time, ay);
    trajectoryMsg.y = profile.at(0) + s0y;
    trajectoryMsg.vy = profile.at(1);
    trajectoryMsg.ay = profile.at(2);

    profile = splineProfile(current_time, ath);
    trajectoryMsg.yaw = profile.at(0) +s0th;
    trajectoryMsg.yaw_speed = profile.at(1);
    trajectoryMsg.yaw_acceleration = profile.at(2);

    trajectoryMsg.vx_local = std::sqrt( std::pow(trajectoryMsg.vx,2) + std::pow(trajectoryMsg.vy,2) );

    trajectoryPub.publish(trajectoryMsg);

}

float Trajectory::getX(double current_time){
    //ROS_INFO("current_time= %0.4f", current_time);
    std::vector<float> profile = splineProfile(current_time, ax);
    return profile.at(0);
}

float Trajectory::getVx(double current_time){
    //ROS_INFO("current_time= %0.4f", current_time);
    std::vector<float> profile = splineProfile(current_time, ax);
    return profile.at(1);
}

std::vector<float> Trajectory::splineProfile(double current_time,float (&a)[5][3]){
    float vel = 0.0f;
    float pos = 0.0f;
    float acel = 0.0f;
    float lastValue = 0.0f;
    std::vector<float> profile;
    if (current_time >= 0.0f && current_time < t1)
    {
        acel = 2.0f*a[0][1] + 6.0f*a[0][2]*(current_time);
        vel = a[0][0] + 2 * a[0][1] * current_time + 3 * a[0][2] * pow(current_time, 2);
        pos = a[0][0] * current_time + a[0][1] * pow(current_time, 2) + a[0][2] * pow(current_time, 3.0);
    }
    else if (current_time >= t1 && current_time < 2.0f * t1)
    {
        lastValue = a[0][0] * t1 + a[0][1] * pow(t1, 2) + a[0][2] * pow(t1, 3.0);
        acel = 2.0f*a[1][1] + 6.0f*a[1][2]*(current_time - t1);
        vel = a[1][0] + 2 * a[1][1] * (current_time - t1) + 3 * a[1][2] * pow(current_time - t1, 2.0f);
        pos = lastValue + a[1][0] * (current_time-t1) + a[1][1] * pow((current_time-t1), 2.0f) + a[1][2] * pow((current_time-t1), 3.0f);
    }
    else if (current_time >= 2.0f * t1 && current_time < 2.0f * t1 + t2)
    {
        lastValue = a[0][0] * t1 + a[0][1] * pow(t1, 2) + a[0][2] * pow(t1, 3.0) + a[1][0] * (2.0f * t1 - t1) + a[1][1] * pow((2.0f * t1 - t1), 2) + a[1][2] * pow((2.0f * t1 - t1), 3.0);
        acel = 2.0f*a[2][1] + 6.0f*a[2][2]*(current_time - 2.0f * t1);
        pos = lastValue + a[2][0] * (current_time - 2.0f * t1) + a[2][1] * pow((current_time - 2.0f * t1), 2) + a[2][2] * pow((current_time - 2.0f * t1), 3.0);
        vel = a[2][0] + 2 * a[2][1] * (current_time - 2.0f * t1) + 3 * a[2][2] * pow((current_time - 2.0f * t1), 2);
    }
    else if (current_time >= 2.0f * t1 + t2 && current_time < 3.0f * t1 + t2)
    {
        lastValue = a[0][0] * t1 + a[0][1] * pow(t1, 2) + a[0][2] * pow(t1, 3.0) + a[1][0] * (2.0f * t1 - t1) + a[1][1] * pow((2.0f * t1 - t1), 2) + a[1][2] * pow((2.0f * t1 - t1), 3.0) + a[2][0] * ((2.0f * t1 + t2) - 2.0f * t1) + a[2][1] * pow(((2.0f * t1 + t2) - 2.0f * t1), 2) + a[2][2] * pow(((2.0f * t1 + t2) - 2.0f * t1), 3.0);
        acel = (2*a[3][1]*(current_time - (2.0f * t1 + t2))+6*a[3][2]*((current_time - (2.0f * t1 + t2))));
        pos = lastValue + a[3][0] * (current_time - (2.0f * t1 + t2)) + a[3][1] * pow((current_time - (2.0f * t1 + t2)), 2) + a[3][2] * pow((current_time - (2.0f * t1 + t2)), 3.0);
        vel = a[3][0] + 2 * a[3][1] * (current_time - (2.0f * t1 + t2)) + 3 * a[3][2] * pow((current_time - (2.0f * t1 + t2)), 2);
    }
    else if (current_time >= 3.0f * t1 + t2 && current_time < 4.0f * t1 + t2)
    {
        lastValue = a[0][0] * t1 + a[0][1] * pow(t1, 2) + a[0][2] * pow(t1, 3.0) + a[1][0] * (2.0f * t1 - t1) + a[1][1] * pow((2.0f * t1 - t1), 2) + a[1][2] * pow((2.0f * t1 - t1), 3.0) + a[2][0] * ((2.0f * t1 + t2) - 2.0f * t1) + a[2][1] * pow(((2.0f * t1 + t2) - 2.0f * t1), 2) + a[2][2] * pow(((2.0f * t1 + t2) - 2.0f * t1), 3.0) + a[3][0] * ((3.0f * t1 + t2) - (2.0f * t1 + t2)) + a[3][1] * pow(((3.0f * t1 + t2) - (2.0f * t1 + t2)), 2) + a[3][2] * pow(((3.0f * t1 + t2) - (2.0f * t1 + t2)), 3.0);
        acel = 2*a[4][1]*(current_time - (3.0f * t1 + t2))+6*a[4][2]*(current_time - (3.0f * t1 + t2));
        pos = lastValue + a[4][0] * (current_time - (3.0f * t1 + t2)) + a[4][1] * pow((current_time - (3.0f * t1 + t2)), 2) + a[4][2] * pow((current_time - (3.0f * t1 + t2)), 3.0);
        vel = a[4][0] + 2 * a[4][1] * (current_time - (3.0f * t1 + t2)) + 3 * a[4][2] * pow((current_time - (3.0f * t1 + t2)), 2);
    }
    else
    {
        lastValue = a[0][0] * t1 + a[0][1] * pow(t1, 2) + a[0][2] * pow(t1, 3.0) + a[1][0] * (2.0f * t1 - t1) + a[1][1] * pow((2.0f * t1 - t1), 2) + a[1][2] * pow((2.0f * t1 - t1), 3.0) + a[2][0] * ((2.0f * t1 + t2) - 2.0f * t1) + a[2][1] * pow(((2.0f * t1 + t2) - 2.0f * t1), 2) + a[2][2] * pow(((2.0f * t1 + t2) - 2.0f * t1), 3.0) + a[3][0] * ((3.0f * t1 + t2) - (2.0f * t1 + t2)) + a[3][1] * pow(((3.0f * t1 + t2) - (2.0f * t1 + t2)), 2) + a[3][2] * pow(((3.0f * t1 + t2) - (2.0f * t1 + t2)), 3.0) + a[4][0] * ((4.0f * t1 + t2) - (3.0f * t1 + t2)) + a[4][1] * pow(((4.0f * t1 + t2)  - (3.0f * t1 + t2)), 2) + a[4][2] * pow(((4.0f * t1 + t2)  - (3.0f * t1 + t2)), 3.0);
        pos = lastValue; //+ a[4][0] * (4.0f * t1 + t2) + a[4][1] * pow((4.0f * t1 + t2), 2) + a[4][2] * pow((4.0f * t1 + t2), 3.0);
        vel = 0.0f;
        acel = 0.0f;
    }
    profile.push_back(pos);
    profile.push_back(vel);
    profile.push_back(acel);

    return profile;
}

Trajectory::~Trajectory()
{
    ROS_INFO("Killing soft trajectory");
}

Point::Point(float x_f, float y_f, float yaw_0, float yaw_f, float v_f, float t_f){
    xf = x_f;
    yf = y_f;
    yaw0 = yaw_0;
    yawf = yaw_f;
    vf = v_f;
    tf = t_f;
}