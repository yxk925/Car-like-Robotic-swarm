#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include "plan_utils/traj_container.hpp"
#include "kinematics_simulator/MincoTraj.h"
#include "kinematics_simulator/SingleMinco.h"
#include "kinematics_simulator/Trajectory.h"

#define OMINIDIRECTION 0
#define DIFFERENTIAL   1
#define ACKERMANN 	   2

using namespace std;

// ros interface
ros::Subscriber traj_sub;
ros::Publisher cmd_vel_push;

ros::Timer pub_timer;

// simulator variables
plan_utils::TrajContainer newest_trajectory;
std::default_random_engine generator;
std::normal_distribution<double> distribution{0.0,1.0};

int car_id = 0;
double x = 0.0;
double y = 0.0;
double yaw = 0.0;
double vx = 0.0;
double vy = 0.0;
double w = 0.0;
bool rcv_cmd = false;
bool rcv_traj = false;


// utils
void normYaw(double& th)
{
	while (th > M_PI)
		th -= M_PI * 2;
	while (th < -M_PI)
		th += M_PI * 2;
}

Eigen::Vector2d guassRandom2d(double std)
{
	return std * Eigen::Vector2d(distribution(generator), distribution(generator));
}

Eigen::Vector3d guassRandom3d(double std)
{
	return std * Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));
}

double guassRandom(double std)
{
	return std * distribution(generator);
}

void rcvTrajCallBack(const kinematics_simulator::TrajectoryConstPtr traj_msg)
{
    if (traj_msg->minco_path.trajs.size() == 0) {
        ROS_INFO("received empty traj, clear local traj");
        newest_trajectory.clearSingul();
        rcv_traj = false;
        return;
    }

    ROS_INFO("received trajs");
	plan_utils::MinJerkOpt jerk_opter;
    // std::vector<plan_utils::LocalTrajData> minco_traj;
    plan_utils::TrajContainer surround_traj;
    std::vector<bool> reverse;
    double total_time = 0.0;
    for(int i = 0; i < traj_msg->minco_path.trajs.size(); i++)
    {
        double start_time = traj_msg->minco_path.trajs.at(i).start_time.toSec();
        kinematics_simulator::SingleMinco sm = traj_msg->minco_path.trajs[i];
        Eigen::MatrixXd posP(2, sm.pos_pts.size() - 2);
        Eigen::VectorXd T(sm.t_pts.size());
        Eigen::MatrixXd head(2, 3), tail(2, 3);
        const int N = sm.t_pts.size();
        reverse.push_back(sm.reverse);
        int direction = sm.reverse?-1:1;

        for(int j = 1; j < (int)sm.pos_pts.size() - 1; j++)
        {
            posP(0, j - 1) = sm.pos_pts[j].x;
            posP(1, j - 1) = sm.pos_pts[j].y;
        }
        for(int j = 0; j < (int)sm.t_pts.size(); j++)
        {
            T(j) = sm.t_pts[j];
        }
        head.row(0) = Eigen::Vector3d(sm.head_x.x, sm.head_x.y, sm.head_x.z);
        head.row(1) = Eigen::Vector3d(sm.head_y.x, sm.head_y.y, sm.head_y.z);
        tail.row(0) = Eigen::Vector3d(sm.tail_x.x, sm.tail_x.y, sm.tail_x.z);
        tail.row(1) = Eigen::Vector3d(sm.tail_y.x, sm.tail_y.y, sm.tail_y.z);

        jerk_opter.reset(head, tail, N);
        jerk_opter.generate(posP, T);
        plan_utils::Trajectory traj = jerk_opter.getTraj(direction);

        surround_traj.addSingulTraj(traj, start_time, car_id);

        total_time += traj.getTotalDuration();
        // minco_traj.push_back(sur_traj);
    }
	newest_trajectory = surround_traj;	

    rcv_traj = true;
}

void transRoutine(const ros::TimerEvent &e)
{
    double time_now = ros::Time::now().toSec();

    if(!rcv_traj)
    {
        return;
    }

    plan_utils::Trajectory* cur_segment;
    int segment_Id = newest_trajectory.locateSingulId(time_now);
    double segment_start_time = newest_trajectory.singul_traj[segment_Id].start_time;
    double segment_end_time   = newest_trajectory.singul_traj[segment_Id].end_time;
    double pt_time;
    if(time_now - segment_end_time > 0) {
        ROS_INFO("segment time out, clear");
        newest_trajectory.clearSingul();
        rcv_traj = false;
    }
        
    pt_time = segment_end_time - segment_start_time;
    cur_segment = &newest_trajectory.singul_traj[segment_Id].traj;

    int singul;
    Eigen::Matrix2d B_h;
    B_h << 0, -1,
            1,  0;
    double cur_yaw, vel, angul_vel;
    Eigen::Vector2d sigma, dsigma, ddsigma;
    sigma   = cur_segment->getPos(pt_time);
    dsigma  = cur_segment->getdSigma(pt_time);
    ddsigma = cur_segment->getddSigma(pt_time);
    singul  = cur_segment->getSingul(pt_time);
    cur_yaw = cur_segment->getAngle(pt_time);
    vel = singul * dsigma.norm();
    angul_vel = (ddsigma.transpose() * B_h * dsigma)(0, 0) / dsigma.squaredNorm();

    x = sigma(0); y = sigma(1);
    yaw = cur_yaw;
    vx = vel; vy = 0;
    w = angul_vel;

    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = vx;
    cmd_vel.angular.z = w;

    cmd_vel_push.publish(cmd_vel);
}

// main loop
int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "cmd_trans_node");
    ros::NodeHandle nh("~");

	
	traj_sub = nh.subscribe("traj", 100, rcvTrajCallBack);	

    pub_timer = nh.createTimer(ros::Duration(0.01), transRoutine);

    cmd_vel_push = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	ros::spin();

    return 0;
}