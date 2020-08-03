/**
 *      @file   rexrov2.h
 *      @brief  Class for Rexrov2
 *
 *
 *     @author  Kamil Cetin, k.cetin@hw.ac.uk
 *
 *   @internal
 *     Created  15-May-2019
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Heriot-Watt University
 * ===============================================================
 */
#ifndef _REXROV2_
#define _REXROV2_

#include <math.h> // for sin cos
// print to terminal
#include <iostream>
// standard string
#include <string>
#include <fstream>
// ROS header
#include <ros/ros.h>
// Kinematics library
#include "copilotROSInterface/ROVPose.h"
#include "copilotROSInterface/NavSts.h"
#include "underwater_sensor_msgs/DVL.h"
#include <sensor_msgs/Imu.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense> // Library for vectors, matrices, and algebra operations

#ifndef VAL_SAT
#define VAL_SAT(val,min,max)    ((val)>(max)?(max):((val)<(min)?(min):(val)))
#endif

class Rexrov2
{
	public:
		Rexrov2(); // class constructor
		virtual ~Rexrov2(); // destructor
		// Kinematic related variables
		Eigen::Matrix<double, 3, 1> Rexrov2_rpy;
		Eigen::Matrix<double, 4, 1> Rexrov2_quat;
		Eigen::Matrix<double, 6, 1> Rexrov2_pos, Rexrov2_vel, Rexrov2_thrusters;
		Eigen::Matrix<double, 7, 1> Rexrov2_pose;
		Eigen::Matrix<double, 6, 6> Rexrov2_jac;
		bool gotROVState;
		/**¬
		 * @brief Initialize class
		 */
		void Init();

		int setRovVel(const Eigen::Matrix<double, 6, 1> &vel);
		
	private:
		//-----------------------------------------
		// ROS variables
		//-----------------------------------------
		ros::Subscriber rovpose_sub;
    	ros::Subscriber rovjoint_sub;
		ros::Publisher rovvel_pub;
		ros::NodeHandle nh_; // ROS node handle
		geometry_msgs::Twist msg;

		void rexrov2poseCallback(const nav_msgs::Odometry::ConstPtr& rexrov2pose);
		void rexrov2thrusterCallback(const sensor_msgs::JointState::ConstPtr& rexrov2thrusters);

		void quaternionToEulerAngles(const Eigen::Matrix<double, 3, 1> rpy_, const Eigen::Matrix<double, 4, 1> &quat_);
		//Eigen::Matrix<double, 6, 1> getJac(const Eigen::Matrix<double, 6, 1> &p);
};

#endif // _REXROV2_
