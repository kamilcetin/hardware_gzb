/**
 *      @file   falcon.h
 *      @brief  Class for Stewart Platform
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
#ifndef _FALCON_
#define _FALCON_

#include <math.h> // for sin cos
// print to terminal
#include <iostream>
// standard string
#include <string>
// ROS header
#include <ros/ros.h>
// Kinematics library
#include "copilotROSInterface/ROVPose.h"
#include "copilotROSInterface/NavSts.h"
#include "underwater_sensor_msgs/DVL.h"
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense> // Library for vectors, matrices, and algebra operations

#ifndef VAL_SAT
#define VAL_SAT(val,min,max)    ((val)>(max)?(max):((val)<(min)?(min):(val)))
#endif

class Falcon
{
	public:
		Falcon(); // class constructor
		virtual ~Falcon(); // destructor
		// Kinematic related variables
		Eigen::Matrix<double, 3, 1> Fal_rpy;
		Eigen::Matrix<double, 6, 1> Fal_nav;
		Eigen::Matrix<double, 30, 1> Fal_dvl;
		Eigen::Matrix<double, 10, 1> Fal_imu;
		bool gotROVState;
		/**¬
		 * @brief Initialize class
		 */
		void Init();
		
	private:
		//-----------------------------------------
		// ROS variables
		//-----------------------------------------
		ros::Subscriber falconpose_sub;
		ros::Subscriber falconnav_sub;
		ros::Subscriber falconDVL_sub;
		ros::Subscriber falconIMU_sub;
		ros::NodeHandle nh_; // ROS node handle
		/**¬
		 * @brief Reads Falcon Pose and Navigation data
		 * @param msg: ROV state message
		 */
		void falconposeCallback(const copilotROSInterface::ROVPose::ConstPtr& falconposeData);
		void falconNavCallback(const copilotROSInterface::NavSts::ConstPtr& falconNavData);
		void falconDVLCallback(const underwater_sensor_msgs::DVL::ConstPtr& falconDVLData);
		void falconRazorIMUCallback(const sensor_msgs::Imu::ConstPtr& falconImuData);
};

#endif // _FALCON_
