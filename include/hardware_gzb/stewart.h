/**
 *      @file   stewart.h
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
#ifndef _STEWART_
#define _STEWART_

#include <math.h> // for sin cos
// print to terminal
#include <iostream>
// standard string
#include <string>
// ROS header
#include <ros/ros.h>

// Messages
#include <geometry_msgs/WrenchStamped.h>
// Kinematics library
#include <serial_arm_lib/parallelKin.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense> // Library for vectors, matrices, and algebra operations

#ifndef VAL_SAT
#define VAL_SAT(val,min,max)    ((val)>(max)?(max):((val)<(min)?(min):(val)))
#endif

class Stewart
{
	public:
		Stewart(); // class constructor
		virtual ~Stewart(); // destructor
		// Kinematic related variables
		//parallelKin kin;
		Eigen::Matrix<double, 6, 1> poseValues; // read Cartesian positions
		Eigen::Matrix<double, 6, 1> velValues; // read Cartesian velocities
		Eigen::Matrix<double, 6, 1> poseTargets;
		bool gotPlatformState;
		/**¬
		 * @brief Initialize class
		 */
		void Init();
		/**¬
		 * @brief Commands the platform Cartesian Pose
		 * @param p: Cartesian Pose vector
		 */
		int setPose(const Eigen::Matrix<double, Eigen::Dynamic, 1> &p);
		
		int getPFK(Eigen::Matrix<double, 3, 1> &Ppos, Eigen::Matrix<double, 3, 3> &Prot, Eigen::Matrix<double, 3, 3> &Ptra, const Eigen::Matrix<double, Eigen::Dynamic, 1> &p);
		int getPjac(Eigen::Matrix<double, 6, 6> &Pjac, const Eigen::Matrix<double, Eigen::Dynamic, 1> &p);
		int getVel(Eigen::Matrix<double, 6, 1> &dot_Xp, const Eigen::Matrix<double, Eigen::Dynamic, 1> &Xp);
		int getVelRot(Eigen::Matrix<double, 6, 6> &dot_Prot6, const Eigen::Matrix<double, Eigen::Dynamic, 1> &p, const Eigen::Matrix<double, Eigen::Dynamic, 1> &dot_p);
		void pubBackPoseState_(Eigen::Matrix<double, 6, 1> &Xp, Eigen::Matrix<double, 6, 1> &dot_Xp);
		int filterStates(Eigen::Matrix<double, 6, 1> &filt_Xp, Eigen::Matrix<double, 6, 1> &filt_dot_Xp, const Eigen::Matrix<double, 6, 1> &Xp, const Eigen::Matrix<double, Eigen::Dynamic, 1> &dot_Xp);
	private:
		//-----------------------------------------
		// ROS variables
		//-----------------------------------------
		sensor_msgs::JointState msgPoseState_;
		sensor_msgs::JointState msgPposeState_rviz_;
		ros::Publisher pubPoseState_;
		ros::Publisher pubPposeState_rviz_;
		ros::Subscriber subPoseState_;
		ros::NodeHandle nh_; // ROS node handle
		/**¬
		 * @brief Reads cartesian pose and stores it in private vector
		 * @param msg: platform state message
		 */
		void callbackPoseState_(const sensor_msgs::JointState::ConstPtr& msg);
		// Time
		ros::Time currT, prevT;
		ros::Duration deltaT;
		// Filter
		int avsize = 10;
		double beta = 0.75;
		Eigen::Matrix<double, 6, 1> preXp, preVelFiltered, prePoseFiltered;
		Eigen::Matrix<double, 6, 10> preVelStored, prePoseStored;
		Eigen::Matrix<double, 6, 9> temVelStored, temPoseStored;
		
};

#endif // _STEWART_
