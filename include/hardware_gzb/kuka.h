/**
 *      @file  kuka.h
 *      @brief  Class for kuka robot
 *
 *
 *     @author  joão moura, Joao.Moura@ed.ac.uk
 *
 *   @internal
 *     Created  28-Feb-2017
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Edinburgh Centre for Robotics
 * ===============================================================
 */
#ifndef _KUKA_
#define _KUKA_

// standard string
#include <string>
// Messages
//#include <ipab_lwr_msgs/FriState.h>		// commented by KC
//#include <ipab_lwr_msgs/FriCommandJointPosition.h>	// 	KC
#include "iiwa_msgs/JointPosition.h"		// Added by KC
#include "iiwa_msgs/JointPositionVelocity.h"		// Added by KC
#include "iiwa_ros.h"				// Added by KC
#include <geometry_msgs/WrenchStamped.h>
// Kinematics library
#include <serial_arm_lib/serialArmKin.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>	// gazebo

#ifndef VAL_SAT
#define VAL_SAT(val,min,max)    ((val)>(max)?(max):((val)<(min)?(min):(val)))
#endif

#define PI 3.14159265

class Kuka
{
	public:
		Kuka(); // class constructor
		virtual ~Kuka(); // destructor
		// Kinematic related variables
		SerialArmKin kin;
		Eigen::Matrix<double, Eigen::Dynamic, 1> jointValues, jointVels; // read joint positions/velocities
		Eigen::Matrix<double, 7, 1> poseValues; // read Cartesian pose
		Eigen::Matrix<double, Eigen::Dynamic, 1> jointTargets, jointVelTargets; // added by KC
		bool gotRobotState;
		bool gotInitJoint;		// added by KC
		/**¬
		 * @brief Initialize class
		 * @param toolLink: name of the last link
		 */
		void Init(std::string &toolLink);
		/**¬
		 * @brief Commands the robot arm joint positions
		 * @param q: joint positions
		 */
		int setJointPos(const Eigen::Matrix<double, Eigen::Dynamic, 1> &q);
		/**¬
		 * @brief Commands the robot arm joint velocities
		 * @param q: joint velocities
		 */
		int setCartPose(const Eigen::Matrix<double, 7, 1> &Cpose);
		int setJointVelocity(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq);
		void setCartVelTime(const Eigen::Matrix<double, 6, 1> &Cvel, double &dt);
		void setJointVel(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq);
		void setJointVelTime(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq, double &dt);
		void setJointAccTime(const Eigen::Matrix<double, Eigen::Dynamic, 1> &ddq, double &dt);
		void setHome(const Eigen::Matrix<double, Eigen::Dynamic, 1> &jointInit);
		void GetNullVel(Eigen::Matrix<double, Eigen::Dynamic, 1> &nullVel, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q, const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq, const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian, const Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobianInv);
	private:
		//-----------------------------------------
		// ROS variables
		//-----------------------------------------
		geometry_msgs::PoseStamped msgCartPose_;
		iiwa_msgs::JointVelocity msgJointsVels_;
		iiwa_msgs::JointPosition msgJointsState_;
		sensor_msgs::JointState msgJointsState_rviz_;
		trajectory_msgs::JointTrajectory g; // gazebo
		ros::Publisher pubCartPose_;
		ros::Publisher pubJointsVels_;
		ros::Publisher pubJointsState_;
		ros::Publisher pubJointsState_rviz_;
		ros::Publisher arm_pub;	// gazebo
		ros::Subscriber arm_sub;// gazebo
		ros::Subscriber subJointsState_;
		ros::Subscriber subCartsState_;
		ros::NodeHandle nh_; // ROS node handle
		/**¬
		 * @brief Reads joints position and stores it in private vector
		 * @param msg: robot state message
		 */
		void callbackJointsState_(const iiwa_msgs::JointPositionVelocity::ConstPtr& msg);
		void callbackGazeboJointsState_(const sensor_msgs::JointState::ConstPtr& msg);//gazebo
		void callbackCartsState_(const geometry_msgs::PoseStamped::ConstPtr& msg);
};
		// Time
		ros::Time currT, prevT;
		ros::Duration deltaT;

#endif // _KUKA_
