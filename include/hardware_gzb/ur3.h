/**
 *      @file  ur3.h
 *      @brief  Class for UR3 robot
 * ===============================================================
 */
#ifndef _UR3_
#define _UR3_

// standard string
#include <string>
//#include "iiwa_msgs/JointPosition.h"
//#include "iiwa_msgs/JointPositionVelocity.h"
//#include "iiwa_ros.h"
//#include <geometry_msgs/WrenchStamped.h>
// Kinematics library
#include <serial_arm_lib/serialArmKin.h>
//#include <sensor_msgs/JointState.h>
//#include <trajectory_msgs/JointTrajectory.h>	// gazebo

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h> // to get tool_velocity
#include <tf2_msgs/TFMessage.h>
#include <trajectory_msgs/JointTrajectory.h>

#ifndef VAL_SAT
#define VAL_SAT(val,min,max)    ((val)>(max)?(max):((val)<(min)?(min):(val)))
#endif

class Ur3
{
	public:
		Ur3(); // class constructor
		virtual ~Ur3(); // destructor
		// Kinematic related variables
		SerialArmKin kin;
		Eigen::Matrix<double, Eigen::Dynamic, 1> jointValues, jointVels; // read joint positions/velocities
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
		int setJointVelocity(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq);
		void setJointVel(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq);
		void setJointVelTime(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq, double &dt);
		void setHome(const Eigen::Matrix<double, Eigen::Dynamic, 1> &jointInit);
		void GetNullVel(Eigen::Matrix<double, Eigen::Dynamic, 1> &nullVel, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q, const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq, const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian, const Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobianInv);
	private:
		//-----------------------------------------
		// ROS variables
		//-----------------------------------------
		ros::NodeHandle nh_; // ROS node handle

		trajectory_msgs::JointTrajectory g; // gazebo
		//control_msgs::FollowJointTrajectoryGoal g; // header file FollowJointTrajectoryActionGoal? ask to HT!

		ros::Publisher arm_pub;	// gazebo
		ros::Subscriber arm_sub;// gazebo

		void callbackGazeboJointsState_(const sensor_msgs::JointState::ConstPtr& msg);//gazebo
};
		// Time
		//ros::Time currT, prevT;
		//ros::Duration deltaT;

#endif // _UR3_
