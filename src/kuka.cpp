/**
 *      @file  kuka.cpp
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
#include <hardware_gzb/kuka.h>

Kuka::Kuka()
{
}

Kuka::~Kuka()
{
}

void Kuka::Init(std::string &toolLink)
{
	gotRobotState = false; // did not get robot state
	gotInitJoint = false; // added by KC
	// Initialize kinematics class from the ROS robot_description parameter
	kin.initFromParam("robot_description",toolLink);
//	kin.initFromParam("/rexrov2/robot_description",toolLink);
	// fill in msgJointsState_ with joint names
	msgJointsState_rviz_.name = kin.jointNames;
	//msgJointsState_.joint_names = kin.jointNames;	// commented
	// Initialize joint position vector
	jointValues.resize(kin.nrJoints); jointValues.setZero();
	jointVels.resize(kin.nrJoints); jointVels.setZero();
	jointTargets.resize(kin.nrJoints); jointTargets.setZero();	// added by KC
	jointVelTargets.resize(kin.nrJoints); jointVelTargets.setZero();// added by KC
	// initial previous time
	prevT = ros::Time::now();
	// Publish Kuka operation mode in joint position, joint velocity
	//pubCartPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1, true);
	//pubJointsVels_ = nh_.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 1, true);
	//pubJointsState_ = nh_.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1, true);
	pubJointsState_rviz_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1, true);
	// Subscribe to robot state
	//subJointsState_ = nh_.subscribe<iiwa_msgs::JointPositionVelocity>("/iiwa/state/JointPositionVelocity", 1, &Kuka::callbackJointsState_, this); // comment here when gazebo run
	//subCartsState_ = nh_.subscribe<geometry_msgs::PoseStamped>("/iiwa/state/CartesianPose", 1, &Kuka::callbackCartsState_, this);

	// gazebo with rexrov2 and kuka
 	arm_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/rexrov2/PositionJointInterface_trajectory_controller/command",1,true);
	arm_sub = nh_.subscribe<sensor_msgs::JointState>("/rexrov2/joint_states", 1, &Kuka::callbackGazeboJointsState_, this);
	// gazebo with only kuka
	//arm_sub = nh_.subscribe<sensor_msgs::JointState>("/iiwa/joint_states", 1, &Kuka::callbackGazeboJointsState_, this);
	//arm_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/iiwa/PositionJointInterface_trajectory_controller/command",1,true);
	
    	g.joint_names.push_back("iiwa_joint_1");
    	g.joint_names.push_back("iiwa_joint_2");
    	g.joint_names.push_back("iiwa_joint_3");
		g.joint_names.push_back("iiwa_joint_4");
		g.joint_names.push_back("iiwa_joint_5");
    	g.joint_names.push_back("iiwa_joint_6");
    	g.joint_names.push_back("iiwa_joint_7");
	
}

// gazebo. original callback joints below
void Kuka::callbackGazeboJointsState_(const sensor_msgs::JointState::ConstPtr& msg)
{
	if(kin.nrJoints==7){
		jointValues(0) = msg->position[0];
		jointValues(1) = msg->position[1];
		jointValues(2) = msg->position[2];
		jointValues(3) = msg->position[3];
		jointValues(4) = msg->position[4];
		jointValues(5) = msg->position[5];
		jointValues(6) = msg->position[6];
		
		jointVels(0) = msg->velocity[0];
		jointVels(1) = msg->velocity[1];
		jointVels(2) = msg->velocity[2];
		jointVels(3) = msg->velocity[3];
		jointVels(4) = msg->velocity[4];
		jointVels(5) = msg->velocity[5];
		jointVels(6) = msg->velocity[6];
		
		// just for rexrov2+kuka sim.s
		jointValues(3) = jointValues(3) + PI/2;
		if(!gotRobotState) { 
			gotRobotState = true;
			std::cout << "!!! gotRobotState is " << gotRobotState << '\n';
		}
	}
	else ROS_WARN_STREAM("Joint position/velocity vector with wrong size");
}

// gazebo. The original one is above
int Kuka::setJointPos(const Eigen::Matrix<double, Eigen::Dynamic, 1> &q)
{
	if(gotRobotState) {

		g.points.resize(1);
	        int ind = 0;
        	g.points[ind].positions.resize(7);
        	g.points[ind].velocities.resize(7);
        	for (size_t j = 0; j< 7; ++j){ 
        	    g.points[ind].positions[j] = q(j);
        	}	
		g.points[ind].time_from_start = ros::Duration(0.02);
		 	
		// just for rexrov2+kuka sim.s
		g.points[ind].positions[3] = g.points[ind].positions[3]-PI/2;

		arm_pub.publish(g);
		//std::cout << " here3 " << '\n';
	}
	return 0;
}

void Kuka::setJointVelTime(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq, double &dt)
{
	Eigen::Matrix<double, Eigen::Dynamic, 1> q; q.resize(dq.size());
	//std::cout << " q size: " << q.size() << '\n';

	if(gotRobotState) { 
		// Initialize target position vector
		if(!gotInitJoint) { 
			for(int idx=0; idx<dq.size(); idx++) {
				jointTargets(idx) = jointValues(idx);	
				std::cout << "jointTargets(" << idx << ")= " << jointTargets(idx) << '\n';
			}
			gotInitJoint = true;
			std::cout << "!!! gotInitJoint is " << gotInitJoint << '\n';
		}
		for(int idx=0; idx<dq.size(); idx++) {
			jointTargets(idx) += dt*VAL_SAT(dq(idx), -kin.jointVelocity(idx), kin.jointVelocity(idx));
			q(idx) = jointTargets(idx);
		}
		//std::cout << " here2 " << '\n';
		setJointPos(q);
	}
}


void Kuka::setHome(const Eigen::Matrix<double, Eigen::Dynamic, 1> &jointInit)
{
	Eigen::Matrix<double, Eigen::Dynamic, 1> q; q.resize(7);
	//q << 0.0, 0.0, 0.0, 1.5, 0.0, -1.0, 0.0;
	q << jointInit;
	setJointPos(q);
}


/*int Kuka::setJointVelocity(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq)
{

	g.points.resize(1);
        int ind = 0;
        g.points[ind].positions.resize(7);
        g.points[ind].velocities.resize(7);
        for (size_t j = 0; j< 7; ++j){ 
            	g.points[ind].velocities[j] = dq(j);
        }	
	g.points[ind].time_from_start = ros::Duration(0.02);
	arm_pub.publish(g);
	//std::cout << " !!!BURDA!!! " << '\n';
	return 0;
}*/

/*
void Kuka::callbackJointsState_(const iiwa_msgs::JointPositionVelocity::ConstPtr& msg)
{
	if(kin.nrJoints==7){
		jointValues(0) = msg->position.a1;
		jointValues(1) = msg->position.a2;
		jointValues(2) = msg->position.a3;
		jointValues(3) = msg->position.a4;
		jointValues(4) = msg->position.a5;
		jointValues(5) = msg->position.a6;
		jointValues(6) = msg->position.a7;

		jointVels(0) = msg->velocity.a1;
		jointVels(1) = msg->velocity.a2;
		jointVels(2) = msg->velocity.a3;
		jointVels(3) = msg->velocity.a4;
		jointVels(4) = msg->velocity.a5;
		jointVels(5) = msg->velocity.a6;
		jointVels(6) = msg->velocity.a7;
		if(!gotRobotState) gotRobotState = true;
	}
	else ROS_WARN_STREAM("Joint position/velocity vector with wrong size");
}

void Kuka::callbackCartsState_(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	poseValues(0) = msg->pose.position.x;
	poseValues(1) = msg->pose.position.y;
	poseValues(2) = msg->pose.position.z;
	poseValues(3) = msg->pose.orientation.x;
	poseValues(4) = msg->pose.orientation.y;
	poseValues(5) = msg->pose.orientation.z;
	poseValues(6) = msg->pose.orientation.w;

	if(!gotRobotState) gotRobotState = true;
}


int Kuka::setJointPos(const Eigen::Matrix<double, Eigen::Dynamic, 1> &q)
{
	// Fill in message:
	msgJointsState_.header.stamp = ros::Time::now();
	//msgJointsState_.jointPosition.clear();	//commented by KC
	//msgJointsState_.position.clear();		//added by KC
	msgJointsState_.position.a1 = 0.0;
	msgJointsState_.position.a2 = 0.0;
	msgJointsState_.position.a3 = 0.0;
	msgJointsState_.position.a4 = 0.0;
	msgJointsState_.position.a5 = 0.0;
	msgJointsState_.position.a6 = 0.0;
	msgJointsState_.position.a7 = 0.0;

	msgJointsState_.position.a1 = VAL_SAT(q(0), kin.jointLowerLimits(0), kin.jointUpperLimits(0));
	msgJointsState_.position.a2 = VAL_SAT(q(1), kin.jointLowerLimits(1), kin.jointUpperLimits(1));
	msgJointsState_.position.a3 = VAL_SAT(q(2), kin.jointLowerLimits(2), kin.jointUpperLimits(2));
	msgJointsState_.position.a4 = VAL_SAT(q(3), kin.jointLowerLimits(3), kin.jointUpperLimits(3));
	msgJointsState_.position.a5 = VAL_SAT(q(4), kin.jointLowerLimits(4), kin.jointUpperLimits(4));
	msgJointsState_.position.a6 = VAL_SAT(q(5), kin.jointLowerLimits(5), kin.jointUpperLimits(5));
	msgJointsState_.position.a7 = VAL_SAT(q(6), kin.jointLowerLimits(6), kin.jointUpperLimits(6));

	msgJointsState_rviz_.header.stamp = ros::Time::now();	//added by JM
	msgJointsState_rviz_.position.clear();			//added by JM
	for(int idx=0; idx<7; idx++) msgJointsState_rviz_.position.push_back(VAL_SAT(q(idx), kin.jointLowerLimits(idx), kin.jointUpperLimits(idx)));	//added by JM
	pubJointsState_rviz_.publish(msgJointsState_rviz_);	//added by JM

	pubJointsState_.publish(msgJointsState_);

	return 0;
}


int Kuka::setJointVelocity(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq)
{
	// Fill in message:
	msgJointsVels_.header.stamp = ros::Time::now();
	msgJointsVels_.velocity.a1 = 0.0;
	msgJointsVels_.velocity.a2 = 0.0;
	msgJointsVels_.velocity.a3 = 0.0;
	msgJointsVels_.velocity.a4 = 0.0;
	msgJointsVels_.velocity.a5 = 0.0;
	msgJointsVels_.velocity.a6 = 0.0;
	msgJointsVels_.velocity.a7 = 0.0;

	msgJointsVels_.velocity.a1 = dq(0);
	msgJointsVels_.velocity.a2 = dq(1);
	msgJointsVels_.velocity.a3 = dq(2);
	msgJointsVels_.velocity.a4 = dq(3);
	msgJointsVels_.velocity.a5 = dq(4);
	msgJointsVels_.velocity.a6 = dq(5);
	msgJointsVels_.velocity.a7 = dq(6);
	pubJointsVels_.publish(msgJointsVels_);

	return 0;
}

int Kuka::setCartPose(const Eigen::Matrix<double, 7, 1> &desPose)
{
	// Fill in message:
	msgCartPose_.header.stamp = ros::Time::now();
	msgCartPose_.pose.position.x = 0.0;
	msgCartPose_.pose.position.y = 0.0;
	msgCartPose_.pose.position.z = 0.0;
	msgCartPose_.pose.orientation.x = 0.0;
	msgCartPose_.pose.orientation.y = 0.0;
	msgCartPose_.pose.orientation.z = 0.0;
	msgCartPose_.pose.orientation.w = 0.0;

	msgCartPose_.pose.position.x = desPose(0);
	msgCartPose_.pose.position.y = desPose(1);
	msgCartPose_.pose.position.z = desPose(2);
	msgCartPose_.pose.orientation.x = desPose(3);
	msgCartPose_.pose.orientation.y = desPose(4);
	msgCartPose_.pose.orientation.z = desPose(5);
	msgCartPose_.pose.orientation.w = desPose(6);
	pubCartPose_.publish(msgCartPose_);

	return 0;
}

void Kuka::setJointVel(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq)
{
	Eigen::Matrix<double, Eigen::Dynamic, 1> q; q.resize(dq.size());
	for(int idx=0; idx<dq.size(); idx++) q(idx) = jointValues(idx) + VAL_SAT(dq(idx), -kin.jointVelocity(idx), kin.jointVelocity(idx));

	//setJointPos(q);
	setJointVelocity(dq);
}

void Kuka::setJointVelTime(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq, double &dt)
{
	Eigen::Matrix<double, Eigen::Dynamic, 1> q; q.resize(dq.size());
	// Initialize target position vector
	if(!gotInitJoint) { 
		for(int idx=0; idx<dq.size(); idx++) jointTargets(idx) = jointValues(idx);	
		gotInitJoint = true;
		std::cout << "ONCE..." << '\n';
	}
	for(int idx=0; idx<dq.size(); idx++) {
		jointTargets(idx) += dt*VAL_SAT(dq(idx), -kin.jointVelocity(idx), kin.jointVelocity(idx));
		q(idx) = jointTargets(idx);
	}
	setJointPos(q);
}

// for SMC
void Kuka::setJointAccTime(const Eigen::Matrix<double, Eigen::Dynamic, 1> &ddq, double &dt)
{
	Eigen::Matrix<double, Eigen::Dynamic, 1> q; q.resize(ddq.size());
	// Initialize target position and target velocity vectors
	if(!gotInitJoint) { 
		for(int idx=0; idx<ddq.size(); idx++) jointTargets(idx) = jointValues(idx);
		jointVelTargets.setZero();	// for SMC
		gotInitJoint = true;
		std::cout << "ONCE...acc" << '\n';
	}
	for(int idx=0; idx<ddq.size(); idx++) {
		jointVelTargets(idx) += dt*ddq(idx);
		jointTargets(idx) += dt*VAL_SAT(jointVelTargets(idx), -kin.jointVelocity(idx), kin.jointVelocity(idx));
		q(idx) = jointTargets(idx);
	}
	setJointPos(q);
}
*/

/*void Kuka::setCartVelTime(const Eigen::Matrix<double, 6, 1> &Cvel, double &dt)
{
	/*Eigen::Matrix<double, 7, 1> Cpose;
	Eigen::Matrix<double, 6, 1> Cpos;
	
	////////////////////// How to obtain Cpos 6x1 through Rpose
	Cpos = Cpos + Cvel*dt;
	////////////////////// How to convert the resulted Cpos to quaternion based Cpose

	setCartPose(Cpose);
}*/


/*void Kuka::GetNullVel(Eigen::Matrix<double, Eigen::Dynamic, 1> &nullVel, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q, const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq, const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian, const Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobianInv)
{
	Eigen::Matrix<double, 7, 7> identity, null;
	double dt, obj_null;
		
	// Update time
	currT  = ros::Time::now();
	deltaT = currT - prevT;
	prevT  = currT;
	dt = deltaT.toSec();

	// Null space control
	identity.setIdentity();
	null = identity - jacobian.transpose() * jacobianInv.transpose();
	obj_null = sqrt((jacobian*jacobian.transpose()).determinant());
	//for(int idx=0; idx<dq.size(); idx++) nullVel(idx) = (obj_null/dt)/dq(idx);
	//nullVel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

}
*/
