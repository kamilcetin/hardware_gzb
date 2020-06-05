/**
 *      @file  ur3.cpp
 * ===============================================================
 */
#include <hardware_gzb/ur3.h>

Ur3::Ur3()
{
}

Ur3::~Ur3()
{
}

void Ur3::Init(std::string &toolLink)
{
	gotRobotState = false; // did not get robot state
	gotInitJoint = false; // added by KC
	// Initialize kinematics class from the ROS robot_description parameter
	kin.initFromParam("robot_description",toolLink);
//	kin.initFromParam("/rexrov2/robot_description",toolLink);
	//msgJointsState_.joint_names = kin.jointNames;	// commented
	// Initialize joint position vector
	jointValues.resize(kin.nrJoints); jointValues.setZero();
	jointVels.resize(kin.nrJoints); jointVels.setZero();
	jointTargets.resize(kin.nrJoints); jointTargets.setZero();	// added by KC
	jointVelTargets.resize(kin.nrJoints); jointVelTargets.setZero();// added by KC
	// initial previous time
	//prevT = ros::Time::now();
	// Publish Kuka operation mode in joint position, joint velocity
	// gazebo with rexrov2 and kuka
 	//arm_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/rexrov2/PositionJointInterface_trajectory_controller/command",1,true);
	//arm_sub = nh_.subscribe<sensor_msgs::JointState>("/rexrov2/joint_states", 1, &Kuka::callbackGazeboJointsState_, this);
	// gazebo with only ur3
//	arm_pub = nh_.advertise<control_msgs::FollowJointTrajectoryGoal>("arm_controller/follow_joint_trajectory",1,true);
	arm_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1,true);
	arm_sub = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &Ur3::callbackGazeboJointsState_, this);

	g.header.stamp = ros::Time::now() ; //+ ros::Duration(0.1);
    	g.joint_names.push_back("shoulder_pan_joint");
	g.joint_names.push_back("shoulder_lift_joint");
 	g.joint_names.push_back("elbow_joint");
 	g.joint_names.push_back("wrist_1_joint");
 	g.joint_names.push_back("wrist_2_joint");
 	g.joint_names.push_back("wrist_3_joint");

}

// gazebo
void Ur3::callbackGazeboJointsState_(const sensor_msgs::JointState::ConstPtr& msg)
{
	if(kin.nrJoints==6){
		jointValues(0) = msg->position[0];
		jointValues(1) = msg->position[1];
		jointValues(2) = msg->position[2];
		jointValues(3) = msg->position[3];
		jointValues(4) = msg->position[4];
		jointValues(5) = msg->position[5];

		if(!gotRobotState) { 
			gotRobotState = true;
			std::cout << "!!! gotRobotState is " << gotRobotState << '\n';
		}
	}
	else ROS_WARN_STREAM("Joint position/velocity vector with wrong size");
}

// gazebo
int Ur3::setJointPos(const Eigen::Matrix<double, Eigen::Dynamic, 1> &q)
{
	if(gotRobotState) {

		g.points.resize(1);
	        int ind = 0;
        	g.points[ind].positions.resize(6);
        	for (size_t j = 0; j< 6; ++j){ 
        	    	g.points[ind].positions[j] = q(j);
        	}	
		g.points[ind].time_from_start = ros::Duration(0.02); //0.02 idi
		g.header.stamp = ros::Time::now() + ros::Duration(0);
		arm_pub.publish(g);
		//std::cout << " to be published " << '\n';
	}
	return 0;
}

void Ur3::setJointVelTime(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq, double &dt)
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


void Ur3::setHome(const Eigen::Matrix<double, Eigen::Dynamic, 1> &jointInit)
{
	Eigen::Matrix<double, Eigen::Dynamic, 1> q; q.resize(6);
	//q << 0.0, 0.0, 0.0, 1.5, 0.0, 0.0;
	q << jointInit;
	setJointPos(q);
}

