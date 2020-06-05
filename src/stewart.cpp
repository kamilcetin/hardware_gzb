/**
 *      @file   stewart.cpp
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
#include <hardware_gzb/stewart.h>
#include "funcLibrary/J_stewart.cpp"

Stewart::Stewart()
{
}

Stewart::~Stewart()
{
}

void Stewart::Init()
{
	gotPlatformState = false; // did not get platform state
	// Initialize vectors
	poseValues.setZero();
	velValues.setZero();
	poseTargets.setZero();
	// LP filter
	preVelFiltered.setZero();
	prePoseFiltered.setZero();
	// Average filter
	temVelStored.setZero();
	temPoseStored.setZero();
	preVelStored.setZero();
	prePoseStored.setZero();
	// Publish Stewart operation mode in position
	pubPoseState_ = nh_.advertise<sensor_msgs::JointState>("/Platform_command", 1, true);
	pubPposeState_rviz_ = nh_.advertise<sensor_msgs::JointState>("/Ppose_states", 1, true);
	// Subscribe to robot state
	subPoseState_ = nh_.subscribe<sensor_msgs::JointState>("/Platform_state", 1, &Stewart::callbackPoseState_, this);
	// Initial time
	prevT = ros::Time::now();
	// initial platform velocity
	preXp << 0.0, 0.0, 0.948, 0.0, 0.0, 0.0;
}

void Stewart::callbackPoseState_(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int idx=0; idx<poseValues.size(); idx++){
		poseValues(idx) = msg->position[idx];
		velValues(idx) = msg->velocity[idx];
	}
	poseValues(2) = poseValues(2) + 0.948;	// ATTENTION: when you publish it, subtract the same value

	if(!gotPlatformState) gotPlatformState = true;
	
	/*Eigen::Matrix<double, 6, 1> Xp, dot_Xp;
	for(int idx=0; idx<6; idx++){
		Xp(idx) = poseValues(idx);
		dot_Xp(idx) = velValues(idx);
	}
	pubBackPoseState_(Xp, dot_Xp);*/
}

int Stewart::filterStates(Eigen::Matrix<double, 6, 1> &filt_Xp, Eigen::Matrix<double, 6, 1> &filt_dot_Xp, const Eigen::Matrix<double, 6, 1> &Xp, const Eigen::Matrix<double, Eigen::Dynamic, 1> &dot_Xp)
{
	// LP filter for position
	//poseValues = beta * poseValues + (1 - beta) * prePoseFiltered;
	//prePoseFiltered = poseValues;
	// Average filter for velocity
	temPoseStored =  prePoseStored.block(0,0,6,avsize-1);
    	prePoseStored.block(0,1,6,avsize-1) = temPoseStored;
    	prePoseStored.block(0,0,6,1) = Xp;
    	for(int i=0; i<6 ; i++) filt_Xp(i,0) = prePoseStored.block(i,0,1,avsize).sum()/avsize;

	// LP filter for velocity
	//velValues = beta * velValues + (1 - beta) * preVelFiltered;
	//preVelFiltered = velValues;
	// Average filter for velocity
	temVelStored =  preVelStored.block(0,0,6,avsize-1);
    	preVelStored.block(0,1,6,avsize-1) = temVelStored;
    	preVelStored.block(0,0,6,1) = dot_Xp;
    	for(int j=0; j<6 ; j++) filt_dot_Xp(j,0) = preVelStored.block(j,0,1,avsize).sum()/avsize;

	return 0;
}

/*void Stewart::pubBackPoseState_(const Eigen::Matrix<double, 6, 1> &Xp, const Eigen::Matrix<double, 6, 1> &dot_Xp)
{
	msgPposeState_rviz_.header.stamp = ros::Time::now();
	msgPposeState_rviz_.position.clear();	
	msgPposeState_rviz_.velocity.clear();	
	for(int idx=0; idx<6; idx++) {
		msgPposeState_rviz_.position(idx) = Xp(idx);
		msgPposeState_rviz_.velocity(idx) = dot_Xp(idx);
	}
	pubJointsState_rviz_.publish(msgPposeState_rviz_);
}*/

int Stewart::getVel(Eigen::Matrix<double, 6, 1> &dot_Xp, const Eigen::Matrix<double, Eigen::Dynamic, 1> &Xp)
{
	/*// Update time
	double dt;
	currT  = ros::Time::now();
	deltaT = currT - prevT;
	prevT  = currT;
	dt = deltaT.toSec();
	// Update position differences
	Eigen::Matrix<double, 6, 1> deltaXp;
	deltaXp = Xp - preXp;
	preXp = Xp;
	dot_Xp = deltaXp / dt;
	
	return 0;*/
}

int Stewart::setPose(const Eigen::Matrix<double, Eigen::Dynamic, 1> &p)
{
	// Subtract Offset for p(2) before publishing a target position p
	//p(2) = p(2) - 0.948;
	// Fill in message:
	msgPoseState_.header.stamp = ros::Time::now();
	msgPoseState_.position.clear();
	for(int idx=0; idx<poseValues.size(); idx++){
		msgPoseState_.position[idx] = 0.0;
	}
	pubPoseState_.publish(msgPoseState_);
	// 
	msgPposeState_rviz_.header.stamp = ros::Time::now();
	msgPposeState_rviz_.position.clear();
	for(int idx=0; idx<poseValues.size(); idx++){ 
		//msgPposeState_rviz_.position.push_back(p);	// KC: Motions icinde henuz platforma komut gonderen komut yok, o yuzden &p bos.
	}
	
	pubPposeState_rviz_.publish(msgPposeState_rviz_);

	return 0;
} 

int Stewart::getPFK(Eigen::Matrix<double, 3, 1> &Ppos, Eigen::Matrix<double, 3, 3> &Prot, Eigen::Matrix<double, 3, 3> &Ptra, const Eigen::Matrix<double, Eigen::Dynamic, 1> &p)
{
	Ppos << p(0), p(1), p(2);	// X-p(3)=phi, Y-p(4)=theta, Z-p(5)=psi
	Prot << cos(p(5))*cos(p(4)), cos(p(5))*sin(p(4))*sin(p(3))-sin(p(5))*cos(p(3)), cos(p(3))*sin(p(4))*cos(p(5))+sin(p(3))*sin(p(5)),
		sin(p(5))*cos(p(4)), sin(p(5))*sin(p(4))*sin(p(3))+cos(p(5))*cos(p(3)), cos(p(3))*sin(p(4))*sin(p(5))-sin(p(3))*cos(p(5)),
		-sin(p(4))	   , cos(p(4))*sin(p(3))			      , cos(p(4))*cos(p(3));
	Ptra << 1, sin(p(3))*tan(p(4)), cos(p(3))*tan(p(4)), 0, cos(p(3)), -sin(p(3)), 0, sin(p(3))/cos(p(4)), cos(p(3))/cos(p(4));
	//Ptra << 1, 0, -sin(p(4)), 0, cos(p(3)), cos(p(4))*sin(p(3)), 0, -sin(p(3)), cos(p(4))*cos(p(3));
	return 0;

}

int Stewart::getPjac(Eigen::Matrix<double, 6, 6> &Pjac, const Eigen::Matrix<double, Eigen::Dynamic, 1> &p)
{
	Pjac = J_stewart(p);

	return 0;
}

int Stewart::getVelRot(Eigen::Matrix<double, 6, 6> &dot_Prot6, const Eigen::Matrix<double, Eigen::Dynamic, 1> &p, const Eigen::Matrix<double, Eigen::Dynamic, 1> &dot_p)
{
	Eigen::Matrix<double, 3, 3> dot_Prot, dot_Ptra;

	dot_Prot << -sin(p(5))*cos(p(4))*dot_p(5)-cos(p(5))*sin(p(4))*dot_p(4), 
                     -sin(p(5))*sin(p(4))*sin(p(3))*dot_p(5)+cos(p(5))*cos(p(4))*sin(p(3))*dot_p(4)+cos(p(5))*sin(p(4))*cos(p(3))*dot_p(3)-cos(p(5))*cos(p(3))*dot_p(5)+sin(p(5))*sin(p(3))*dot_p(3), 
                     -sin(p(3))*sin(p(4))*cos(p(5))*dot_p(3)+cos(p(3))*cos(p(4))*cos(p(5))*dot_p(4)-cos(p(3))*sin(p(4))*sin(p(5))*dot_p(5)+cos(p(3))*sin(p(5))*dot_p(3)+sin(p(3))*cos(p(5))*dot_p(5),
		      cos(p(5))*cos(p(4))*dot_p(5)-sin(p(5))*sin(p(4))*dot_p(4), 
                      cos(p(5))*sin(p(4))*sin(p(3))*dot_p(5)+sin(p(5))*cos(p(4))*sin(p(3))*dot_p(4)+sin(p(5))*sin(p(4))*cos(p(3))*dot_p(3)-sin(p(5))*cos(p(3))*dot_p(5)-cos(p(5))*sin(p(3))*dot_p(3), 
                     -sin(p(3))*sin(p(4))*sin(p(5))*dot_p(3)+cos(p(3))*cos(p(4))*sin(p(5))*dot_p(4)+cos(p(3))*sin(p(4))*cos(p(5))*dot_p(5)-cos(p(3))*cos(p(5))*dot_p(3)+sin(p(3))*sin(p(5))*dot_p(5),
		     -cos(p(4))*dot_p(4), 
                     -sin(p(4))*sin(p(3))*dot_p(4)+cos(p(4))*cos(p(3))*dot_p(3), 
                     -sin(p(4))*cos(p(3))*dot_p(4)-cos(p(4))*sin(p(3))*dot_p(3);

	dot_Ptra << 0, cos(p(3))*tan(p(4))*dot_p(3)+sin(p(3))*(1/(cos(p(4))*cos(p(4))))*dot_p(4), -sin(p(3))*tan(p(4))*dot_p(3)+cos(p(3))*(1/(cos(p(4))*cos(p(4))))*dot_p(4), 0, -sin(p(3))*dot_p(3), -cos(p(3))*dot_p(3), 0, cos(p(3))*dot_p(3)/cos(p(4))+sin(p(3))*(2*sin(p(4))/(cos(2*p(4))+1))*dot_p(4), -sin(p(3))*dot_p(3)/cos(p(4))+cos(p(3))*(2*sin(p(4))/(cos(2*p(4))+1))*dot_p(4);

	dot_Prot6 << dot_Prot, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), dot_Ptra;

	return 0;
}
