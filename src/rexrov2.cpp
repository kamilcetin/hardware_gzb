/*
    @file  rexrov2.cpp
  ===============================================================
*/
#include <hardware_gzb/rexrov2.h>

Rexrov2::Rexrov2()
{
}

Rexrov2::~Rexrov2()
{
}

void Rexrov2::Init()
{
	gotROVState = false; // did not get robot state

	rovpose_sub = nh_.subscribe<nav_msgs::Odometry>("/rexrov2/pose_gt", 1, &Rexrov2::rexrov2poseCallback, this);
    rovjoint_sub = nh_.subscribe<sensor_msgs::JointState>("/rexrov2/joint_states", 1, &Rexrov2::rexrov2thrusterCallback, this);
	rovvel_pub = nh_.advertise<geometry_msgs::Twist>("/rexrov2/cmd_vel", 1, true);


	/*Rexrov2_pos(0) = rexrov2pose->pose.pose.position.x;
    Rexrov2_pos(1) = rexrov2pose->pose.pose.position.y;
    Rexrov2_pos(2) = rexrov2pose->pose.pose.position.z;
    Rexrov2_pose(3)   = rexrov2pose->pose.pose.orientation.x;
    Rexrov2_pose(4)   = rexrov2pose->pose.pose.orientation.y;
    Rexrov2_pose(5)   = rexrov2pose->pose.pose.orientation.z;
    Rexrov2_pose(6)   = rexrov2pose->pose.pose.orientation.w;

    //Rexrov2_quat << Rexrov2_pose(3), Rexrov2_pose(4), Rexrov2_pose(5), Rexrov2_pose(6);
    //Rexrov2_rpy = quaternionToEulerAngles(Rexrov2_quat);
    //Rexrov2_pos(3) = Rexrov2_rpy(0); Rexrov2_pos(4) = Rexrov2_rpy(1); Rexrov2_pos(5) = Rexrov2_rpy(2);

    Rexrov2_vel(0)   = rexrov2pose->twist.twist.linear.x;
    Rexrov2_vel(1)   = rexrov2pose->twist.twist.linear.y;
    Rexrov2_vel(2)   = rexrov2pose->twist.twist.linear.z;
    Rexrov2_vel(3)   = rexrov2pose->twist.twist.angular.x;
    Rexrov2_vel(4)   = rexrov2pose->twist.twist.angular.y;
    Rexrov2_vel(5)   = rexrov2pose->twist.twist.angular.z;

    // desired position is set to the initial pose
    //des_pos = Rpos;
*/
}

void Rexrov2::rexrov2poseCallback(const nav_msgs::Odometry::ConstPtr& rexrov2pose)
{
	if (rexrov2pose == NULL){
      Rexrov2_pos.setZero();
      Rexrov2_pose.setZero();
      Rexrov2_vel.setZero();
      Rexrov2_rpy.setZero();
    }else{
      Rexrov2_pose(0) = rexrov2pose->pose.pose.position.x;
      Rexrov2_pose(1) = rexrov2pose->pose.pose.position.y;
      Rexrov2_pose(2) = rexrov2pose->pose.pose.position.z;
      Rexrov2_pose(3)   = rexrov2pose->pose.pose.orientation.x;
      Rexrov2_pose(4)   = rexrov2pose->pose.pose.orientation.y;
      Rexrov2_pose(5)   = rexrov2pose->pose.pose.orientation.z;
      Rexrov2_pose(6)   = rexrov2pose->pose.pose.orientation.w;

      //Rexrov2_quat << Rexrov2_pose(3), Rexrov2_pose(4), Rexrov2_pose(5), Rexrov2_pose(6);
      //quaternionToEulerAngles(Rexrov2_rpy,Rexrov2_quat);
      //Rexrov2_pos(3) = Rexrov2_rpy(0); Rexrov2_pos(4) = Rexrov2_rpy(1); Rexrov2_pos(5) = Rexrov2_rpy(2);

      Rexrov2_vel(0)   = rexrov2pose->twist.twist.linear.x;
      Rexrov2_vel(1)   = rexrov2pose->twist.twist.linear.y;
      Rexrov2_vel(2)   = rexrov2pose->twist.twist.linear.z;
      Rexrov2_vel(3)   = rexrov2pose->twist.twist.angular.x;
      Rexrov2_vel(4)   = rexrov2pose->twist.twist.angular.y;
      Rexrov2_vel(5)   = rexrov2pose->twist.twist.angular.z;
    }

}

void Rexrov2::rexrov2thrusterCallback(const sensor_msgs::JointState::ConstPtr& rexrov2thrusters){
	if (rexrov2thrusters == NULL){
      	Rexrov2_thrusters.setZero();
    }else{
      	Rexrov2_thrusters(0) = rexrov2thrusters->position[26];
      	Rexrov2_thrusters(1) = rexrov2thrusters->position[27];
      	Rexrov2_thrusters(2) = rexrov2thrusters->position[28];
      	Rexrov2_thrusters(3) = rexrov2thrusters->position[29];
      	Rexrov2_thrusters(4) = rexrov2thrusters->position[30];
      	Rexrov2_thrusters(5) = rexrov2thrusters->position[31];
    }
}
/*
//---- Calculates euler angles (x, y, z) from quaternion (x, y, z, w)
void Rexrov2::quaternionToEulerAngles(const Eigen::Matrix<double, 3, 1> rpy_, const Eigen::Matrix<double, 4, 1> &quat_) {
  //Eigen::Matrix<double, 3, 1> rpy_; 
  double x,y,z,w,x2,y2,z2,w2;
  x = quat_(0,0);  y = quat_(1,0); z = quat_(2,0);  w = quat_(3,0);
  x2 = x*x;  y2 = y*y; z2 = z*z;  w2 = w*w;
  Eigen::Matrix<double, 3, 3> rot;
  rot << w2+x2-y2-z2, 2*x*y-2*w*z, 2*x*z+2*w*y, 2*x*y+2*w*z, w2-x2+y2-z2, 2*y*z-2*w*x, 2*x*z-2*w*y, 2*y*z+2*w*x, w2-x2-y2+z2;

 	double sy = sqrt(rot(0,0) * rot(0,0) +  rot(1,0) * rot(1,0) );
	bool singular = sy < 1e-6;	
	if (!singular) {
	        rpy_(0,0) << atan2(rot(2,1) , rot(2,2));
	        rpy_(1,0) = atan2(-rot(2,0), sy);
	        rpy_(2,0) = atan2(rot(1,0), rot(0,0));
	} else {
        	rpy_(0,0) = atan2(-rot(1,2), rot(1,1));
	        rpy_(1,0) = atan2(-rot(2,0), sy);
        	rpy_(2,0) = 0;
    	}

      //return rpy_;
}
*/

/*
Eigen::Matrix<double, 6, 1> getJac(const Eigen::Matrix<double, 6, 1> &p)
{
	Eigen::Matrix<double, 3, 3> rot0, rot1, rot2;
  	rot0.setZero();
  	rot1 << cos(p(5))*cos(p(4)), cos(p(5))*sin(p(4))*sin(p(3))-sin(p(5))*cos(p(3)), cos(p(3))*sin(p(4))*cos(p(5))+sin(p(3))*sin(p(5)),
  		sin(p(5))*cos(p(4)), sin(p(5))*sin(p(4))*sin(p(3))+cos(p(5))*cos(p(3)), cos(p(3))*sin(p(4))*sin(p(5))-sin(p(3))*cos(p(5)),
  		-sin(p(4))	   , cos(p(4))*sin(p(3))			      , cos(p(4))*cos(p(3));
  	rot2 << 1, sin(p(3))*tan(p(4)), cos(p(3))*tan(p(4)), 0, cos(p(3)), -sin(p(3)), 0, sin(p(3))/cos(p(4)), cos(p(3))/cos(p(4));
  
  	Eigen::Matrix<double, 6, 6> J;
  	J << rot1, rot0, rot0, rot2;

  	return J;
}
*/
// gazebo
int Rexrov2::setRovVel(const Eigen::Matrix<double, 6, 1> &vel){
	
	//publish the message
    msg.linear.x = vel(0);
    msg.linear.y = vel(1);
    msg.linear.z = vel(2);
    msg.angular.x = vel(3);
    msg.angular.y = vel(4);
    msg.angular.z = vel(5);
	rovvel_pub.publish(msg);
	
	return 0;
}


/*
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

*/