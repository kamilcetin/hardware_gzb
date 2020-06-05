/* Falcon to SW platform /copilot/ROVPos 
 * SW Platform will mimics the Falcon ROV motions with its  workspace
 * A sigmoid function with 0.1 limits is introduced for the limited workspace of the Platform 
 */

#include <hardware_gzb/falcon.h>

Falcon::Falcon()
{
}

Falcon::~Falcon()
{
}

void Falcon::Init()
{
	gotROVState = false; // did not get ROV state
	Fal_rpy.setZero();
	Fal_nav.setZero();
	
	// Subscribe from ROV
	falconpose_sub = nh_.subscribe<copilotROSInterface::ROVPose>("/copilot/ROVPose", 1, &Falcon::falconposeCallback, this);
    	falconnav_sub  = nh_.subscribe<copilotROSInterface::NavSts>("/copilot/NavSts",1 , &Falcon::falconNavCallback, this);
	falconDVL_sub  = nh_.subscribe<underwater_sensor_msgs::DVL>("/copilot/DVLSts",1 , &Falcon::falconDVLCallback, this);
	falconIMU_sub  = nh_.subscribe<sensor_msgs::Imu>("/razor_imu",1 , &Falcon::falconRazorIMUCallback, this);

}


void Falcon::falconposeCallback(const copilotROSInterface::ROVPose::ConstPtr& falconposeData)
{
	// Update the Fal_rpy every cycle
     	Fal_rpy(0) = falconposeData->r;
     	Fal_rpy(1) = falconposeData->p;
     	Fal_rpy(2) = falconposeData->y;

	if(!gotROVState) gotROVState = true;
}

void Falcon::falconNavCallback(const copilotROSInterface::NavSts::ConstPtr& falconNavData)
{
	// Update the Fal_nav every cycle
      	Fal_nav(0) = falconNavData->LocalPos.n;
      	Fal_nav(1) = falconNavData->LocalPos.e;
      	Fal_nav(2) = falconNavData->LocalPos.d;
      	Fal_nav(3) = falconNavData->AngularPos.r;
      	Fal_nav(4) = falconNavData->AngularPos.p;
      	Fal_nav(5) = falconNavData->AngularPos.y;

	if(!gotROVState) gotROVState = true;
}

void Falcon::falconDVLCallback(const underwater_sensor_msgs::DVL::ConstPtr& falconDVLData)
{
	// Update the Fal_dvl every cycle
	Fal_dvl(0) = falconDVLData-> wi_x_axis;
	Fal_dvl(1) = falconDVLData-> wi_y_axis;
	Fal_dvl(2) = falconDVLData-> wi_z_axis;
	Fal_dvl(3) = falconDVLData-> wi_error;
	Fal_dvl(4) = falconDVLData-> bi_x_axis;
	Fal_dvl(5) = falconDVLData-> bi_y_axis;
	Fal_dvl(6) = falconDVLData-> bi_z_axis;
	Fal_dvl(7) = falconDVLData-> bi_error;
	Fal_dvl(8) = falconDVLData-> ws_transverse;
	Fal_dvl(9) = falconDVLData-> ws_longitudinal;
	Fal_dvl(10) = falconDVLData-> ws_normal;
	Fal_dvl(11) = falconDVLData-> bs_transverse;
	Fal_dvl(12) = falconDVLData-> bs_longitudinal;
	Fal_dvl(13) = falconDVLData-> bs_normal;
	Fal_dvl(14) = falconDVLData-> we_east;
	Fal_dvl(15) = falconDVLData-> we_north;
	Fal_dvl(16) = falconDVLData-> we_upwards;
	Fal_dvl(17) = falconDVLData-> be_east;
	Fal_dvl(18) = falconDVLData-> be_north;
	Fal_dvl(19) = falconDVLData-> be_upwards;
	Fal_dvl(20) = falconDVLData-> wd_east;
	Fal_dvl(21) = falconDVLData-> wd_north;
	Fal_dvl(22) = falconDVLData-> wd_upwards;
	Fal_dvl(23) = falconDVLData-> wd_range;
	Fal_dvl(24) = falconDVLData-> wd_time;
	Fal_dvl(25) = falconDVLData-> bd_east;
	Fal_dvl(26) = falconDVLData-> bd_north;
	Fal_dvl(27) = falconDVLData-> bd_upwards;
	Fal_dvl(28) = falconDVLData-> bd_range;
	Fal_dvl(29) = falconDVLData-> bd_time;

	if(!gotROVState) gotROVState = true;
}

void Falcon::falconRazorIMUCallback(const sensor_msgs::Imu::ConstPtr& falconImuData)
{
	// Update the Fal_nav every cycle
      	Fal_imu(0) = falconImuData->angular_velocity.x;
      	Fal_imu(1) = falconImuData->angular_velocity.y;
      	Fal_imu(2) = falconImuData->angular_velocity.z;
      	Fal_imu(3) = falconImuData->linear_acceleration.x;
      	Fal_imu(4) = falconImuData->linear_acceleration.y;
      	Fal_imu(5) = falconImuData->linear_acceleration.z;
      	Fal_imu(6) = falconImuData->orientation.x;
      	Fal_imu(7) = falconImuData->orientation.y;
      	Fal_imu(8) = falconImuData->orientation.z;
      	Fal_imu(9) = falconImuData->orientation.w;

	if(!gotROVState) gotROVState = true;
}
