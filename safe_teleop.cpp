/**
 * @file safe_teleop.cpp
 * @brief Safe teleoperation library implementation
 * Created by rakesh on 28/09/18.
 */
#include <limits>
#include <safe_teleop/safe_teleop.h>
namespace safe_teleop
{

SafeTeleop::SafeTeleop() :
  is_shutdown_(false),
  max_cmd_vel_age_(1.0),
  max_linear_vel_(1.0),
  max_angular_vel_(1.0),
  linear_vel_increment_(0.05),
  angular_vel_increment_(0.05),
  laser_safety_check_angle_(0.25),
  min_safety_impact_time_(0.5),
  min_safety_distance_(0.5),
  linear_vel_(0.0),
  angular_vel_(0.0),
  linear_speed_(0.0),              ///< Current linear velocity
  angular_speed_(0.0),              ///< Current angular velocity

  last_command_timestamp_(0.0)
{
  ros::NodeHandle global_nh;
  cmd_vel_pub_ = global_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  // The subscriber callback is set to the laserScanCallback method of the instantiated object of this class
  laser_scan_sub_ = global_nh.subscribe("scan", 5, &SafeTeleop::laserScanCallback, this);

  run_thread_ = boost::thread(&SafeTeleop::run, this);
  displayCurrentSpeeds();
}

SafeTeleop::~SafeTeleop()
{
  shutdown();
  // wait for the run thread to terminate
  run_thread_.join();

  geometry_msgs::Twist zero_cmd_vel;
  zero_cmd_vel.linear.x = 0;
  zero_cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(zero_cmd_vel);
}

void SafeTeleop::run()
{
  ros::Rate r(10);
  while (ros::ok() && !is_shutdown_)
  {
    auto current_timestamp = ros::Time::now().toSec();

    auto last_cmd_vel_age = current_timestamp - last_command_timestamp_;

    if (last_cmd_vel_age > max_cmd_vel_age_)
    {
      linear_vel_ = 0;
      ROS_WARN_THROTTLE(1.0, "Timeout not implemented\r");
    }
    else
    {
      auto is_safe = checkSafety(static_cast<double>(linear_vel_));
      ROS_WARN_THROTTLE(1.0, "command velocity publishing not implemented\r");
    }

    geometry_msgs::Twist new_cmd_vel;
    new_cmd_vel.linear.x = linear_vel_;
    new_cmd_vel.angular.z = angular_vel_;
    cmd_vel_pub_.publish(new_cmd_vel);

    ROS_INFO_THROTTLE(1.0,"%f",(double)linear_vel_);
    r.sleep();
  }
}

void SafeTeleop::moveForward()
{

  linear_vel_ =linear_speed_ * 1.0;
  last_command_timestamp_ = ros::Time::now().toSec();

  ROS_WARN("Method implemented\r");

}

void SafeTeleop::moveBackward()
{
  linear_vel_ =linear_speed_ * (-1.0);
  last_command_timestamp_ = ros::Time::now().toSec();
  ROS_WARN("Method implemented\r");
}

void SafeTeleop::rotateClockwise()
{
  angular_vel_ = angular_speed_ * (1.0);
  last_command_timestamp_ = ros::Time::now().toSec();
  ROS_WARN("Method implemented\r");
}

void SafeTeleop::rotateCounterClockwise()
{
  
  angular_vel_=angular_speed_ * (-1.0);
  last_command_timestamp_ = ros::Time::now().toSec();
  ROS_WARN("Method implemented\r");
}

void SafeTeleop::stop()
{
  linear_vel_ =0.0;
  angular_vel_ =0.0;
  last_command_timestamp_ = ros::Time::now().toSec();
  ROS_WARN("Method implemented\r");
}


void SafeTeleop::increaseLinearSpeed()
{
  linear_speed_ =linear_speed_ + linear_vel_increment_;
  last_command_timestamp_ = ros::Time::now().toSec();
  ROS_WARN("Method implemented\r");
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseLinearSpeed()
{
  linear_speed_ =linear_speed_ - linear_vel_increment_;
  last_command_timestamp_ = ros::Time::now().toSec();
  ROS_WARN("Method implemented\r");
  displayCurrentSpeeds();
}

void SafeTeleop::increaseAngularSpeed()
{
  angular_speed_ =angular_speed_ + angular_vel_increment_;
  last_command_timestamp_ = ros::Time::now().toSec();
  ROS_WARN("Method implemented\r");
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseAngularSpeed()
{
  angular_speed_ =angular_speed_ - angular_vel_increment_;
  last_command_timestamp_ = ros::Time::now().toSec();
  ROS_WARN("Method implemented\r");
  displayCurrentSpeeds();
}

bool SafeTeleop::checkSafety(double linear_vel)
{
  auto laser_scan = getLaserScan();
  auto inc = 360.0/laser_scan.ranges.size();
  auto angle = 0.0;
  if(linear_vel_ >= max_linear_vel_)
  	linear_vel_ = max_linear_vel_;
  if(angular_vel_ >= max_angular_vel_)
  	angular_vel_ = max_angular_vel_;
  auto s=laser_scan.ranges;
  for(int i=0;s.size()>i;i++){
	angle = i*inc -180.0;  	
	if((angle > -15.0 && angle < 15.0) || (angle > 165.0 && angle < 195.0))
		if(s[i]<=min_safety_distance_)
			stop();	
  }
  	
  ROS_INFO_THROTTLE(1.0,"%f,%f,%f",laser_scan.angle_min,laser_scan.angle_max,laser_scan.angle_increment);
  
  ROS_WARN("Method not implemented\r");
}

} // namespace safe_teleop_node


