#include <BotControl/BotControl.hpp>

BotControl::BotControl(ros::NodeHandle& nh)
{
  if(!loadParam()){
    ROS_ERROR("Error in loading the parameters.");
    ros::requestShutdown();
  }
  nh_ = nh;
  target_sub_ = nh_.subscribe<geometry_msgs::Point>("/pathplan/target", 1, &BotControl::targetCallBack, this);
  curr_sub_ = nh_.subscribe<geometry_msgs::Point>("/pathplan/curr", 1, &BotControl::currCallBack, this);
  //control_pub_ = nh_.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);
  control_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  
  error_pos_ = 0;
  error_heading_ = 0;
  error_pos_prev_ = 0;
  error_heading_prev_ = 0;
  I_heading_ = 0;
  I_pos_ = 0;
  D_heading_ = 0;
  D_pos_ = 0;

  ROS_INFO("bot_control_node initialized successfully.");
  
}


void BotControl::targetCallBack(const geometry_msgs::PointConstPtr& target_msg)
{
  if(!flag){
    target_x_prev_ = target_msg->x;
    target_y_prev_ = target_msg->y;
    goal_reached_ = target_msg->z;
    flag = true;
  } else{
    target_x_ = target_msg->x;
    target_y_ = target_msg->y;
    goal_reached_ = target_msg->z;
  }
  
  if(goal_reached_ && error_pos_ <= 0.1) {
    ROS_INFO("Goal Reached!");
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;
    control_pub_.publish(cmd);
    
    return;
  } else{
    if(error_pos_ <= 0.1) {
      target_x_prev_ = target_x_;
      target_y_prev_ = target_y_;
      
      error_pos_prev_ = 0;
      error_heading_prev_ = 0;  
      I_heading_ = 0;
      I_pos_ = 0;
      D_heading_ = 0;
      D_pos_ = 0;

    }
    controlPub();
  }
  
}

void BotControl::currCallBack(const geometry_msgs::PointConstPtr& curr_msg)
{
  pos_x_ = curr_msg->x;
  pos_y_ = curr_msg->y;
  heading_ = curr_msg->z; // in degree, with repsect to x axis in world frame
}


void BotControl::controlPub()
{
  
  double trans_x = 0, trans_heading = 0; // no control cmd
  
  error_pos_prev_ = error_pos_;
  error_heading_prev_ = error_heading_;
  
  double error_x = target_x_prev_ + 0.5 - pos_x_; //toward the center of the cell
  double error_y = target_y_prev_ + 0.5 - pos_y_;
  error_pos_ = sqrt(error_x*error_x + error_y*error_y);
  
  
  // the angle is with spect to the NORTH direction
  double target_heading = atan2(error_y, error_x)* 180/PI; //in degree, with repsect to x axis in world frame
  error_heading_ = (target_heading - heading_) *PI/180; //in rad
  
  while(error_heading_ < -PI){
    error_heading_ += 2*PI;
  }
  while(error_heading_ > PI){
    error_heading_ -= 2*PI;
  }
  
  //implement a PID control here
  I_heading_ += dt*error_heading_;
  I_pos_ += dt*error_pos_;
  
  D_heading_ = (error_heading_ - error_heading_prev_) / dt;
  D_pos_ = (error_pos_ - error_pos_prev_) / dt;
  
  trans_heading = Kp_a * error_heading_ + Ki_a * I_heading_ + Kd_a * D_heading_;
  if(fabs(error_heading_) > 0.15){
    trans_x = 0;
  }else{
    trans_x = Kp_x * error_pos_ + Ki_x * I_pos_ + Kd_x * D_pos_;
    trans_heading = 0;
  }
  //trans_heading = Kp_a * error_heading_ + Ki_a * I_heading;
  
  
  if(trans_x > max_vel) trans_x = max_vel;
  if(trans_x < -max_vel) trans_x = -max_vel;
  if(trans_heading > max_ang) trans_heading = max_ang;
  if(trans_heading < -max_ang) trans_heading = -max_ang;
  
  geometry_msgs::Twist cmd;
  cmd.linear.x = trans_x;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = trans_heading;
  
  control_pub_.publish(cmd);
  ROS_INFO("pos_x_: %f, pos_y_: %f, heading_: %f", pos_x_, pos_y_, heading_);
  ROS_INFO("target_heading:%f, error_pos_:%f, error_heading_:%f", target_heading, error_pos_, error_heading_);
  ROS_INFO("trans_x:%f, trans_ang: %f",trans_x, trans_heading);
}

void BotControl::spin()
{
  ros::Rate loop_rate(1.0/dt);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
}


bool BotControl::loadParam(){

	if(!nh_.getParam("/Kp_a", Kp_a)){
		ROS_ERROR("Kp_a Load Error");
		return false;
	}
	if(!nh_.getParam("/Ki_a", Ki_a)){
		ROS_ERROR("Ki_a Load Error");
		return false;
	}
	if(!nh_.getParam("/Kd_a", Kd_a)){
		ROS_ERROR("Kd_a Load Error");
		return false;
	}
	if(!nh_.getParam("/Kp_x", Kp_x)){
		ROS_ERROR("Kp_x Load Error");
		return false;
	}
	if(!nh_.getParam("/Ki_x", Ki_x)){
		ROS_ERROR("Ki_x Load Error");
		return false;
	}
	if(!nh_.getParam("/Kd_x", Kd_x)){
		ROS_ERROR("Kd_x Load Error");
		return false;
	}

	if(!nh_.getParam("/dt", dt)){
		ROS_ERROR("dt Load Error");
		return false;
	}

	return true;

}


