#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>



using namespace std_msgs;

enum {GOAL_NOT_REACHED = 4, GOAL_REACHED = 5};

class Controller{
private:
    ros::Publisher pub_control_speed;
    ros::Subscriber sub_target;
    ros::Subscriber sub_position;

    double trans_x, trans_heading, I_heading, I_pos, dt, D_heading, D_pos;
    double pos_x, pos_y, heading, target_x, target_y, target_heading;
    double error_x,error_y, error_heading, error_heading_prev, error_pos, error_pos_prev;
    double PI, Kp_fwd, Kp_ang, Ki_ang, Ki_fwd, Kd_fwd;
    int goal_reached;

public:
    Controller(ros::NodeHandle nh);

    //callbacks
    void get_target(const geometry_msgs::Point& targetMsg);
    void get_position(const geometry_msgs::Point& posMsg);

    //Functions
    void calculate_speed();
    void spin();
};

//Constructor
Controller::Controller(ros::NodeHandle nh) {

    //Initialize
    pos_x = 0;
    pos_y = 0;
    heading = 0;
    target_x = 0;
    target_y = 0;
    target_heading = 0;
    error_x = 0;
    error_y = 0;
    error_heading = 0;
    error_pos = 0;
    error_pos_prev = 0;
    PI = 3.14159265359;
    I_heading = 0;
    I_pos = 0;
    dt = 0.1; //10Hz, which means that the control loop runs 10 times a second

    // Controller Value
    Kp_ang = 0.30;                        //Angular Proportional Gain
    Ki_ang = 0.020;                       //Angular Integral Gain
    Kp_fwd = 0.40;                        //Translational Proportional Gain
    Ki_fwd = 0.025 * 100;                 //Translational Integral Gain
    Kd_fwd = 0.09;                        //Translational Derivative Gain

    goal_reached = GOAL_NOT_REACHED;

    // Callback updates set point position in x and y
    sub_position = nh.subscribe("pathplanner/x_y_yaw",1,&Controller::get_position,this);
    sub_target = nh.subscribe("/pathplanner/target",1,&Controller::get_target, this);
    pub_control_speed = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
}

//Subscribes to current position
void Controller::get_position(const geometry_msgs::Point& posMsg) {

    pos_x = posMsg.x;
    pos_y = posMsg.y;
    heading = posMsg.z;

}

//This function will subscribe to target messages from the pathplanner node.
void Controller::get_target(const geometry_msgs::Point& targetMsg) {

    //Position from pathplanner node
    target_x = targetMsg.x;
    target_y = targetMsg.y;
    goal_reached = targetMsg.z;

    // Move the robot to its target position
    calculate_speed();
}

void Controller::calculate_speed(){
    geometry_msgs::Twist base_cmd;

    target_heading = atan2(error_x, error_y);

    //Calculate target heading
    error_x = (target_x - pos_x);
    error_y = (target_y - pos_y);
    error_pos_prev = error_pos;
    error_pos = sqrt(error_x * error_x + error_y * error_y);
    error_heading_prev = error_heading;
    error_heading = target_heading - heading;


    // Heading is defined as [-pi, pi], so the error can not be less than -pi or greater than pi:
    if (error_heading < -PI)
    {
        error_heading += 2*PI;
    }
    if (error_heading >= PI)
    {
        error_heading -= 2*PI;
    }

    //Integral term in PID controller
    I_heading += dt * error_heading;
    I_pos += dt * error_pos;

    //Derivative term in PID Controller
    D_pos = error_pos_prev - error_pos;
    D_heading = error_pos_prev - error_heading;

    // If error in position is large, the proportional term will do the job
    if (error_pos > 0.4) {
        I_heading = 0;
        I_pos = 0;
    }


    if (fabs(error_heading) > 0.05) {

        if (fabs(error_heading) > 0.12) {
            //heading
            trans_heading = -Kp_ang * error_heading - Ki_ang * I_heading;
            trans_heading *= pow(3.0, fabs(trans_heading) + 1.0);

            //velocity
            trans_x = Kp_fwd * error_pos + Ki_fwd * I_pos + Kd_fwd * D_pos;
            trans_x *= (1.0 / pow(8.0, fabs(trans_heading) + 1.0));

        }
        else{
            // Heading
            trans_heading = -Kp_ang * error_heading - Ki_ang * I_heading;
            trans_heading *= pow(3.0, fabs(trans_heading) + 1.0);

            // Velocity
            trans_x = Kp_fwd * error_pos + Ki_fwd * I_pos + Kd_fwd * D_pos;
            trans_x *= (1.0 / pow(1.5, fabs(trans_heading) + 1.0));
        }
    }
    else{
        trans_heading = -Kp_ang * error_heading - Ki_ang * I_heading;
        trans_x = Kp_fwd * error_pos + Ki_fwd * I_pos + Kd_fwd * D_pos;
    }

    // Goal not reached = 4,
    if (goal_reached > (GOAL_NOT_REACHED + 0.5)){ // +0.5 to avoid numerical error
        base_cmd.linear.x = 0;
        base_cmd.angular.z = 0;
    }
    else{
        base_cmd.linear.x = trans_x;
        base_cmd.angular.z = trans_heading;
    }


    // Check if commanded velocity exceeds limits(Deadband)
    double max_trans_vel = 0.7; // Max translational velocity [m/s]
    double max_rot_vel = 180.0 * (3.14159265 / 180.0); // Max rotational velocity [rad/s]
    if (base_cmd.linear.x > max_trans_vel){
        base_cmd.linear.x = max_trans_vel;
    }
    if (base_cmd.linear.x < -max_trans_vel){
        base_cmd.linear.x = -max_trans_vel;
    }
    if (base_cmd.angular.z > max_rot_vel){
        base_cmd.angular.z = max_rot_vel;
    }
    if (base_cmd.angular.z < -max_rot_vel){
        base_cmd.angular.z = -max_rot_vel;
    }


    pub_control_speed.publish(base_cmd);
}



void Controller::spin(){
    ros::Rate loop_rate(1.0 / dt);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    Controller *control;
    Controller temp_control(nh);
    control = &temp_control;
    control->spin();

    return 0;
}
