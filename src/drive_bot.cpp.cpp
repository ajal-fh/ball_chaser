#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ball_chaser/DriveToTarget.h"



class Drivebot{
public:
Drivebot(ros::NodeHandle n)
:nh_{n},private_nh_{"~"} {
    cmd_pub_ = private_nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    cmd_server_ = private_nh_.advertiseService("/ball_chaser/command_robot",&Drivebot::handle_cmd_service, this);
    ROS_INFO("Ready to send command velocities");    
}

bool handle_cmd_service(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res){
    geometry_msgs::Twist msg;
    msg.linear.x = req.linear_x;
    msg.angular.z = req.angular_z;
    cmd_pub_.publish(msg);
    res.msg_feedback = "velocity set to linear: " + std::to_string(req.linear_x)+" angular: "+std::to_string(req.angular_z);
    return true;
}
~Drivebot(){};
private:
ros::NodeHandle nh_;
ros::NodeHandle private_nh_;
ros::Publisher cmd_pub_;
ros::ServiceServer cmd_server_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;
    Drivebot drive(n);
    ros::spin();
    return 0;
}



