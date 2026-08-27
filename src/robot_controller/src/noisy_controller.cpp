#include "robot_controller/noisy_controller.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.hpp>

using  std::placeholders::_1;

NoisyController::NoisyController(const std::string &name) : Node(name),
    left_wheel_prev_pos_(0.0),
    right_wheel_prev_pos_(0.0),
    x_(0.0),
    y_(0.0),
    theta_(0.0)
    {
    declare_parameter("wheel_radius",0.033);
    declare_parameter("wheel_separation",0.17);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();
    
    prev_time_ = this->get_clock()->now();

    RCLCPP_INFO_STREAM(this->get_logger(),"wheel radius : " << wheel_radius_ << " and wheel separation : " << wheel_separation_);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,std::bind(&NoisyController::jointCallback,this,_1));
    
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("robot_controller/odom",10);

    
    // initializing odom_msg
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0;
    odom_msg_.pose.pose.orientation.y = 0;
    odom_msg_.pose.pose.orientation.z = 0;
    odom_msg_.pose.pose.orientation.w = 1;
        
}


void NoisyController::jointCallback(const sensor_msgs::msg::JointState &msg){

    double dp_left = msg.position.at(1) - left_wheel_prev_pos_;
    double dp_right = msg.position.at(0) - right_wheel_prev_pos_;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    left_wheel_prev_pos_ = msg.position.at(1);
    right_wheel_prev_pos_ = msg.position.at(0);
    prev_time_ = msg_time;

    double fi_left = dp_left/dt.seconds();
    double fi_right = dp_right/dt.seconds();

    // linear and angular velocity

    double linear  = (wheel_radius_ * fi_right + wheel_radius_ * fi_left)/2;
    double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left)/ wheel_separation_;

    RCLCPP_INFO_STREAM(get_logger(),"linear : " << linear << " angular : " << angular);
    
    // position and orientation
    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left)/2;
    double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left)/wheel_separation_;

    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    // publishing odom_msg
    tf2::Quaternion q;
    q.setRPY(0,0,theta_);

    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_msg_.header.stamp = this->get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;

    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;

    odom_pub_->publish(odom_msg_);
    
    RCLCPP_INFO_STREAM(get_logger(),"x : " << x_ << " y : " << y_ << " theta : " <<theta_);
}
int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<NoisyController> ("noisy_controller"));
    rclcpp::shutdown();

    return 0;
}