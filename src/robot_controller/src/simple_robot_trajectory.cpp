#include "robot_controller/simple_robot_trajectory.hpp"
using std::placeholders::_1;

RobotTrajectory::RobotTrajectory(const std::string &name) : Node(name){
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/robot_controller/odom",10,std::bind(&RobotTrajectory::odomCallback,this,_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/robot_controller/trajectory",10); 

}
void RobotTrajectory::odomCallback(const nav_msgs::msg::Odometry &msg){
    path_.header.frame_id = "odom";
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = get_clock()->now();
    pose_stamped.pose = msg.pose.pose;

    path_.poses.push_back(pose_stamped);

    path_pub_->publish(path_);
    RCLCPP_INFO_STREAM(get_logger()," Got odom msg and converted to path");
}

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RobotTrajectory>("robot_trajectory");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}