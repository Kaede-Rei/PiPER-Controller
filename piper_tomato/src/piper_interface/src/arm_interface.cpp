#include "piper_interface/arm_interface.hpp"

namespace piper {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

ArmAction::ArmAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name)
    : _arm_(std::move(arm)), _dispatcher_(std::move(dispatcher)) {
    _as_ = std::make_unique<MoveArmAS>(nh, action_name, false);
    _as_->registerGoalCallback(boost::bind(&ArmAction::on_goal, this));
    _as_->registerPreemptCallback(boost::bind(&ArmAction::on_preempt, this));
    _as_->start();
}

SimpleArmAction::SimpleArmAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name)
    : _arm_(std::move(arm)), _dispatcher_(std::move(dispatcher)) {
    _as_ = std::make_unique<MoveArmAS>(nh, action_name, false);
    _as_->registerGoalCallback(boost::bind(&SimpleArmAction::on_goal, this));
    _as_->registerPreemptCallback(boost::bind(&SimpleArmAction::on_preempt, this));
    _as_->start();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

bool ArmAction::convert_goal_to_request(const piper_msgs::MoveArmGoal& goal, ArmCmdRequest& req) {
    req.type = static_cast<ArmCmdType>(goal.command_type);
    if(!(ArmCmdType::MIN < req.type && req.type < ArmCmdType::MAX)) {
        ROS_WARN("接收到无效的命令类型: %d", goal.command_type);
        return false;
    }
    req.joint_names = goal.joint_names;
    req.joints = goal.joints;

    if(goal.target_type == goal.TARGET_POSE) req.target = goal.pose;
    else if(goal.target_type == goal.TARGET_POINT) req.target = goal.point;
    else if(goal.target_type == goal.TARGET_QUATERNION) req.target = goal.quaternion;
    else {
        ROS_WARN("接收到无效的目标类型: %d", goal.target_type);
        return false;
    }

    req.values = goal.values;
    req.waypoints = goal.waypoints;

    return true;
}

bool SimpleArmAction::convert_goal_to_request(const piper_msgs::SimpleMoveArmGoal& goal, ArmCmdRequest& req) {
    req.type = static_cast<ArmCmdType>(goal.command_type);
    if(!(ArmCmdType::MIN < req.type && req.type < ArmCmdType::MAX)) {
        ROS_WARN("接收到无效的命令类型: %d", goal.command_type);
        return false;
    }
    req.joint_names = goal.joint_names;
    req.joints = goal.joints;

    if(goal.target_type == goal.TARGET_POSE) {
        geometry_msgs::Pose pose;
        pose.position.x = goal.x[0];
        pose.position.y = goal.y[0];
        pose.position.z = goal.z[0];
        tf2::Quaternion quat;
        quat.setRPY(goal.roll[0], goal.pitch[0], goal.yaw[0]);
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        req.target = pose;
    }
    else if(goal.target_type == goal.TARGET_POINT) {
        geometry_msgs::Point point;
        point.x = goal.x[0];
        point.y = goal.y[0];
        point.z = goal.z[0];
        req.target = point;
    }
    else if(goal.target_type == goal.TARGET_ORIENTATION) {
        tf2::Quaternion quat;
        quat.setRPY(goal.roll[0], goal.pitch[0], goal.yaw[0]);
        geometry_msgs::Quaternion orientation;
        orientation.x = quat.x();
        orientation.y = quat.y();
        orientation.z = quat.z();
        orientation.w = quat.w();
        req.target = orientation;
    }
    else {
        ROS_WARN("接收到无效的目标类型: %d", goal.target_type);
        return false;
    }

    req.values = goal.values;
    for(size_t i = 1; i < goal.x.size(); ++i) {
        geometry_msgs::Pose waypoint;
        waypoint.position.x = goal.x[i];
        waypoint.position.y = goal.y[i];
        waypoint.position.z = goal.z[i];
        tf2::Quaternion quat;
        quat.setRPY(goal.roll[i], goal.pitch[i], goal.yaw[i]);
        waypoint.orientation.x = quat.x();
        waypoint.orientation.y = quat.y();
        waypoint.orientation.z = quat.z();
        waypoint.orientation.w = quat.w();
        req.waypoints.push_back(waypoint);
    }

    return true;
}

}
