#ifndef _arm_interface_hpp_
#define _arm_interface_hpp_

#include <actionlib/server/simple_action_server.h>

#include "piper_controller/arm_controller.hpp"
#include "piper_commander/cmd_dispatcher.hpp"
#include "piper_msgs/MoveArmAction.h"
#include "piper_msgs/SimpleMoveArmAction.h"

namespace piper {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class ArmAction {
public:
    using MoveArmAS = actionlib::SimpleActionServer<piper_msgs::MoveArmAction>;
    ArmAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name = "move_arm");
    ~ArmAction() = default;

    ArmAction(const ArmAction&) = delete;
    ArmAction& operator=(const ArmAction&) = delete;
    ArmAction(ArmAction&&) = delete;
    ArmAction& operator=(ArmAction&&) = delete;

private:
    void on_goal();
    void on_preempt();
    bool convert_goal_to_request(const piper_msgs::MoveArmGoal& goal, ArmCmdRequest& req);

private:
    std::unique_ptr<MoveArmAS> _as_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<ArmCmdDispatcher> _dispatcher_;
};

class SimpleArmAction {
public:
    using MoveArmAS = actionlib::SimpleActionServer<piper_msgs::SimpleMoveArmAction>;
    SimpleArmAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name = "simple_move_arm");
    ~SimpleArmAction() = default;

    SimpleArmAction(const SimpleArmAction&) = delete;
    SimpleArmAction& operator=(const SimpleArmAction&) = delete;
    SimpleArmAction(SimpleArmAction&&) = delete;
    SimpleArmAction& operator=(SimpleArmAction&&) = delete;

private:
    void on_goal();
    void on_preempt();
    bool convert_goal_to_request(const piper_msgs::SimpleMoveArmGoal& goal, ArmCmdRequest& req);

private:
    std::unique_ptr<MoveArmAS> _as_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<ArmCmdDispatcher> _dispatcher_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}

#endif
