#ifndef _arm_interface_hpp_
#define _arm_interface_hpp_

#include <actionlib/server/simple_action_server.h>

#include "piper_controller/arm_controller.hpp"
#include "piper_commander/cmd_dispatcher.hpp"
#include "piper_msgs/MoveArmAction.h"
#include "piper_msgs/SimpleMoveArmAction.h"
#include "piper_msgs/ConfigArm.h"
#include "piper_msgs/QueryArm.h"

namespace piper {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class ArmMoveAction {
public:
    using MoveArmAS = actionlib::SimpleActionServer<piper_msgs::MoveArmAction>;
    ArmMoveAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name);
    ~ArmMoveAction() = default;

    ArmMoveAction(const ArmMoveAction&) = delete;
    ArmMoveAction& operator=(const ArmMoveAction&) = delete;
    ArmMoveAction(ArmMoveAction&&) = delete;
    ArmMoveAction& operator=(ArmMoveAction&&) = delete;

private:
    void on_goal();
    void on_preempt();
    bool convert_goal_to_request(const piper_msgs::MoveArmGoal& goal, ArmCmdRequest& req);

private:
    std::unique_ptr<MoveArmAS> _as_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<ArmCmdDispatcher> _dispatcher_;
};

class SimpleArmMoveAction {
public:
    using MoveArmAS = actionlib::SimpleActionServer<piper_msgs::SimpleMoveArmAction>;
    SimpleArmMoveAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name);
    ~SimpleArmMoveAction() = default;

    SimpleArmMoveAction(const SimpleArmMoveAction&) = delete;
    SimpleArmMoveAction& operator=(const SimpleArmMoveAction&) = delete;
    SimpleArmMoveAction(SimpleArmMoveAction&&) = delete;
    SimpleArmMoveAction& operator=(SimpleArmMoveAction&&) = delete;

private:
    void on_goal();
    void on_preempt();
    bool convert_goal_to_request(const piper_msgs::SimpleMoveArmGoal& goal, ArmCmdRequest& req);

private:
    std::unique_ptr<MoveArmAS> _as_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<ArmCmdDispatcher> _dispatcher_;
};

class ArmConfigService {
public:
    ArmConfigService(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string service_name);
    ~ArmConfigService() = default;

    ArmConfigService(const ArmConfigService&) = delete;
    ArmConfigService& operator=(const ArmConfigService&) = delete;
    ArmConfigService(ArmConfigService&&) = delete;
    ArmConfigService& operator=(ArmConfigService&&) = delete;

private:
    bool on_request(piper_msgs::ConfigArm::Request& req, piper_msgs::ConfigArm::Response& res);
    bool convert_srvreq_to_armreq(const piper_msgs::ConfigArm::Request& srv_req, ArmCmdRequest& arm_req);

private:
    std::unique_ptr<ros::ServiceServer> _srv_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<ArmCmdDispatcher> _dispatcher_;
};

class ArmQueryService {
public:
    ArmQueryService(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string service_name);
    ~ArmQueryService() = default;

    ArmQueryService(const ArmQueryService&) = delete;
    ArmQueryService& operator=(const ArmQueryService&) = delete;
    ArmQueryService(ArmQueryService&&) = delete;
    ArmQueryService& operator=(ArmQueryService&&) = delete;

private:
    bool on_request(piper_msgs::QueryArm::Request& req, piper_msgs::QueryArm::Response& res);
    bool convert_srvreq_to_armreq(const piper_msgs::QueryArm::Request& srv_req, ArmCmdRequest& arm_req);

private:
    std::unique_ptr<ros::ServiceServer> _srv_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<ArmCmdDispatcher> _dispatcher_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}

#endif
