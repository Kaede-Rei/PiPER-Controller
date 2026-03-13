#ifndef _piper_service_hpp_
#define _piper_service_hpp_

#include <memory>
#include <vector>

#include <ros/ros.h>

#include "piper_controller/arm_controller.hpp"
#include "piper_controller/eef_controller.hpp"
#include "piper_controller/tasks_manager.hpp"
#include "piper_msgs_srvs/piper_cmd.h"
#include "serial_driver/serial_driver.hpp"

namespace piper {

// ! ========================= Typedef / 量 定 义 ========================= ! //



// ! ========================= 接 口 A P I 声 明 ========================= ! //

/**
 * @brief Piper 机械臂服务端类
 */
class Server {
public:
    /**
     * @brief 构造函数
     * @param nh ROS 节点句柄
     * @param plan_group 机械臂规划组名称
     */
    Server(ros::NodeHandle& nh, const std::string& plan_group);

    /**
     * @brief 末端控制命令回调
     */
    bool eefPoseCmdCallback(piper_msgs_srvs::piper_cmd::Request& req,
        piper_msgs_srvs::piper_cmd::Response& res);

    /**
     * @brief 任务组命令回调
     */
    bool taskGroupPlannerCallback(piper_msgs_srvs::piper_cmd::Request& req,
        piper_msgs_srvs::piper_cmd::Response& res);

private:
    ros::ServiceServer _srv_eef_cmd_;
    ros::ServiceServer _srv_task_planner_;
    std::shared_ptr<piper::ArmController> _arm_controller_;
    std::shared_ptr<piper::EndEffector> _eef_controller_;
    std::shared_ptr<piper::TasksManager> _task_planner_;
    std::string _task_group_name_;
    int _next_task_id_;
    STM32Serial _stm32_serialer_;
};

class Client {
public:
    /**
     * @brief 构造函数
     * @param nh ROS 节点句柄
     */
    Client(ros::NodeHandle& nh);

    /**
     * @brief 发送命令到指定服务
     */
    bool sendCmd(ros::ServiceClient& client,
        const piper_msgs_srvs::piper_cmd::Request& req,
        piper_msgs_srvs::piper_cmd::Response& res);

    bool zero(void);
    bool zero(std::string& message);
    bool goalBase(double x, double y, double z, double roll, double pitch, double yaw);
    bool goalBase(double x, double y, double z, double roll, double pitch, double yaw, std::string& message);
    bool goalEef(double x, double y, double z, double roll, double pitch, double yaw);
    bool goalEef(double x, double y, double z, double roll, double pitch, double yaw, std::string& message);
    bool stretch(double length);
    bool stretch(double length, std::string& message);
    bool rotate(double angle);
    bool rotate(double angle, std::string& message);
    bool getPose(double& x, double& y, double& z, double& roll, double& pitch, double& yaw);
    bool getPose(double& x, double& y, double& z, double& roll, double& pitch, double& yaw, std::string& message);
    bool getJoints(std::vector<double>& joints);
    bool getJoints(std::vector<double>& joints, std::string& message);

    bool addTask(geometry_msgs::Pose target_pose, double wait_time, const std::string& action, double param1);
    bool addTask(geometry_msgs::Pose target_pose, double wait_time, const std::string& action, double param1, std::string& message);
    bool clearTasks(void);
    bool clearTasks(std::string& message);
    bool exeAllTasks(void);
    bool exeAllTasks(std::string& message);

private:
    ros::NodeHandle _nh_;
    ros::ServiceClient _client_eef_cmd_;
    ros::ServiceClient _client_task_planner_;
};
}

#endif
