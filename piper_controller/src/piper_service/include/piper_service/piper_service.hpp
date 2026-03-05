#ifndef _piper_service_hpp_
#define _piper_service_hpp_

#include <ros/ros.h>

#include "piper_controller/eef_cmd.hpp"
#include "piper_msgs_srvs/piper_cmd.h"
#include "serial_driver/serial_driver.hpp"

namespace piper {

    // ! ========================= Typedef / 量 定 义 ========================= ! //



    // ! ========================= 接 口 A P I 声 明 ========================= ! //

    class Server {
    public:
        Server(ros::NodeHandle& nh, const std::string& plan_group);
        bool eefPoseCmdCallback(piper_msgs_srvs::piper_cmd::Request& req,
            piper_msgs_srvs::piper_cmd::Response& res);
        bool taskGroupPlannerCallback(piper_msgs_srvs::piper_cmd::Request& req,
            piper_msgs_srvs::piper_cmd::Response& res);

    private:
        ros::ServiceServer _srv_eef_cmd_;
        ros::ServiceServer _srv_task_planner_;
        piper::EefPoseCmd _eef_controller_;
        piper::TaskGroupPlanner _task_planner_;
        STM32Serial _stm32_serialer_;
    };

    class Client {
    public:
        Client(ros::NodeHandle& nh);
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
