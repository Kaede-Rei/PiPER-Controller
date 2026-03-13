#include "piper_service/piper_service.hpp"

#include <clocale>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "piper_controller/eef_controller.hpp"

// ! ========================= 宏 定 义 ========================= ! //

#define STM32_SERIAL_PORT "/dev/ttyACM0"

namespace piper {

// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //

static bool CheckLifterIsNeed(double z, STM32Serial& serialer);

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief 服务端构造函数：初始化控制器、任务管理器、串口与服务
 * @param nh ROS 节点句柄
 * @param plan_group 机械臂规划组名称
 */
Server::Server(ros::NodeHandle& nh, const std::string& plan_group)
    : _task_group_name_("default"),
    _next_task_id_(1),
    _stm32_serialer_(nh, STM32_SERIAL_PORT, 115200) {

    _arm_controller_ = std::make_shared<piper::ArmController>(plan_group);
    _eef_controller_ = std::make_shared<piper::TwoFingerGripper>("gripper");
    _task_planner_ = std::make_shared<piper::TasksManager>(_arm_controller_, _eef_controller_);

    _task_planner_->create_task_group(_task_group_name_, piper::SortType::DIST);
    _task_planner_->set_dist_sort_weight_orient(_task_group_name_, 0.3f);

    if(!_stm32_serialer_.connect()) {
        ROS_ERROR("STM32 串口连接失败：%s", STM32_SERIAL_PORT);
    }

    _srv_eef_cmd_ = nh.advertiseService("/piper_server/eef_cmd", &Server::eefPoseCmdCallback, this);
    _srv_task_planner_ = nh.advertiseService("/piper_server/task_planner", &Server::taskGroupPlannerCallback, this);

    _arm_controller_->reset_to_zero();
}

/**
 * @brief 末端控制服务回调
 * @param req 请求
 * @param res 响应
 * @return 回调执行状态
 */
bool Server::eefPoseCmdCallback(piper_msgs_srvs::piper_cmd::Request& req,
    piper_msgs_srvs::piper_cmd::Response& res) {
    if(req.command == "zero") {
        _arm_controller_->reset_to_zero();

        res.success = true;
        res.message = "回到零点成功";
    }
    else if(req.command == "goal_base" || req.command == "goal_eef") {
        geometry_msgs::PoseStamped target_pose;
        tf2::Quaternion qtn;

        qtn.setRPY(req.roll, req.pitch, req.yaw);
        qtn.normalize();

        target_pose.pose.position.x = req.x;
        target_pose.pose.position.y = req.y;
        target_pose.pose.position.z = req.z;
        target_pose.pose.orientation.x = qtn.x();
        target_pose.pose.orientation.y = qtn.y();
        target_pose.pose.orientation.z = qtn.z();
        target_pose.pose.orientation.w = qtn.w();

        bool lifter_success = true;
        piper::ErrorCode arm_result = piper::ErrorCode::SUCCESS;

        if(req.command == "goal_base") {
            lifter_success = piper::CheckLifterIsNeed(target_pose.pose.position.z, _stm32_serialer_);
            if(lifter_success) {
                arm_result = _arm_controller_->set_target(target_pose.pose);
                if(arm_result == piper::ErrorCode::SUCCESS) {
                    arm_result = _arm_controller_->plan_and_execute();
                }
            }
        }
        else {
            geometry_msgs::Pose target_pose_base;
            arm_result = _arm_controller_->end_to_base_tf(target_pose.pose, target_pose_base);
            if(arm_result == piper::ErrorCode::SUCCESS) {
                lifter_success = piper::CheckLifterIsNeed(target_pose_base.position.z, _stm32_serialer_);
            }
            if(lifter_success && arm_result == piper::ErrorCode::SUCCESS) {
                arm_result = _arm_controller_->set_target_in_eef_frame(target_pose.pose);
                if(arm_result == piper::ErrorCode::SUCCESS) {
                    arm_result = _arm_controller_->plan_and_execute();
                }
            }
        }

        res.success = lifter_success && arm_result == piper::ErrorCode::SUCCESS;
        if(!lifter_success) res.message = "升降台升降失败";
        else if(res.success) res.message = "设置目标位姿成功";
        else res.message = "设置目标位姿失败: " + err_to_string(arm_result);
    }
    else if(req.command == "stretch") {
        piper::ErrorCode err = _arm_controller_->telescopic_end(std::stod(req.param1));
        if(err == piper::ErrorCode::SUCCESS) {
            err = _arm_controller_->plan_and_execute();
        }

        res.success = (err == piper::ErrorCode::SUCCESS);
        if(res.success) res.message = "末端伸缩成功";
        else res.message = "末端伸缩失败: " + err_to_string(err);
    }
    else if(req.command == "rotate") {
        piper::ErrorCode err = _arm_controller_->rotate_end(std::stod(req.param1));
        if(err == piper::ErrorCode::SUCCESS) {
            err = _arm_controller_->plan_and_execute();
        }

        res.success = (err == piper::ErrorCode::SUCCESS);
        if(res.success) res.message = "末端旋转成功";
        else res.message = "末端旋转失败: " + err_to_string(err);
    }
    else if(req.command == "get_pose") {
        geometry_msgs::Pose cur_pose = _arm_controller_->get_current_pose();
        tf2::Quaternion qtn(cur_pose.orientation.x,
            cur_pose.orientation.y,
            cur_pose.orientation.z,
            cur_pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(qtn).getRPY(roll, pitch, yaw);

        res.cur_x = cur_pose.position.x; res.cur_y = cur_pose.position.y; res.cur_z = cur_pose.position.z;
        res.cur_roll = roll; res.cur_pitch = pitch; res.cur_yaw = yaw;
        res.message = "获取末端位姿成功";
        res.success = true;
    }
    else if(req.command == "get_joints") {
        res.cur_joint = _arm_controller_->get_current_joints();

        res.message = "获取各关节角度成功";
        res.success = true;
    }
    else if(req.command == "open") {
        std::string data = "$GRIPPER:OPEN#\n";
        _stm32_serialer_.sendData(data);
        res.success = true;
        res.message = "夹爪打开命令已发送";
    }
    else if(req.command == "close") {
        std::string data = "$GRIPPER:CLOSE#\n";
        _stm32_serialer_.sendData(data);
        res.success = true;
        res.message = "夹爪闭合命令已发送";
    }
    else if(req.command == "angle") {
        int angle_cmd = static_cast<int>(2000.0 * req.angle_eef / 270.0 + 500);
        std::ostringstream oss;
        oss << std::setw(4) << std::setfill('0') << angle_cmd;
        std::string data = "$GRIPPER:POS:" + oss.str() + "#\n";
        _stm32_serialer_.sendData(data);
        res.success = true;
        res.message = "夹爪角度命令已发送";
    }
    else {
        res.success = false;
        res.message = "未知命令";
    }

    return true;
}

/**
 * @brief 任务组服务回调
 * @param req 请求
 * @param res 响应
 * @return 回调执行状态
 */
bool Server::taskGroupPlannerCallback(piper_msgs_srvs::piper_cmd::Request& req,
    piper_msgs_srvs::piper_cmd::Response& res) {
    if(req.command == "add_task") {
        geometry_msgs::Pose target_pose;
        tf2::Quaternion qtn;

        qtn.setRPY(req.roll, req.pitch, req.yaw);
        qtn.normalize();

        target_pose.position.x = req.x;
        target_pose.position.y = req.y;
        target_pose.position.z = req.z;
        target_pose.orientation.x = qtn.x();
        target_pose.orientation.y = qtn.y();
        target_pose.orientation.z = qtn.z();
        target_pose.orientation.w = qtn.w();

        piper::TaskType task_type = piper::TaskType::NONE;
        if(req.param2 == "PICK") task_type = piper::TaskType::PICK;
        else if(!req.param2.empty() && req.param2 != "NONE") {
            res.success = false;
            res.message = "add_task 仅支持 NONE 或 PICK";
            return true;
        }

        int task_id = _next_task_id_++;
        ErrorCode err = _task_planner_->add_task(_task_group_name_, task_id, task_type, "piper_task");
        if(err == ErrorCode::SUCCESS) {
            err = _task_planner_->set_task_target(_task_group_name_, task_id, target_pose);
        }

        res.success = (err == ErrorCode::SUCCESS);
        if(res.success) res.message = "添加任务成功";
        else res.message = "添加任务失败: " + err_to_string(err);
    }
    else if(req.command == "clear_tasks") {
        ErrorCode err = _task_planner_->clear_task_group(_task_group_name_);
        _next_task_id_ = 1;

        res.success = (err == ErrorCode::SUCCESS);
        if(res.success) res.message = "清除任务成功";
        else res.message = "清除任务失败: " + err_to_string(err);
    }
    else if(req.command == "exe_all_tasks") {
        ErrorCode err = _task_planner_->execute_task_group(_task_group_name_);

        res.success = (err == ErrorCode::SUCCESS);
        if(res.success) res.message = "执行所有任务完成";
        else res.message = "执行任务失败: " + err_to_string(err);
    }
    else {
        res.success = false;
        res.message = "未知任务命令";
    }

    return true;
}

/**
 * @brief 客户端构造函数：等待并连接服务
 * @param nh ROS 节点句柄
 */
Client::Client(ros::NodeHandle& nh) : _nh_(nh) {
    ROS_INFO("等待机械臂服务...");
    ros::service::waitForService("piper_server/eef_cmd");
    ros::service::waitForService("piper_server/task_planner");

    _client_eef_cmd_ = _nh_.serviceClient<piper_msgs_srvs::piper_cmd>("piper_server/eef_cmd");
    _client_task_planner_ = _nh_.serviceClient<piper_msgs_srvs::piper_cmd>("piper_server/task_planner");

    ROS_INFO("机械臂服务已连接");
}

/**
 * @brief 发送服务请求
 * @param client 服务客户端
 * @param req 请求
 * @param res 响应
 * @return 调用是否成功
 */
bool Client::sendCmd(ros::ServiceClient& client,
    const piper_msgs_srvs::piper_cmd::Request& req,
    piper_msgs_srvs::piper_cmd::Response& res) {
    piper_msgs_srvs::piper_cmd srv;
    srv.request = req;

    if(client.call(srv)) {
        res = srv.response;

        if(!res.success) {
            ROS_ERROR("机械臂服务调用失败：%s", res.message.c_str());
        }

        return true;
    }
    else {
        ROS_ERROR("无法请求机械臂服务");
        return false;
    }
}

bool Client::zero(void) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "zero";

    return sendCmd(_client_eef_cmd_, req, res);
}

bool Client::zero(std::string& message) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "zero";

    bool success = sendCmd(_client_eef_cmd_, req, res);
    message = res.message;

    return success && res.success;
}

bool Client::goalBase(double x, double y, double z, double roll, double pitch, double yaw) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "goal_base";
    req.x = x; req.y = y; req.z = z;
    req.roll = roll; req.pitch = pitch; req.yaw = yaw;

    return sendCmd(_client_eef_cmd_, req, res);
}

bool Client::goalBase(double x, double y, double z, double roll, double pitch, double yaw, std::string& message) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "goal_base";
    req.x = x; req.y = y; req.z = z;
    req.roll = roll; req.pitch = pitch; req.yaw = yaw;

    bool success = sendCmd(_client_eef_cmd_, req, res);
    message = res.message;

    return success && res.success;
}

bool Client::goalEef(double x, double y, double z, double roll, double pitch, double yaw) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "goal_eef";
    req.x = x; req.y = y; req.z = z;
    req.roll = roll; req.pitch = pitch; req.yaw = yaw;

    return sendCmd(_client_eef_cmd_, req, res);
}

bool Client::goalEef(double x, double y, double z, double roll, double pitch, double yaw, std::string& message) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "goal_eef";
    req.x = x; req.y = y; req.z = z;
    req.roll = roll; req.pitch = pitch; req.yaw = yaw;

    bool success = sendCmd(_client_eef_cmd_, req, res);
    message = res.message;

    return success && res.success;
}

bool Client::stretch(double length) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "stretch";
    req.param1 = std::to_string(length);

    return sendCmd(_client_eef_cmd_, req, res);
}

bool Client::stretch(double length, std::string& message) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "stretch";
    req.param1 = std::to_string(length);

    bool success = sendCmd(_client_eef_cmd_, req, res);
    message = res.message;

    return success && res.success;
}

bool Client::rotate(double angle) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "rotate";
    req.param1 = std::to_string(angle);

    return sendCmd(_client_eef_cmd_, req, res);
}

bool Client::rotate(double angle, std::string& message) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "rotate";
    req.param1 = std::to_string(angle);

    bool success = sendCmd(_client_eef_cmd_, req, res);
    message = res.message;

    return success && res.success;
}

bool Client::getPose(double& x, double& y, double& z, double& roll, double& pitch, double& yaw) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "get_pose";

    if(sendCmd(_client_eef_cmd_, req, res)) {
        x = res.cur_x; y = res.cur_y; z = res.cur_z;
        roll = res.cur_roll; pitch = res.cur_pitch; yaw = res.cur_yaw;

        return true;
    }
    else return false;
}

bool Client::getPose(double& x, double& y, double& z, double& roll, double& pitch, double& yaw, std::string& message) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "get_pose";

    bool success = sendCmd(_client_eef_cmd_, req, res);
    message = res.message;

    if(success && res.success) {
        x = res.cur_x; y = res.cur_y; z = res.cur_z;
        roll = res.cur_roll; pitch = res.cur_pitch; yaw = res.cur_yaw;

        return true;
    }
    else return false;
}

bool Client::getJoints(std::vector<double>& joints) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "get_joints";

    if(sendCmd(_client_eef_cmd_, req, res)) {
        joints = res.cur_joint;
        return true;
    }
    else return false;
}

bool Client::getJoints(std::vector<double>& joints, std::string& message) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "get_joints";

    bool success = sendCmd(_client_eef_cmd_, req, res);
    message = res.message;

    if(success && res.success) {
        joints = res.cur_joint;
        return true;
    }
    else return false;
}

bool Client::addTask(geometry_msgs::Pose target_pose, double wait_time, const std::string& action, double param1) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "add_task";
    req.x = target_pose.position.x;
    req.y = target_pose.position.y;
    req.z = target_pose.position.z;

    tf2::Quaternion qtn(target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(qtn).getRPY(roll, pitch, yaw);

    req.roll = roll;
    req.pitch = pitch;
    req.yaw = yaw;

    req.param1 = std::to_string(wait_time);
    req.param2 = action;
    req.param3 = std::to_string(param1);

    return sendCmd(_client_task_planner_, req, res);
}

bool Client::addTask(geometry_msgs::Pose target_pose, double wait_time, const std::string& action, double param1, std::string& message) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "add_task";
    req.x = target_pose.position.x;
    req.y = target_pose.position.y;
    req.z = target_pose.position.z;

    tf2::Quaternion qtn(target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(qtn).getRPY(roll, pitch, yaw);

    req.roll = roll;
    req.pitch = pitch;
    req.yaw = yaw;

    req.param1 = std::to_string(wait_time);
    req.param2 = action;
    req.param3 = std::to_string(param1);

    bool success = sendCmd(_client_task_planner_, req, res);
    message = res.message;

    return success && res.success;
}

bool Client::clearTasks(void) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "clear_tasks";

    return sendCmd(_client_task_planner_, req, res);
}

bool Client::clearTasks(std::string& message) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "clear_tasks";

    bool success = sendCmd(_client_task_planner_, req, res);
    message = res.message;

    return success && res.success;
}

bool Client::exeAllTasks(void) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "exe_all_tasks";

    return sendCmd(_client_task_planner_, req, res);
}

bool Client::exeAllTasks(std::string& message) {
    piper_msgs_srvs::piper_cmd::Request req;
    piper_msgs_srvs::piper_cmd::Response res;

    req.command = "exe_all_tasks";

    bool success = sendCmd(_client_task_planner_, req, res);
    message = res.message;

    return success && res.success;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 检查是否需要升降台动作
 * @param z 目标 Z 高度
 * @param serialer 串口对象
 * @return 是否处理成功
 */
static bool CheckLifterIsNeed(double z, STM32Serial& serialer) {
    if(z > 0.65 || z < -0.15) {
        double offset = 0.0f;

        if(z > 0.65) offset = std::round((z - 0.75) * 100.0) / 100.0;
        else offset = std::round((z + 0.25) * 100.0) / 100.0;

        std::string data = "$LIFTER:" + std::to_string(offset) + "#";
        serialer.sendData(data);

        std::string res_data = serialer.rcvdData(1000);
        if(res_data != "$LIFTER:START#") return false;

        res_data = serialer.rcvdData(0);
        if(res_data != "$LIFTER:OK#") return false;
    }

    return true;
}

} /* namespace piper */

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "piper_server");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int num_threads;
    pnh.param<int>("num_threads", num_threads, 12);

    ROS_INFO("====================================");
    ROS_INFO("Piper机械臂服务器启动中...");
    ROS_INFO("使用线程数: %d", num_threads);
    ROS_INFO("====================================");

    ROS_INFO("等待机器人状态发布...");
    try {
        ros::topic::waitForMessage<sensor_msgs::JointState>(
            "/joint_states",
            ros::Duration(10.0)
        );
        ROS_INFO("机器人状态已就绪");
    }
    catch(const std::exception& e) {
        ROS_WARN("等待机器人状态超时: %s", e.what());
        ROS_WARN("将使用 fake execution 模式");
    }

    ros::AsyncSpinner spinner(num_threads);
    spinner.start();

    try {
        piper::Server srv(nh, "arm");

        ROS_INFO("====================================");
        ROS_INFO("Piper机械臂服务器已启动");
        ROS_INFO("可用的服务有：");
        ROS_INFO("  1. /piper_server/eef_cmd");
        ROS_INFO("     └─ 末端位姿控制服务");
        ROS_INFO("  2. /piper_server/task_planner");
        ROS_INFO("     └─ 任务组规划服务");
        ROS_INFO("====================================");

        ros::waitForShutdown();
    }
    catch(const std::exception& e) {
        ROS_ERROR("服务器初始化失败: %s", e.what());
        return 1;
    }

    ROS_INFO("Piper机械臂服务器已关闭");
    return 0;
}
