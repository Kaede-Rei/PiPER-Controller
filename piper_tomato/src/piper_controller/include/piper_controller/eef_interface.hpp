#ifndef _eef_interface_hpp_
#define _eef_interface_hpp_

#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "piper_controller/types.hpp"

namespace piper {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 关节类末端执行器接口
 */
class JointEefInterface {
public:
    JointEefInterface() = default;
    virtual ~JointEefInterface() = default;

    JointEefInterface(const JointEefInterface&) = delete;
    JointEefInterface& operator=(const JointEefInterface&) = delete;
    JointEefInterface(JointEefInterface&&) = delete;
    JointEefInterface& operator=(JointEefInterface&&) = delete;

    /**
     * @brief 获取末端执行器规划组名称
     * @return 规划组名称
     */
    virtual const std::string& get_group_name() const = 0;

    /**
     * @brief 获取 MoveGroupInterface 对象
     * @return MoveGroupInterface 引用
     */
    virtual moveit::planning_interface::MoveGroupInterface& get_move_group() = 0;

    /**
     * @brief 打开末端执行器
     * @return 错误码
     */
    virtual ErrorCode open() = 0;

    /**
     * @brief 关闭末端执行器
     * @return 错误码
     */
    virtual ErrorCode close() = 0;

    /**
     * @brief 执行末端预设位姿
     * @param pose_name 位姿名称
     * @return 错误码
     */
    virtual ErrorCode execute_preset_pose(const std::string& pose_name) = 0;

    /**
     * @brief 设置单关节值
     * @param joint_name 关节名称
     * @param value 目标值
     * @return 错误码
     */
    virtual ErrorCode set_joint_value(const std::string& joint_name, double value) = 0;

    /**
     * @brief 设置全部关节值
     * @param joint_values 目标关节值列表
     * @return 错误码
     */
    virtual ErrorCode set_joint_values(const std::vector<double>& joint_values) = 0;

    /**
     * @brief 规划末端轨迹
     * @param plan 规划输出
     * @return 错误码
     */
    virtual ErrorCode plan(moveit::planning_interface::MoveGroupInterface::Plan& plan) = 0;

    /**
     * @brief 执行末端轨迹
     * @param plan 轨迹输入
     * @return 错误码
     */
    virtual ErrorCode execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan) = 0;

    /**
     * @brief 规划并执行
     * @return 错误码
     */
    virtual ErrorCode plan_and_execute() = 0;

    /**
     * @brief 获取当前关节值
     */
    virtual std::vector<double> get_current_joints() const = 0;

    /**
     * @brief 获取当前连杆名称
     */
    virtual std::vector<std::string> get_current_link_names() const = 0;
};

/**
 * @brief IO 类末端执行器接口
 */
class IoEefInterface {
public:
    IoEefInterface() = default;
    virtual ~IoEefInterface() = default;

    IoEefInterface(const IoEefInterface&) = delete;
    IoEefInterface& operator=(const IoEefInterface&) = delete;
    IoEefInterface(IoEefInterface&&) = delete;
    IoEefInterface& operator=(IoEefInterface&&) = delete;

    virtual std::vector<std::string> get_io_names() const = 0;
    virtual ErrorCode enable_io(const std::string& io_name) = 0;
    virtual ErrorCode disable_io(const std::string& io_name) = 0;
    virtual ErrorCode enable_all() = 0;
    virtual ErrorCode disable_all() = 0;
};

/**
 * @brief PWM 类末端执行器接口
 */
class PwmEefInterface {
public:
    PwmEefInterface() = default;
    virtual ~PwmEefInterface() = default;

    PwmEefInterface(const PwmEefInterface&) = delete;
    PwmEefInterface& operator=(const PwmEefInterface&) = delete;
    PwmEefInterface(PwmEefInterface&&) = delete;
    PwmEefInterface& operator=(PwmEefInterface&&) = delete;

    virtual std::vector<std::string> get_io_names() const = 0;
    virtual ErrorCode set_pwm(const std::string& io_name, double pwm_value) = 0;
    virtual ErrorCode set_all_pwm(double pwm_value) = 0;
    virtual ErrorCode get_pwm(const std::string& io_name, double& pwm_value) const = 0;
    virtual ErrorCode get_all_pwm(std::map<std::string, double>& pwm_values) const = 0;
};

/**
 * @brief 力反馈类末端执行器接口
 */
class ForceFeedbackEefInterface {
public:
    ForceFeedbackEefInterface() = default;
    virtual ~ForceFeedbackEefInterface() = default;

    ForceFeedbackEefInterface(const ForceFeedbackEefInterface&) = delete;
    ForceFeedbackEefInterface& operator=(const ForceFeedbackEefInterface&) = delete;
    ForceFeedbackEefInterface(ForceFeedbackEefInterface&&) = delete;
    ForceFeedbackEefInterface& operator=(ForceFeedbackEefInterface&&) = delete;

    virtual std::vector<std::string> get_force_names() const = 0;
    virtual ErrorCode get_force(const std::string& force_name, double& force_value) const = 0;
};

/**
 * @brief 末端执行器抽象基类
 */
class EndEffector {
public:
    explicit EndEffector(const std::string& eef_name) : _eef_name_(eef_name) {}
    virtual ~EndEffector() = default;

    EndEffector(const EndEffector&) = delete;
    EndEffector& operator=(const EndEffector&) = delete;
    EndEffector(EndEffector&&) = delete;
    EndEffector& operator=(EndEffector&&) = delete;

    /**
     * @brief 获取末端执行器名称
     */
    virtual const std::string& get_eef_name() const { return _eef_name_; }

    /**
     * @brief 立即停止末端执行器动作
     */
    virtual void stop() = 0;

    /**
     * @brief 查询末端执行器支持能力
     */
    virtual bool supports_joint_control() const { return false; }
    virtual bool supports_io_control() const { return false; }
    virtual bool supports_fluid_control() const { return false; }
    virtual bool supports_force_feedback() const { return false; }
    virtual bool supports_grasp_planning() const { return false; }

    /**
     * @brief 设置 TCP 偏移（相对于 flange）
     * @param tcp_offset TCP 偏移位姿，即 tf_flange_tcp，表示从 flange 到 TCP 的变换矩阵
     */
    void set_tcp_offset(const geometry_msgs::Pose& tcp_offset) { _tf_flange_tcp_ = tcp_offset; }

    /**
     * @brief 获取 TCP 偏移（相对于 flange）
     * @return TCP 偏移位姿
     */
    const geometry_msgs::Pose& get_tcp_offset() const { return _tf_flange_tcp_; }

private:
    /**
     * @brief 位姿乘法：将两个位姿进行矩阵乘法，得到新的位姿
     * @param p_left 位姿乘法的左操作数，通常表示当前位姿或目标位姿
     * @param p_right 位姿乘法的右操作数，通常表示偏移位姿（如 TCP 偏移）
     * @return 位姿乘法的结果，表示将 p_right 应用到 p_left 上后的新位姿
     */
    inline geometry_msgs::Pose pose_multiply(const geometry_msgs::Pose& p_left, const geometry_msgs::Pose& p_right) const {
        tf2::Transform tf_left, tf_right;
        tf2::fromMsg(p_left, tf_left);
        tf2::fromMsg(p_right, tf_right);
        geometry_msgs::Pose placeholder;
        return tf2::toMsg(tf_left * tf_right, placeholder);
    }

    /**
     * @brief 位姿求逆：计算一个位姿的逆变换，得到从目标位姿到当前位姿的变换
     * @param pose 输入位姿，通常表示一个偏移位姿（如 TCP 偏移）
     * @return 位姿的逆变换，表示从目标位姿到当前位姿的变换
     */
    inline geometry_msgs::Pose pose_inverse(const geometry_msgs::Pose& pose) const {
        tf2::Transform tf;
        tf2::fromMsg(pose, tf);
        geometry_msgs::Pose placeholder;
        return tf2::toMsg(tf.inverse(), placeholder);
    }

    std::string _eef_name_;
    geometry_msgs::Pose _tf_flange_tcp_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

/**
 * @brief 获取末端执行器接口实例
 * @tparam InterfaceT 期望的接口类型，例如 JointEefInterface、IoEefInterface、PwmEefInterface 或 ForceFeedbackEefInterface
 * @param eef 末端执行器实例
 * @return 如果末端执行器支持该接口，则返回对应接口的指针；否则返回 nullptr
 */
template <typename InterfaceT>
InterfaceT* get_eef_interface(EndEffector* eef) {
    return eef ? dynamic_cast<InterfaceT*>(eef) : nullptr;
}
template <typename InterfaceT>
const InterfaceT* get_eef_interface(const EndEffector* eef) {
    return eef ? dynamic_cast<const InterfaceT*>(eef) : nullptr;
}

} /* namespace piper */

#endif
