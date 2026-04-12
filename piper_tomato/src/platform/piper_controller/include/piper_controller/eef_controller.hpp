#ifndef _eef_controller_hpp_
#define _eef_controller_hpp_

#include "piper_controller/eef_interface.hpp"
#include "serial_driver/serial_driver.hpp"
#include "tl_optional/optional.hpp"

namespace piper {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //


/**
 * @brief 双指夹爪末端执行器类，支持关节控制和力反馈
 */
class TwoFingerGripper : public EndEffector,
    public GripperEefInterface,
    public JointEefInterface,
    public ForceFeedbackEefInterface {
public:
    explicit TwoFingerGripper(const std::string& eef_name = "gripper");
    ~TwoFingerGripper() override = default;

    TwoFingerGripper(const TwoFingerGripper&) = delete;
    TwoFingerGripper& operator=(const TwoFingerGripper&) = delete;
    TwoFingerGripper(TwoFingerGripper&&) = delete;
    TwoFingerGripper& operator=(TwoFingerGripper&&) = delete;

    void stop() override;

    const std::string& get_group_name() const override;
    moveit::planning_interface::MoveGroupInterface& get_move_group() override;
    ErrorCode open() override;
    ErrorCode close() override;
    ErrorCode set_angle(double angle) override;
    tl::optional<double> get_angle() const override;
    ErrorCode execute_preset_pose(const std::string& pose_name) override;
    ErrorCode set_joint_value(const std::string& joint_name, double value) override;
    ErrorCode set_joint_values(const std::vector<double>& joint_values) override;
    ErrorCode plan(moveit::planning_interface::MoveGroupInterface::Plan& plan) override;
    ErrorCode execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan) override;
    ErrorCode plan_and_execute() override;

    std::vector<double> get_current_joints() const override;
    std::vector<std::string> get_current_link_names() const override;

    std::vector<std::string> get_force_names() const override;
    tl::optional<double> get_force(const std::string& force_name) const override;

private:
    moveit::planning_interface::MoveGroupInterface _gripper_;
};

/**
 * @brief 总线舵机末端执行器类
 */
class ServoGripper : public EndEffector,
    public GripperEefInterface {
public:
    ServoGripper(ros::NodeHandle& nh, const std::string& serial_port = "/dev/ttyACM0", int baud_rate = 115200);
    ~ServoGripper() override = default;

    ServoGripper(const ServoGripper&) = delete;
    ServoGripper& operator=(const ServoGripper&) = delete;
    ServoGripper(ServoGripper&&) = delete;
    ServoGripper& operator=(ServoGripper&&) = delete;

    void stop() override;

    ErrorCode open() override;
    ErrorCode close() override;
    ErrorCode set_angle(double angle) override;
    tl::optional<double> get_angle() const override;

private:
    STM32Serial _serialer_;
    tl::optional<double> _current_angle_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! ///



} /* namespace piper */

#endif
