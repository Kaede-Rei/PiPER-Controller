#include "piper_interface/piper_interface.hpp"

#include "piper_controller/eef_controller.hpp"

namespace piper {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

ROSInterface::ROSInterface(ros::NodeHandle& nh, const ROSInterfaceConfig& config) {
    _arm_ = std::make_shared<ArmController>(config.arm_group_name);
    init_eef(nh, config);
    _dispatcher_ = std::make_shared<ArmCmdDispatcher>(_arm_);
    init_interfaces(nh, config);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

void ROSInterface::init_eef(ros::NodeHandle& nh, const ROSInterfaceConfig& config) {
    if(!config.eef_enabled) {
        ROS_INFO("EEF 未启用，跳过 attach_eef");
        return;
    }

    try {
        if(config.eef_type == "two_finger_gripper") {
            auto eef = std::make_shared<TwoFingerGripper>(config.eef_name);
            eef->set_tcp_offset(config.eef_tcp_offset);
            _eef_ = eef;
        }
        else if(config.eef_type == "servo_gripper") {
            auto eef = std::make_shared<ServoGripper>(
                nh,
                config.eef_tcp_offset,
                config.eef_serial_port,
                config.eef_baud_rate);
            _eef_ = eef;
        }
        else {
            ROS_WARN("未知 EEF 类型: %s，跳过 attach_eef", config.eef_type.c_str());
            return;
        }

        _arm_->attach_eef(_eef_);
        ROS_INFO("EEF 已挂载: type=%s, name=%s", config.eef_type.c_str(), _eef_->get_eef_name().c_str());
    }
    catch(const std::exception& e) {
        ROS_ERROR("EEF 初始化失败: %s", e.what());
        _eef_.reset();
    }
}

void ROSInterface::init_interfaces(ros::NodeHandle& nh, const ROSInterfaceConfig& config) {
    add_interface<ArmMoveAction>(config.arm_move_action, nh, _arm_, _dispatcher_);
    add_interface<SimpleArmMoveAction>(config.simple_arm_move_action, nh, _arm_, _dispatcher_);
    add_interface<ArmConfigService>(config.arm_config_service, nh, _arm_, _dispatcher_);
    add_interface<ArmQueryService>(config.arm_query_service, nh, _arm_, _dispatcher_);
}

}
