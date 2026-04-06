#include "piper_interface/piper_interface.hpp"

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //



// ! ========================= 私 有 函 数 实 现 ========================= ! //

int main(int argc, char** argv) {
    ros::init(argc, argv, "piper_start");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    piper::ROSInterfaceConfig config;
    config.arm_group_name = pnh.param<std::string>("start/arm_group_name", "arm");
    config.eef_enabled = pnh.param<bool>("start/eef/enabled", false);
    config.eef_type = pnh.param<std::string>("start/eef/type", "");
    config.eef_name = pnh.param<std::string>("start/eef/name", "gripper");
    config.eef_serial_port = pnh.param<std::string>("start/eef/serial_port", "/dev/ttyACM0");
    config.eef_baud_rate = pnh.param<int>("start/eef/baud_rate", 115200);

    config.arm_move_action = { pnh.param<bool>("start/arm_move_action/enabled", true), pnh.param<std::string>("start/arm_move_action/name", "move_arm") };
    config.simple_arm_move_action = { pnh.param<bool>("start/simple_arm_move_action/enabled", true), pnh.param<std::string>("start/simple_arm_move_action/name", "simple_move_arm") };
    config.arm_config_service = { pnh.param<bool>("start/arm_config_service/enabled", true), pnh.param<std::string>("start/arm_config_service/name", "arm_config") };
    config.arm_query_service = { pnh.param<bool>("start/arm_query_service/enabled", true), pnh.param<std::string>("start/arm_query_service/name", "arm_query") };
    config.eef_cmd_service = { pnh.param<bool>("start/eef_command_service/enabled", true), pnh.param<std::string>("start/eef_command_service/name", "eef_command") };
    piper::ROSInterface piper_interface(nh, config);

    ros::waitForShutdown();
    return 0;
}
