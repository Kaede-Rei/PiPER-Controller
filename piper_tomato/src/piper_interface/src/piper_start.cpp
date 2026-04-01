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
    config.eef_tcp_offset.position.x = pnh.param<double>("start/eef/tcp_offset/x", 0.0);
    config.eef_tcp_offset.position.y = pnh.param<double>("start/eef/tcp_offset/y", 0.0);
    config.eef_tcp_offset.position.z = pnh.param<double>("start/eef/tcp_offset/z", 0.0);
    config.eef_tcp_offset.orientation.x = pnh.param<double>("start/eef/tcp_offset/qx", 0.0);
    config.eef_tcp_offset.orientation.y = pnh.param<double>("start/eef/tcp_offset/qy", 0.0);
    config.eef_tcp_offset.orientation.z = pnh.param<double>("start/eef/tcp_offset/qz", 0.0);
    config.eef_tcp_offset.orientation.w = pnh.param<double>("start/eef/tcp_offset/qw", 1.0);

    config.arm_move_action = { pnh.param<bool>("start/arm_move_action/enabled", true), pnh.param<std::string>("start/arm_move_action/name", "move_arm") };
    config.simple_arm_move_action = { pnh.param<bool>("start/simple_arm_move_action/enabled", true), pnh.param<std::string>("start/simple_arm_move_action/name", "simple_move_arm") };
    config.arm_config_service = { pnh.param<bool>("start/arm_config_service/enabled", true), pnh.param<std::string>("start/arm_config_service/name", "arm_config") };
    config.arm_query_service = { pnh.param<bool>("start/arm_query_service/enabled", true), pnh.param<std::string>("start/arm_query_service/name", "arm_query") };
    piper::ROSInterface piper_interface(nh, config);

    ros::waitForShutdown();
    return 0;
}
