#ifndef _serial_driver_hpp_
#define _serial_driver_hpp_

#include <ros/ros.h>

#include <serial/serial.h>
#include <linux/can.h>
#include <net/if.h>

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

using CanFrame = struct can_frame;
using CanAddr = struct sockaddr_can;
using CanIfReq = struct ifreq;

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief STM32 串口通信类
 * @details 封装了与 STM32 通过串口进行通信的功能，包括：
 *          - 连接/重连与断开连接
 *          - 发送与接收数据(含超时接收/全部接收)
 *          - 清空缓冲区
 */
class STM32Serial {
public:
    STM32Serial(ros::NodeHandle& nh, const std::string& port_name, int baud_rate);
    ~STM32Serial();

    bool isConnected() const { return _is_connected_; }
    bool connect();
    bool reConnect();
    void disConnect();

    bool sendData(const std::string& data);
    std::string rcvdData(uint32_t timeout_ms = 0);
    void clearBuffer(void);

private:
    /// @brief ROS 节点句柄
    ros::NodeHandle _nh_;
    /// @brief 串口对象
    serial::Serial _serial_;
    /// @brief 串口名称
    std::string _port_name_;
    /// @brief 波特率
    int _baud_rate_;
    /// @brief 连接状态
    bool _is_connected_;
};

#endif
