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
    /**
     * @brief STM32 串口通信类构造函数
     * @param nh ROS 节点句柄
     * @param port_name 串口名称
     * @param baud_rate 波特率
     */
    STM32Serial(ros::NodeHandle& nh, const std::string& port_name, int baud_rate);
    /**
     * @brief STM32 串口通信类析构函数
     */
    ~STM32Serial();

    bool isConnected() const { return _is_connected_; }
    /**
     * @brief 连接串口
     * @return 是否连接成功
     */
    bool connect();
    /**
     * @brief 重新连接串口
     * @return 是否连接成功
     */
    bool reConnect();
    /**
     * @brief 断开串口连接
     */
    void disConnect();

    /**
     * @brief 发送字符串数据到串口
     * @param data 待发送数据
     * @return 是否发送成功
     */
    bool sendData(const std::string& data);
    /**
     * @brief 接收串口数据
     * @param timeout_ms 超时时间，单位毫秒
     * @return 接收到的数据
     */
    std::string rcvdData(uint32_t timeout_ms = 0);
    /**
     * @brief 清空串口缓冲区
     */
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
