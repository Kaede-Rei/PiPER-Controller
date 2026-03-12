#include "serial_driver/serial_driver.hpp"

#include <sys/ioctl.h>
#include <linux/can/raw.h>

/* ========================= 接 口 量 声 明 ========================= */



/* ========================= 私 有 量 / 函 数 声 明 ========================= */

typedef struct can_filter CanFilter_t;

/* ========================= 接 口 类 / 函 数 实 现 ========================= */

/**
 * @brief STM32 串口通信类构造函数
 * @param nh ROS 节点句柄
 * @param port_name 串口名称，例如 "/dev/ttyUSB0"
 * @param baud_rate 波特率，例如 115200
 * @warning 禁止拷贝构造与移动构造
 */
STM32Serial::STM32Serial(ros::NodeHandle& nh, const std::string& port_name, int baud_rate)
    : _nh_(nh), _port_name_(port_name), _baud_rate_(baud_rate), _is_connected_(false) {
    ROS_INFO("STM32Serial 初始化完成，串口名称：%s，波特率：%d", _port_name_.c_str(), _baud_rate_);
}

/**
 * @brief STM32 串口通信类析构函数
 */
STM32Serial::~STM32Serial() {
    disConnect();
    ROS_INFO("STM32Serial 资源已释放");
}

/**
 * @brief 连接 STM32 串口
 * @return 连接成功返回 true，失败返回 false
 */
bool STM32Serial::connect() {
    if(_is_connected_) return true;

    try {
        _serial_.setPort(_port_name_);
        _serial_.setBaudrate(_baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        _serial_.setTimeout(timeout);
        _serial_.open();
    }
    catch(const std::exception& e) {
        ROS_ERROR("STM32Serial 连接失败：%s", e.what());
        return false;
    }

    if(!_serial_.isOpen()) {
        ROS_ERROR("STM32Serial 连接失败：串口未打开");
        return false;
    }

    _is_connected_ = true;
    ROS_INFO("STM32Serial 连接成功，串口名称：%s，波特率：%d", _port_name_.c_str(), _baud_rate_);
    return true;
}

/**
 * @brief 重新连接 STM32 串口
 * @return 连接成功返回 true，失败返回 false
 */
bool STM32Serial::reConnect() {
    disConnect();
    ros::Duration(0.5).sleep();
    return connect();
}

/**
 * @brief 断开 STM32 串口连接
 */
void STM32Serial::disConnect() {
    if(_serial_.isOpen()) {
        _serial_.close();
    }
    _is_connected_ = false;
    ROS_INFO("STM32Serial 已断开连接，串口名称：%s", _port_name_.c_str());
}

/**
 * @brief 发送数据到 STM32 串口
 * @param data 要发送的数据
 */
void STM32Serial::sendData(const std::string& data) {
    if(!_is_connected_) {
        ROS_ERROR("STM32Serial 发送数据失败：未连接");
        return;
    }

    size_t bytes_written = _serial_.write(data);
    ROS_INFO("STM32Serial 发送数据成功，字节数：%zu，内容：%s", bytes_written, data.c_str());
}

/**
 * @brief 从 STM32 串口接收数据
 * @param timeout_ms 超时时间，单位为毫秒，0 表示阻塞等待
 * @return 接收到的数据
 */
std::string STM32Serial::rcvdData(uint32_t timeout_ms) {
    std::string data;

    if(!_is_connected_) {
        ROS_ERROR("STM32Serial 接收数据失败：未连接");
        return data;
    }

    if(timeout_ms > 0) {
        serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
        _serial_.setTimeout(timeout);
    }

    size_t bytes_available = _serial_.available();
    if(bytes_available > 0) {
        data = _serial_.read(bytes_available);
        ROS_INFO("STM32Serial 接收数据成功，字节数：%zu", data.size());
    }
    else {
        ROS_WARN("STM32Serial 接收数据超时，超时时间：%d ms", timeout_ms);
    }

    return data;
}

/**
 * @brief 清空 STM32 串口缓冲区
 */
void STM32Serial::clearBuffer(void) {
    if(!_is_connected_) {
        ROS_ERROR("STM32Serial 清空缓冲区失败：未连接");
        return;
    }

    _serial_.flush();
    ROS_INFO("STM32Serial 缓冲区已清空");
}
