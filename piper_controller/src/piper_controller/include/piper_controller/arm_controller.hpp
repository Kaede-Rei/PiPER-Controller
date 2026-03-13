#ifndef _arm_controller_hpp_
#define _arm_controller_hpp_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Constraints.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "piper_controller/eef_controller.hpp"
#include "piper_controller/types.hpp"

namespace piper {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 位姿可达性搜索返回状态
 * @param SOLUTION_FOUND 找到精确可达解
 * @param APPROXIMATE_SOLUTION_FOUND 找到近似可达解
 * @param SOLUTION_NOT_FOUND 未找到可达解
 */
enum class SearchReachablePose_e {
    SOLUTION_FOUND = 0,
    APPROXIMATE_SOLUTION_FOUND,
    SOLUTION_NOT_FOUND
};

/**
 * @brief A* 搜索节点
 * @param g_cost 从起点到当前节点的累计代价
 * @param h_cost 从当前节点到目标节点的启发式代价
 * @param pose 当前节点对应的末端位姿
 * @param joint_positions 当前节点对应的关节角解
 * @param depth 当前搜索深度
 */
typedef struct {
    double g_cost;
    double h_cost;
    geometry_msgs::Pose pose;
    std::vector<double> joint_positions;
    int depth;
} AStarNode_t;

/**
 * @brief A* 优先队列比较器（总代价小者优先）
 * @param a 节点 a
 * @param b 节点 b
 * @return 若 a 的总代价大于 b 则返回 true
 */
struct CompareAStarNode {
    bool operator()(const AStarNode_t& a, const AStarNode_t& b) {
        return (a.g_cost + a.h_cost) > (b.g_cost + b.h_cost);
    }
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 机械臂控制器类，负责目标设置、运动规划、笛卡尔轨迹与约束管理
 */
class ArmController {
public:
    explicit ArmController(const std::string& group_name = "arm");

    /**
     * @brief 析构函数
     */
    ~ArmController();

    ArmController(const ArmController&) = delete;
    ArmController& operator=(const ArmController&) = delete;
    ArmController(ArmController&&) = delete;
    ArmController& operator=(ArmController&&) = delete;

    /**
     * @brief 获取末端执行器共享指针
     * @param eef 末端执行器共享指针
     */
    void attach_eef(std::shared_ptr<EndEffector> eef) { _eef_ = std::move(eef); }
    void home();

    ErrorCode set_joints(const std::vector<double>& joint_values);
    ErrorCode set_target(const TargetVariant& target);
    ErrorCode set_target_in_eef_frame(const TargetVariant& target);
    void clear_target();

    ErrorCode telescopic_end(double length);
    ErrorCode rotate_end(double angle);

    ErrorCode plan(moveit::planning_interface::MoveGroupInterface::Plan& plan);
    ErrorCode execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
    ErrorCode plan_and_execute();
    ErrorCode async_plan_and_execute(std::function<void(ErrorCode)> callback = nullptr);
    void stop();

    ErrorCode parameterize_time(moveit_msgs::RobotTrajectory& trajectory, TimeParamMethod method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    DescartesResult plan_decartes(const std::vector<geometry_msgs::Pose>& waypoints, double eef_step = 0.01, double jump_threshold = 0.0, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    DescartesResult set_line(const TargetVariant& start, const TargetVariant& end, double eef_step = 0.01, double jump_threshold = 0.0, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    DescartesResult set_bezier_curve(const TargetVariant& start, const TargetVariant& via, const TargetVariant& end, int curve_segments = 30, double eef_step = 0.01, double jump_threshold = 0.0, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    ErrorCode execute(const moveit_msgs::RobotTrajectory& trajectory);
    ErrorCode async_execute(const moveit_msgs::RobotTrajectory& trajectory, std::function<void(ErrorCode)> callback = nullptr);

    void set_orientation_constraint(const geometry_msgs::Quaternion& target_orientation, double tolerance_x = 0.1, double tolerance_y = 0.1, double tolerance_z = 0.3, double weight = 1.0);
    void set_position_constraint(const geometry_msgs::Point& target_position, const geometry_msgs::Vector3& scope_size, double weight = 1.0);
    void set_joint_constraint(const std::string& joint_name, double target_angle, double above, double below, double weight = 1.0);
    void apply_constraints();
    void clear_constraints();

    bool is_planning_or_executing() const;
    ErrorCode cancel_async();

    geometry_msgs::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);
    geometry_msgs::Pose rpy_to_pose(double roll, double pitch, double yaw, double x, double y, double z);
    template<class T>
    ErrorCode base_to_end_tf(const T& in, T& out);
    template<class T>
    ErrorCode end_to_base_tf(const T& in, T& out);

    const std::string& get_arm_name() const;
    std::vector<double> get_current_joints() const;
    std::vector<std::string> get_current_link_names() const;
    geometry_msgs::Pose get_current_pose() const;

    ErrorCode reset_to_zero();

private:
    geometry_msgs::Pose extract_pose_from_target(const TargetVariant& target) const;
    SearchReachablePose_e search_reachable_pose(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& target_pose, double& score, std::vector<double>& reachable_joints, geometry_msgs::Pose& reachable_pose);

private:
    /// @brief MoveGroupInterface 对象
    moveit::planning_interface::MoveGroupInterface _arm_;
    /// @brief 末端执行器对象
    std::shared_ptr<EndEffector> _eef_;

    /// @brief TF 缓冲区
    tf2_ros::Buffer _tf_buffer_;
    /// @brief TF 监听器
    tf2_ros::TransformListener _tf_listener_;

    /// @brief 规划基坐标系
    const std::string _base_link_;
    /// @brief 末端坐标系
    const std::string _eef_link_;

    /// @brief 笛卡尔轨迹速度缩放
    double _vel_scale_;
    /// @brief 笛卡尔轨迹加速度缩放
    double _acc_scale_;

    /// @brief 笛卡尔步长
    double _eef_step_;
    /// @brief 关节跳跃阈值
    double _jump_threshold_;
    /// @brief 笛卡尔最小成功率
    double _min_success_rate_;

    /// @brief 约束集合
    moveit_msgs::Constraints _constraints_;

    /// @brief 是否处于异步执行状态
    std::atomic<bool> _is_planning_or_executing_{ false };
    /// @brief 异步线程
    std::thread _async_thread_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

/**
 * @brief 将输入从基坐标系变换到末端坐标系
 */
template<class T>
ErrorCode ArmController::base_to_end_tf(const T& in, T& out) {
    static_assert(
        std::is_same_v<T, geometry_msgs::Pose> ||
        std::is_same_v<T, geometry_msgs::Point> ||
        std::is_same_v<T, geometry_msgs::Quaternion> ||
        std::is_same_v<T, geometry_msgs::PoseStamped>,
        "仅支持 Pose(Stamped)、Point 和 Quaternion 的坐标变换");

    try {
        if constexpr(std::is_same_v<T, geometry_msgs::PoseStamped>) {
            auto tf_stamped = _tf_buffer_.lookupTransform(_eef_link_, _base_link_, in.header.stamp, ros::Duration(0.2));
            tf2::doTransform(in, out, tf_stamped);
        }
        else {
            geometry_msgs::PoseStamped pose_in;
            pose_in.header.frame_id = _base_link_;
            pose_in.header.stamp = ros::Time::now();

            if constexpr(std::is_same_v<T, geometry_msgs::Pose>) {
                pose_in.pose = in;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Point>) {
                pose_in.pose.position = in;
                pose_in.pose.orientation.w = 1.0;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Quaternion>) {
                pose_in.pose.orientation = in;
            }

            geometry_msgs::PoseStamped pose_out;
            pose_out = _tf_buffer_.transform(pose_in, _eef_link_, ros::Duration(0.2));
            if constexpr(std::is_same_v<T, geometry_msgs::Pose>) {
                out = pose_out.pose;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Point>) {
                out = pose_out.pose.position;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Quaternion>) {
                out = pose_out.pose.orientation;
            }
        }
        ROS_INFO("坐标变换成功：%s -> %s", _base_link_.c_str(), _eef_link_.c_str());
        return ErrorCode::SUCCESS;
    }
    catch(const tf2::TransformException& e) {
        ROS_WARN("坐标变换失败：%s", e.what());
        return ErrorCode::TF_TRANSFORM_FAILED;
    }
}

/**
 * @brief 将输入从末端坐标系变换到底座坐标系
 */
template<class T>
ErrorCode ArmController::end_to_base_tf(const T& in, T& out) {
    static_assert(
        std::is_same_v<T, geometry_msgs::Pose> ||
        std::is_same_v<T, geometry_msgs::Point> ||
        std::is_same_v<T, geometry_msgs::Quaternion> ||
        std::is_same_v<T, geometry_msgs::PoseStamped>,
        "仅支持 Pose(Stamped)、Point 和 Quaternion 的坐标变换");

    try {
        if constexpr(std::is_same_v<T, geometry_msgs::PoseStamped>) {
            auto tf_stamped = _tf_buffer_.lookupTransform(_base_link_, _eef_link_, in.header.stamp, ros::Duration(0.2));
            tf2::doTransform(in, out, tf_stamped);
        }
        else {
            geometry_msgs::PoseStamped pose_in;
            pose_in.header.frame_id = _eef_link_;
            pose_in.header.stamp = ros::Time::now();

            if constexpr(std::is_same_v<T, geometry_msgs::Pose>) {
                pose_in.pose = in;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Point>) {
                pose_in.pose.position = in;
                pose_in.pose.orientation.w = 1.0;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Quaternion>) {
                pose_in.pose.orientation = in;
            }

            geometry_msgs::PoseStamped pose_out;
            pose_out = _tf_buffer_.transform(pose_in, _base_link_, ros::Duration(0.2));
            if constexpr(std::is_same_v<T, geometry_msgs::Pose>) {
                out = pose_out.pose;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Point>) {
                out = pose_out.pose.position;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Quaternion>) {
                out = pose_out.pose.orientation;
            }
        }
        ROS_INFO("坐标变换成功：%s -> %s", _eef_link_.c_str(), _base_link_.c_str());
        return ErrorCode::SUCCESS;
    }
    catch(const tf2::TransformException& e) {
        ROS_WARN("坐标变换失败：%s", e.what());
        return ErrorCode::TF_TRANSFORM_FAILED;
    }
}

} /* namespace piper */

#endif
