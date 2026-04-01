#include "piper_controller/arm_controller.hpp"

#include <cmath>
#include <cstdint>
#include <queue>
#include <sstream>
#include <unordered_map>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf2/LinearMath/Quaternion.h>

namespace piper {

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief ArmController 构造函数：初始化 MoveGroupInterface、TF 与参数
 * @param group_name 机械臂规划组名称
 */
ArmController::ArmController(const std::string& group_name)
    : _arm_(group_name),
    _tf_listener_(_tf_buffer_),
    _base_link_(_arm_.getPlanningFrame()),
    _eef_link_(_arm_.getEndEffectorLink()) {

    ROS_INFO("Planning Frame - %s 已创建", _base_link_.c_str());
    ROS_INFO("End Effector Link - %s 已创建", _eef_link_.c_str());

    ros::NodeHandle pnh("~");

    pnh.param("decartes/vel_scale", _vel_scale_, 0.1);
    pnh.param("decartes/acc_scale", _acc_scale_, 0.1);
    pnh.param("decartes/eef_step", _eef_step_, 0.01);
    pnh.param("decartes/jump_threshold", _jump_threshold_, 0.0);
    pnh.param("decartes/min_success_rate", _min_success_rate_, 0.8);

    double planning_time = 5.0;
    int planning_attempts = 10;
    double motion_vel_scale = 0.1;
    double motion_acc_scale = 0.1;
    std::string planner_id = "RRTConnect";

    pnh.param("motion_planning/planning_time", planning_time, 5.0);
    pnh.param("motion_planning/planning_attempts", planning_attempts, 10);
    pnh.param("motion_planning/max_velocity_scaling_factor", motion_vel_scale, 0.1);
    pnh.param("motion_planning/max_acceleration_scaling_factor", motion_acc_scale, 0.1);
    pnh.param("motion_planning/planner_id", planner_id, std::string("RRTConnect"));

    _arm_.setPlanningTime(planning_time);
    _arm_.setNumPlanningAttempts(planning_attempts);
    _arm_.setMaxVelocityScalingFactor(motion_vel_scale);
    _arm_.setMaxAccelerationScalingFactor(motion_acc_scale);
    _arm_.setPlannerId(planner_id);

    ros::Duration(1.0).sleep();

    try {
        if(_tf_buffer_.canTransform(_base_link_, _eef_link_, ros::Time(0), ros::Duration(1.0))) {
            ROS_INFO("TF 变换可用：%s -> %s", _base_link_.c_str(), _eef_link_.c_str());
        }
        else {
            ROS_WARN("TF 变换不可用：%s -> %s", _base_link_.c_str(), _eef_link_.c_str());
        }
    }
    catch(const tf2::TransformException& e) {
        ROS_WARN("检查 TF 变换时发生异常：%s", e.what());
    }
}

/**
 * @brief 析构函数：等待异步线程退出
 */
ArmController::~ArmController() {
    if(_async_thread_.joinable()) {
        _async_thread_.join();
    }
}

/**
 * @brief 回到命名位姿 home
 */
ErrorCode ArmController::home() {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法执行 home");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    ROS_INFO("将机械臂复位到命名位姿 zero");

    const bool target_ok = _arm_.setNamedTarget("zero");
    if(!target_ok) {
        ROS_WARN("设置命名位姿 zero 失败");
        return ErrorCode::TARGET_OUT_OF_BOUNDS;
    }

    const moveit::core::MoveItErrorCode err_code = _arm_.move();
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("执行 home 失败，错误码：%d", err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    ROS_INFO("执行 home 成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置目标关节值
 * @param joint_values 关节值向量
 * @return 错误码
 */
ErrorCode ArmController::set_joints(const std::vector<double>& joint_values) {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法设置新目标");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }
    bool success = _arm_.setJointValueTarget(joint_values);
    return success ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
}

/**
 * @brief 设置末端目标（底座坐标系），Pose 目标优先尝试 A* 可达性搜索
 * @param target 目标位姿/位置/姿态
 * @return 错误码
 */
ErrorCode ArmController::set_target(const TargetVariant& target) {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法设置新目标");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    const TargetVariant target_for_planning = _eef_ ? _eef_->tcp_to_flange(target) : target;
    if(_eef_) {
        ROS_INFO("检测到已挂载 EEF，目标将按 tcp_offset 从 TCP 转换到 flange");
    }

    ErrorCode result = ErrorCode::INVALID_TARGET_TYPE;

    std::visit(variant_visitor{
        [this, &result](const std::monostate&) {
            ROS_WARN("目标未设置");
            result = ErrorCode::INVALID_TARGET_TYPE;
        },
        [this, &result](const geometry_msgs::Pose& pose) {
            geometry_msgs::Pose current_pose = this->_arm_.getCurrentPose().pose;
            double score = -1.0;
            std::vector<double> joint_positions;
            geometry_msgs::Pose reachable_pose;

            const auto search_result = search_reachable_pose(
                current_pose, pose, score, joint_positions, reachable_pose);

            if((search_result == SearchReachablePose_e::SOLUTION_FOUND ||
                search_result == SearchReachablePose_e::APPROXIMATE_SOLUTION_FOUND) &&
               !joint_positions.empty()) {
                const bool ok = this->_arm_.setJointValueTarget(joint_positions);
                ROS_INFO("设置目标位姿（IK/A*关节解）是否成功：%s", ok ? "是" : "否");
                result = ok ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
                return;
            }

            ROS_INFO("未找到可达关节解，回退到 MoveIt PoseTarget 规划");
            const bool ok = this->_arm_.setPoseTarget(pose);
            ROS_INFO("设置目标位姿（PoseTarget）是否成功：%s", ok ? "是" : "否");
            result = ok ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
        },
        [this, &result](const geometry_msgs::Point& point) {
            const bool ok = this->_arm_.setPositionTarget(point.x, point.y, point.z);
            ROS_INFO("设置目标位置是否成功：%s", ok ? "是" : "否");
            result = ok ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
        },
        [this, &result](const geometry_msgs::Quaternion& quat) {
            const bool ok = this->_arm_.setOrientationTarget(quat.x, quat.y, quat.z, quat.w);
            ROS_INFO("设置目标姿态是否成功：%s", ok ? "是" : "否");
            result = ok ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
        },
        [this, &result](const geometry_msgs::PoseStamped& pose_stamped) {
            const bool ok = this->_arm_.setPoseTarget(pose_stamped);
            ROS_INFO("设置目标位姿（带时间戳）是否成功：%s", ok ? "是" : "否");
            result = ok ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
        }
        }, target_for_planning);

    return result;
}

/**
 * @brief 设置末端目标（末端坐标系）
 * @param target 目标位姿/位置/姿态
 * @return 错误码
 */
ErrorCode ArmController::set_target_in_eef_frame(const TargetVariant& target) {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法设置新目标");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    const TargetVariant target_for_planning = _eef_ ? _eef_->tcp_to_flange(target) : target;
    if(_eef_) {
        ROS_INFO("检测到已挂载 EEF，EEF 坐标系目标将按 tcp_offset 从 TCP 转换到 flange");
    }

    bool success = std::visit(variant_visitor{
        [this](const std::monostate&) {
            ROS_WARN("目标未设置");
            return false;
        },
        [this](const geometry_msgs::Pose& pose) {
            geometry_msgs::Pose transformed_pose;
            if(end_to_base_tf(pose, transformed_pose) != ErrorCode::SUCCESS) return false;
            return this->set_target(transformed_pose) == ErrorCode::SUCCESS;
        },
        [this](const geometry_msgs::Point& point) {
            geometry_msgs::Point transformed_point;
            if(end_to_base_tf(point, transformed_point) != ErrorCode::SUCCESS) return false;
            bool res = this->_arm_.setPositionTarget(transformed_point.x, transformed_point.y, transformed_point.z);
            ROS_INFO("设置末端坐标系目标位置是否成功：%s", res ? "是" : "否");
            return res;
        },
        [this](const geometry_msgs::Quaternion& quat) {
            geometry_msgs::Quaternion transformed_quat;
            if(end_to_base_tf(quat, transformed_quat) != ErrorCode::SUCCESS) return false;
            bool res = this->_arm_.setOrientationTarget(transformed_quat.x, transformed_quat.y, transformed_quat.z, transformed_quat.w);
            ROS_INFO("设置末端坐标系目标姿态是否成功：%s", res ? "是" : "否");
            return res;
        },
        [this](const geometry_msgs::PoseStamped& pose_stamped) {
            geometry_msgs::PoseStamped transformed_pose_stamped;
            if(end_to_base_tf(pose_stamped, transformed_pose_stamped) != ErrorCode::SUCCESS) return false;
            return this->set_target(transformed_pose_stamped.pose) == ErrorCode::SUCCESS;
        }
        }, target_for_planning);

    return success ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
}

/**
 * @brief 清空当前目标
 */
void ArmController::clear_target() {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法清除目标");
        return;
    }
    _arm_.clearPoseTargets();
    ROS_INFO("已清除所有目标位姿");
}

/**
 * @brief 末端伸缩设置
 * @param length 伸缩长度（米）
 * @return 错误码
 */
ErrorCode ArmController::telescopic_end(double length) {
    geometry_msgs::Pose point;
    point.position.x = 0.0;
    point.position.y = 0.0;
    point.position.z = length;
    point.orientation = rpy_to_quaternion(0.0, 0.0, 0.0);

    return set_target_in_eef_frame(point);
}

/**
 * @brief 末端旋转设置
 * @param angle 旋转角（弧度）
 * @return 错误码
 */
ErrorCode ArmController::rotate_end(double angle) {
    geometry_msgs::Pose pose;
    pose = rpy_to_pose(0.0, 0.0, angle, 0.0, 0.0, 0.0);

    return set_target_in_eef_frame(pose);
}

/**
 * @brief 执行路径规划
 * @param plan 规划输出
 * @return 错误码
 */
ErrorCode ArmController::plan(moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法进行新的规划");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    ROS_INFO("正在规划...");

    moveit::core::MoveItErrorCode err_code = _arm_.plan(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("规划失败，错误码：%d", err_code.val);
        return ErrorCode::PLANNING_FAILED;
    }

    ROS_INFO("规划成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 执行规划结果
 * @param plan 规划输入
 * @return 错误码
 */
ErrorCode ArmController::execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法执行新的规划结果");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    ROS_INFO("正在执行规划结果...");

    moveit::core::MoveItErrorCode err_code = _arm_.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("执行失败，错误码：%d", err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    ROS_INFO("执行成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 规划并执行
 * @return 错误码
 */
ErrorCode ArmController::plan_and_execute() {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法进行新的规划和执行");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    ROS_INFO("正在规划...");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit::core::MoveItErrorCode err_code = _arm_.plan(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("规划失败，错误码：%d", err_code.val);
        return ErrorCode::PLANNING_FAILED;
    }

    ROS_INFO("规划成功，正在执行...");
    err_code = _arm_.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("执行失败，错误码：%d", err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    ROS_INFO("执行成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 异步规划并执行
 * @param callback 完成回调
 * @return 错误码
 */
ErrorCode ArmController::async_plan_and_execute(std::function<void(ErrorCode)> callback) {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，请稍后再试");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    if(_async_thread_.joinable()) {
        _async_thread_.join();
    }

    _is_planning_or_executing_ = true;
    ROS_INFO("正在异步规划...");

    _async_thread_ = std::thread([this, callback]() {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode err_code = _arm_.plan(plan);
        if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_WARN("异步规划失败，错误码：%d", err_code.val);
            _is_planning_or_executing_ = false;
            if(callback) callback(ErrorCode::PLANNING_FAILED);
        }
        else {
            ROS_INFO("异步规划成功，正在执行...");
            err_code = _arm_.execute(plan);
            if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
                ROS_WARN("异步执行失败，错误码：%d", err_code.val);
                _is_planning_or_executing_ = false;
                if(callback) callback(ErrorCode::EXECUTION_FAILED);
            }
            else {
                ROS_INFO("异步执行成功");
                _is_planning_or_executing_ = false;
                if(callback) callback(ErrorCode::SUCCESS);
            }
        }
        });

    return ErrorCode::SUCCESS;
}

/**
 * @brief 停止当前运动
 */
void ArmController::stop() {
    _arm_.stop();
    ROS_INFO("已停止当前运动");
}

/**
 * @brief 对轨迹做时间参数化
 * @param trajectory 轨迹输入输出
 * @param method 时间参数化方法
 * @param vel_scale 速度缩放
 * @param acc_scale 加速度缩放
 * @return 错误码
 */
ErrorCode ArmController::parameterize_time(moveit_msgs::RobotTrajectory& trajectory, TimeParamMethod method, double vel_scale, double acc_scale) {
    robot_trajectory::RobotTrajectory rt(_arm_.getRobotModel(), _arm_.getName());
    rt.setRobotTrajectoryMsg(*_arm_.getCurrentState(), trajectory);

    bool time_param_success = false;
    if(method == TimeParamMethod::TOTG) {
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        time_param_success = totg.computeTimeStamps(rt, vel_scale, acc_scale);
    }
    else if(method == TimeParamMethod::ISP) {
        trajectory_processing::IterativeSplineParameterization isp;
        time_param_success = isp.computeTimeStamps(rt, vel_scale, acc_scale);
    }
    else {
        ROS_WARN("无效的时间参数化方法");
        return ErrorCode::TIME_PARAM_FAILED;
    }

    if(!time_param_success) {
        ROS_WARN("时间参数化失败");
        return ErrorCode::TIME_PARAM_FAILED;
    }

    rt.getRobotTrajectoryMsg(trajectory);
    return ErrorCode::SUCCESS;
}

/**
 * @brief 笛卡尔轨迹规划
 * @param waypoints 路径点
 * @param eef_step 末端步长
 * @param time_param_method 时间参数化方法
 * @param vel_scale 速度缩放
 * @param acc_scale 加速度缩放
 * @return 规划结果
 */
DescartesResult ArmController::plan_decartes(const std::vector<geometry_msgs::Pose>& waypoints, double eef_step, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
    DescartesResult result;
    result.error_code = ErrorCode::FAILURE;
    result.success_rate = 0.0;
    result.message.clear();
    result.trajectory = moveit_msgs::RobotTrajectory();

    if(_is_planning_or_executing_) {
        result.error_code = ErrorCode::ASYNC_TASK_RUNNING;
        result.message = "当前已有异步任务正在执行，无法进行新的笛卡尔规划";
        return result;
    }

    if(waypoints.empty()) {
        result.error_code = ErrorCode::EMPTY_WAYPOINTS;
        result.message = "路径点列表为空";
        return result;
    }

    if(eef_step <= 0.0) {
        result.error_code = ErrorCode::INVALID_PARAMETER;
        result.message = "eef_step 必须大于 0";
        return result;
    }

    if(vel_scale <= 0.0 || vel_scale > 1.0 || acc_scale <= 0.0 || acc_scale > 1.0) {
        result.error_code = ErrorCode::INVALID_PARAMETER;
        result.message = "速度/加速度缩放必须在 (0, 1] 范围内";
        return result;
    }

    moveit_msgs::RobotTrajectory trajectory;
    const double success_rate = _arm_.computeCartesianPath(
        waypoints,
        eef_step,
        trajectory);

    result.success_rate = success_rate;

    if(success_rate <= 0.0) {
        result.error_code = ErrorCode::DESCARTES_PLANNING_FAILED;
        result.message = "笛卡尔路径规划失败，无法生成有效轨迹";
        return result;
    }

    if(success_rate < _min_success_rate_) {
        std::stringstream ss;
        ss << "笛卡尔路径规划成功率不足："
            << (success_rate * 100.0)
            << "%，低于阈值 "
            << (_min_success_rate_ * 100.0)
            << "%";
        result.error_code = ErrorCode::DESCARTES_PLANNING_FAILED;
        result.message = ss.str();
        return result;
    }

    const ErrorCode time_code = parameterize_time(
        trajectory, time_param_method, vel_scale, acc_scale);
    if(time_code != ErrorCode::SUCCESS) {
        result.error_code = time_code;
        result.message = "时间参数化失败";
        return result;
    }

    result.trajectory = trajectory;
    result.error_code = ErrorCode::SUCCESS;

    std::stringstream ss;
    ss << "笛卡尔路径规划成功，成功率：" << (success_rate * 100.0) << "%";
    result.message = ss.str();

    return result;
}

/**
 * @brief 规划直线路径
 * @return 规划结果
 */
DescartesResult ArmController::set_line(const TargetVariant& start, const TargetVariant& end, double eef_step, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
    DescartesResult result;
    result.error_code = ErrorCode::FAILURE;
    result.success_rate = 0.0f;
    result.message.clear();

    const auto start_pose = extract_pose_from_target(start);
    const auto end_pose = extract_pose_from_target(end);

    if(!start_pose || !end_pose) {
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
        result.message = "无法从目标中提取位姿";
        return result;
    }

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(*start_pose);
    waypoints.push_back(*end_pose);

    return plan_decartes(waypoints, eef_step, time_param_method, vel_scale, acc_scale);
}

/**
 * @brief 规划贝塞尔曲线路径
 * @return 规划结果
 */
DescartesResult ArmController::set_bezier_curve(const TargetVariant& start, const TargetVariant& via, const TargetVariant& end, int curve_segments, double eef_step, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
    DescartesResult result;
    result.error_code = ErrorCode::FAILURE;
    result.success_rate = 0.0;
    result.message.clear();

    if(curve_segments < 1) {
        result.error_code = ErrorCode::INVALID_PARAMETER;
        result.message = "curve_segments 必须大于等于 1";
        return result;
    }

    const auto start_pose = extract_pose_from_target(start);
    if(!start_pose) {
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
        result.message = "贝塞尔曲线起点无效";
        return result;
    }

    const auto via_pose = extract_pose_from_target(via);
    if(!via_pose) {
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
        result.message = "贝塞尔曲线控制点无效";
        return result;
    }

    const auto end_pose = extract_pose_from_target(end);
    if(!end_pose) {
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
        result.message = "贝塞尔曲线终点无效";
        return result;
    }

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.reserve(static_cast<std::size_t>(curve_segments + 1));

    for(int i = 0; i <= curve_segments; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(curve_segments);

        geometry_msgs::Pose point;
        point.position.x =
            (1.0 - t) * (1.0 - t) * start_pose->position.x +
            2.0 * (1.0 - t) * t * via_pose->position.x +
            t * t * end_pose->position.x;
        point.position.y =
            (1.0 - t) * (1.0 - t) * start_pose->position.y +
            2.0 * (1.0 - t) * t * via_pose->position.y +
            t * t * end_pose->position.y;
        point.position.z =
            (1.0 - t) * (1.0 - t) * start_pose->position.z +
            2.0 * (1.0 - t) * t * via_pose->position.z +
            t * t * end_pose->position.z;

        tf2::Quaternion quat_start, quat_end, quat_interp;
        tf2::fromMsg(start_pose->orientation, quat_start);
        tf2::fromMsg(end_pose->orientation, quat_end);
        quat_interp = quat_start.slerp(quat_end, t);
        quat_interp.normalize();
        point.orientation = tf2::toMsg(quat_interp);

        waypoints.push_back(point);
    }

    return plan_decartes(waypoints, eef_step, time_param_method, vel_scale, acc_scale);
}

/**
 * @brief 执行轨迹消息
 * @param trajectory 轨迹
 * @return 错误码
 */
ErrorCode ArmController::execute(const moveit_msgs::RobotTrajectory& trajectory) {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法执行新轨迹");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    ROS_INFO("正在按预设轨迹执行...");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    moveit::core::MoveItErrorCode err_code = _arm_.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("执行失败，错误码：%d", err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    ROS_INFO("执行成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 异步执行轨迹消息
 * @param trajectory 轨迹
 * @param callback 完成回调
 * @return 错误码
 */
ErrorCode ArmController::async_execute(const moveit_msgs::RobotTrajectory& trajectory, std::function<void(ErrorCode)> callback) {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，请稍后再试");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    if(_async_thread_.joinable()) {
        _async_thread_.join();
    }

    _is_planning_or_executing_ = true;
    ROS_INFO("正在异步按预设轨迹执行...");

    _async_thread_ = std::thread([this, trajectory, callback]() {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        moveit::core::MoveItErrorCode err_code = _arm_.execute(plan);
        if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_WARN("异步执行失败，错误码：%d", err_code.val);
            _is_planning_or_executing_ = false;
            if(callback) callback(ErrorCode::EXECUTION_FAILED);
        }
        else {
            ROS_INFO("异步执行成功");
            _is_planning_or_executing_ = false;
            if(callback) callback(ErrorCode::SUCCESS);
        }
        });

    return ErrorCode::SUCCESS;
}

/**
 * @brief 添加姿态约束
 */
void ArmController::set_orientation_constraint(const geometry_msgs::Quaternion& target_orientation, double tolerance_x, double tolerance_y, double tolerance_z, double weight) {
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = _eef_link_;
    ocm.header.frame_id = _base_link_;
    ocm.header.stamp = ros::Time::now();

    ocm.orientation = target_orientation;
    ocm.absolute_x_axis_tolerance = tolerance_x;
    ocm.absolute_y_axis_tolerance = tolerance_y;
    ocm.absolute_z_axis_tolerance = tolerance_z;
    ocm.weight = weight;

    _constraints_.orientation_constraints.push_back(ocm);
    ROS_INFO("已设置末端执行器姿态约束，目标姿态：(%f, %f, %f, %f)，容忍度：(%f, %f, %f)，权重：%f",
        target_orientation.x, target_orientation.y, target_orientation.z, target_orientation.w,
        tolerance_x, tolerance_y, tolerance_z, weight);
}

/**
 * @brief 添加位置约束
 */
void ArmController::set_position_constraint(const geometry_msgs::Point& target_position, const geometry_msgs::Vector3& scope_size, double weight) {
    moveit_msgs::PositionConstraint pcm;
    pcm.link_name = _eef_link_;
    pcm.header.frame_id = _base_link_;
    pcm.header.stamp = ros::Time::now();

    pcm.target_point_offset.x = 0.0;
    pcm.target_point_offset.y = 0.0;
    pcm.target_point_offset.z = 0.0;

    shape_msgs::SolidPrimitive bounding_volume;
    bounding_volume.type = shape_msgs::SolidPrimitive::BOX;
    bounding_volume.dimensions.resize(3);
    bounding_volume.dimensions[0] = scope_size.x;
    bounding_volume.dimensions[1] = scope_size.y;
    bounding_volume.dimensions[2] = scope_size.z;

    geometry_msgs::Pose region_pose;
    region_pose.position = target_position;
    region_pose.orientation.w = 1.0;

    pcm.constraint_region.primitives.clear();
    pcm.constraint_region.primitive_poses.clear();
    pcm.constraint_region.primitives.push_back(bounding_volume);
    pcm.constraint_region.primitive_poses.push_back(region_pose);
    pcm.weight = weight;

    _constraints_.position_constraints.push_back(pcm);

    ROS_INFO("已设置末端执行器位置约束，目标位置：(%f, %f, %f)，范围大小：(%f, %f, %f)，权重：%f",
        target_position.x, target_position.y, target_position.z,
        scope_size.x, scope_size.y, scope_size.z, weight);
}
/**
 * @brief 添加关节约束
 */
void ArmController::set_joint_constraint(const std::string& joint_name, double target_angle, double above, double below, double weight) {
    moveit_msgs::JointConstraint jc;
    jc.joint_name = joint_name;
    jc.position = target_angle;
    jc.tolerance_above = above;
    jc.tolerance_below = below;
    jc.weight = weight;

    _constraints_.joint_constraints.push_back(jc);
    ROS_INFO("已设置末端执行器关节约束，关节名称：%s，目标角度：%f，上容忍度：%f，下容忍度：%f，权重：%f",
        joint_name.c_str(), target_angle, above, below, weight);
}

/**
 * @brief 应用约束
 */
void ArmController::apply_constraints() {
    _arm_.setPathConstraints(_constraints_);
    ROS_INFO("已应用所有姿态约束");
}

/**
 * @brief 清空约束
 */
void ArmController::clear_constraints() {
    _constraints_ = moveit_msgs::Constraints();
    _arm_.clearPathConstraints();
    ROS_INFO("已清除所有姿态约束");
}

/**
 * @brief 查询异步任务状态
 */
bool ArmController::is_planning_or_executing() const {
    return _is_planning_or_executing_;
}

/**
 * @brief 取消异步任务
 * @return 错误码
 */
ErrorCode ArmController::cancel_async() {
    if(!_is_planning_or_executing_) {
        ROS_INFO("当前没有正在进行的异步任务");
        return ErrorCode::SUCCESS;
    }

    ROS_INFO("正在取消异步任务...");
    _arm_.stop();

    if(_async_thread_.joinable()) {
        _async_thread_.join();
    }

    _is_planning_or_executing_ = false;
    ROS_INFO("异步任务已取消");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 在当前姿态基础上相对旋转 RPY 角度并转换为四元数（底座坐标系）
 * @param q_in 当前姿态四元数
 * @param roll 旋转角度（弧度）
 * @param pitch 旋转角度（弧度）
 * @param yaw 旋转角度（弧度）
 * @return 旋转后的姿态四元数
 */
geometry_msgs::Quaternion ArmController::rotate_relative_rpy_to_quaternion(const geometry_msgs::Quaternion& q_in, double roll, double pitch, double yaw) {
    tf2::Quaternion q_in_tf2, q_relative, q_out;
    tf2::fromMsg(q_in, q_in_tf2);
    q_relative.setRPY(roll, pitch, yaw);
    q_out = q_in_tf2 * q_relative;
    q_out.normalize();

    return tf2::toMsg(q_out);
}

/**
 * @brief RPY 转四元数
 */
geometry_msgs::Quaternion ArmController::rpy_to_quaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    quat.normalize();

    geometry_msgs::Quaternion quat_msg;
    quat_msg.x = quat.x();
    quat_msg.y = quat.y();
    quat_msg.z = quat.z();
    quat_msg.w = quat.w();

    return quat_msg;
}

/**
 * @brief RPY + 平移 转 Pose
 */
geometry_msgs::Pose ArmController::rpy_to_pose(double roll, double pitch, double yaw, double x, double y, double z) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = rpy_to_quaternion(roll, pitch, yaw);

    return pose;
}

/**
 * @brief 获取规划组名称
 */
const std::string& ArmController::get_arm_name() const {
    return _arm_.getName();
}

/**
 * @brief 获取当前关节值
 */
std::vector<double> ArmController::get_current_joints() const {
    return _arm_.getCurrentJointValues();
}

/**
 * @brief 获取当前末端位姿
 */
geometry_msgs::Pose ArmController::get_current_pose() const {
    return _arm_.getCurrentPose().pose;
}

/**
 * @brief 获取当前连杆名称
 */
std::vector<std::string> ArmController::get_current_link_names() const {
    return _arm_.getLinkNames();
}

/**
 * @brief 关节全部归零并执行
 * @return 错误码
 */
ErrorCode ArmController::reset_to_zero() {
    std::vector<double> zeroes = get_current_joints();
    if(zeroes.empty()) {
        ROS_WARN("无法获取当前关节，重置到零点失败");
        return ErrorCode::FAILURE;
    }

    for(auto& value : zeroes) {
        value = 0.0;
    }

    const ErrorCode set_code = set_joints(zeroes);
    if(set_code != ErrorCode::SUCCESS) {
        ROS_WARN("设置零点关节目标失败：%s", err_to_string(set_code).c_str());
        return set_code;
    }

    return plan_and_execute();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将 TargetVariant 提取为 Pose
 */
tl::optional<geometry_msgs::Pose> ArmController::extract_pose_from_target(const TargetVariant& target) const {
    const geometry_msgs::Pose current_pose = get_current_pose();

    return std::visit(variant_visitor{
        [](const std::monostate&) -> tl::optional<geometry_msgs::Pose> {
            return tl::nullopt;
        },
        [](const geometry_msgs::Pose& pose) -> tl::optional<geometry_msgs::Pose> {
            return pose;
        },
        [&current_pose](const geometry_msgs::Point& point) -> tl::optional<geometry_msgs::Pose> {
            geometry_msgs::Pose pose;
            pose.position = point;
            pose.orientation = current_pose.orientation;
            return pose;
        },
        [&current_pose](const geometry_msgs::Quaternion& quat) -> tl::optional<geometry_msgs::Pose> {
            geometry_msgs::Pose pose;
            pose.position = current_pose.position;
            pose.orientation = quat;
            return pose;
        },
        [](const geometry_msgs::PoseStamped& pose_stamped) -> tl::optional<geometry_msgs::Pose> {
            return pose_stamped.pose;
        }
        }, target);
}
/**
 * @brief A* 搜索最近可达目标位姿
 * @param current_pose 当前位姿
 * @param target_pose 目标位姿
 * @param score 搜索评分输出
 * @param reachable_joints 可达关节解输出
 * @param reachable_pose 可达位姿输出
 * @return 搜索结果
 */
SearchReachablePose_e ArmController::search_reachable_pose(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& target_pose, double& score, std::vector<double>& reachable_joints, geometry_msgs::Pose& reachable_pose) {
    (void)current_pose;

    score = -1.0;
    reachable_joints.clear();
    reachable_pose = target_pose;

    _current_state_ = _arm_.getCurrentState(1.0);
    if(!_current_state_) {
        ROS_ERROR("无法获取当前机械臂状态，A* 可达性搜索终止。");
        return SearchReachablePose_e::SOLUTION_NOT_FOUND;
    }

    _jmg_ = _current_state_->getJointModelGroup(_arm_.getName());
    if(!_jmg_) {
        ROS_ERROR("无法获取关节模型组 '%s'，A* 可达性搜索终止。", _arm_.getName().c_str());
        return SearchReachablePose_e::SOLUTION_NOT_FOUND;
    }

    auto try_ik = [&](const geometry_msgs::Pose& pose_base,
        double candidate_score,
        SearchReachablePose_e state) {
            moveit::core::RobotState state_copy(*_current_state_);
            if(!state_copy.setFromIK(_jmg_, pose_base, 0.0)) {
                return SearchReachablePose_e::SOLUTION_NOT_FOUND;
            }
            state_copy.copyJointGroupPositions(_jmg_, reachable_joints);
            reachable_pose = pose_base;
            score = candidate_score;
            return state;
        };

    const auto exact_result = try_ik(target_pose, 0.0, SearchReachablePose_e::SOLUTION_FOUND);
    if(exact_result == SearchReachablePose_e::SOLUTION_FOUND) {
        ROS_INFO("目标位姿可直接到达，无需 A* 搜索。");
        return exact_result;
    }

    geometry_msgs::TransformStamped tf_stamped;
    geometry_msgs::TransformStamped tf_stamped_inv;
    geometry_msgs::Pose target_pose_eef;

    try {
        tf_stamped = _tf_buffer_.lookupTransform(_eef_link_, _base_link_, ros::Time(0), ros::Duration(1.0));
        tf_stamped_inv = _tf_buffer_.lookupTransform(_base_link_, _eef_link_, ros::Time(0), ros::Duration(1.0));
        target_pose_eef = target_pose;
        tf2::doTransform(target_pose_eef, target_pose_eef, tf_stamped);
    }
    catch(const tf2::TransformException& e) {
        ROS_WARN("A* 可达性搜索 TF 变换失败：%s", e.what());
        return SearchReachablePose_e::SOLUTION_NOT_FOUND;
    }

    ros::NodeHandle pnh("~");
    double step_deg = 5.0;
    double radius_deg = 60.0;
    int max_expand = 624;
    pnh.param("reachable_pose_search/step_deg", step_deg, 5.0);
    pnh.param("reachable_pose_search/radius_deg", radius_deg, 60.0);
    max_expand = std::pow((2 * static_cast<int>(radius_deg / step_deg) + 1), 2) - 1;

    const double step = std::max(step_deg, 0.1) * M_PI / 180.0;
    const double radius = std::max(radius_deg, step_deg) * M_PI / 180.0;

    tf2::Quaternion q_orig;
    tf2::fromMsg(target_pose_eef.orientation, q_orig);
    double roll_orig = 0.0;
    double pitch_orig = 0.0;
    double yaw_orig = 0.0;
    tf2::Matrix3x3(q_orig).getRPY(roll_orig, pitch_orig, yaw_orig);

    auto encode_key = [](int roll_idx, int pitch_idx) -> std::int64_t {
        return (static_cast<std::int64_t>(roll_idx) << 32) ^
            static_cast<std::uint32_t>(pitch_idx);
        };

    auto heuristic = [](double droll, double dpitch) {
        return std::hypot(droll, dpitch);
        };

    auto build_node = [&](double droll, double dpitch, double g_cost, int depth) {
        AStarNode_t node;
        node.g_cost = g_cost;
        node.h_cost = heuristic(droll, dpitch);
        node.depth = depth;
        node.pose = target_pose_eef;

        tf2::Quaternion q_candidate;
        q_candidate.setRPY(roll_orig + droll, pitch_orig + dpitch, yaw_orig);
        q_candidate.normalize();
        node.pose.orientation = tf2::toMsg(q_candidate);
        return node;
        };

    std::priority_queue<AStarNode_t, std::vector<AStarNode_t>, CompareAStarNode> open_set;
    std::unordered_map<std::int64_t, double> best_g_by_idx;

    open_set.push(build_node(0.0, 0.0, 0.0, 0));
    best_g_by_idx[encode_key(0, 0)] = 0.0;

    const int dirs[8][2] = {
        { 1, 0 }, { -1, 0 },
        { 0, 1 }, { 0, -1 },
        { 1, 1 }, { -1, -1 },
        { 1, -1 }, { -1, 1 }
    };

    int expand_count = 0;
    while(!open_set.empty() && expand_count < max_expand) {
        AStarNode_t current_node = open_set.top();
        open_set.pop();
        ++expand_count;

        tf2::Quaternion q_current;
        tf2::fromMsg(current_node.pose.orientation, q_current);
        double roll_now = 0.0;
        double pitch_now = 0.0;
        double yaw_now = 0.0;
        tf2::Matrix3x3(q_current).getRPY(roll_now, pitch_now, yaw_now);

        const double droll = roll_now - roll_orig;
        const double dpitch = pitch_now - pitch_orig;

        if(current_node.depth > 0) {
            geometry_msgs::Pose pose_candidate_base;
            tf2::doTransform(current_node.pose, pose_candidate_base, tf_stamped_inv);

            const auto approx_result = try_ik(
                pose_candidate_base,
                current_node.h_cost,
                SearchReachablePose_e::APPROXIMATE_SOLUTION_FOUND);

            if(approx_result == SearchReachablePose_e::APPROXIMATE_SOLUTION_FOUND) {
                ROS_INFO("A* 搜索到近似可达位姿：droll=%.2f°, dpitch=%.2f°",
                    droll * 180.0 / M_PI,
                    dpitch * 180.0 / M_PI);
                return approx_result;
            }
        }

        const int roll_idx = static_cast<int>(std::round(droll / step));
        const int pitch_idx = static_cast<int>(std::round(dpitch / step));

        for(const auto& dir : dirs) {
            const int next_roll_idx = roll_idx + dir[0];
            const int next_pitch_idx = pitch_idx + dir[1];

            const double next_droll = next_roll_idx * step;
            const double next_dpitch = next_pitch_idx * step;
            if(std::hypot(next_droll, next_dpitch) > radius + 1e-12) {
                continue;
            }

            const double move_cost = std::hypot(next_droll - droll, next_dpitch - dpitch);
            const double next_g = current_node.g_cost + move_cost;
            const std::int64_t key = encode_key(next_roll_idx, next_pitch_idx);

            auto it = best_g_by_idx.find(key);
            if(it != best_g_by_idx.end() && it->second <= next_g) {
                continue;
            }

            best_g_by_idx[key] = next_g;
            open_set.push(build_node(next_droll, next_dpitch, next_g, current_node.depth + 1));
        }
    }

    ROS_WARN("A* 搜索未找到可达位姿（最大扩展=%d）。", max_expand);
    return SearchReachablePose_e::SOLUTION_NOT_FOUND;
}

} /* namespace piper */
