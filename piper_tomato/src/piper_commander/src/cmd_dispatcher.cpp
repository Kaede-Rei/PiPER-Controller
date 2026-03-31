#include "piper_commander/cmd_dispatcher.hpp"

namespace piper {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief 分发命令请求，调用对应的处理函数执行命令
 * @param req 命令请求
 * @return 命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::dispatch(const ArmCmdRequest& req) {
    return dispatch(req, nullptr);
}

/**
 * @brief 分发命令请求，调用对应的处理函数执行命令，并通过回调函数报告执行进度
 * @param req 命令请求
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return 命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::dispatch(const ArmCmdRequest& req, FeedbackCb cb) {
    if(!_arm_) {
        return make_err(ErrorCode::FAILURE, "ArmController 未初始化");
    }
    _is_cancelled_.store(false);

    switch(req.type) {
        case ArmCmdType::MIN:
            return make_err(ErrorCode::FAILURE, "无效的命令类型：MIN");
        case ArmCmdType::HOME:
            return handle_home(req);
        case ArmCmdType::MOVE_JOINTS:
            return handle_move_joints(req, cb);
        case ArmCmdType::MOVE_TARGET:
            return handle_move_target(req, cb);
        case ArmCmdType::MOVE_TARGET_IN_EEF_FRAME:
            return handle_move_target_in_eef_frame(req, cb);
        case ArmCmdType::TELESCOPIC_END:
            return handle_telescopic_end(req, cb);
        case ArmCmdType::ROTATE_END:
            return handle_rotate_end(req, cb);
        case ArmCmdType::MOVE_LINE:
            return handle_move_line(req, cb);
        case ArmCmdType::MOVE_BEZIER:
            return handle_move_bezier(req, cb);
        case ArmCmdType::MOVE_DECARTES:
            return handle_move_decartes(req, cb);
        case ArmCmdType::SET_ORIENTATION_CONSTRAINT:
            return handle_set_orientation_constraint(req);
        case ArmCmdType::SET_POSITION_CONSTRAINT:
            return handle_set_position_constraint(req);
        case ArmCmdType::SET_JOINT_CONSTRAINT:
            return handle_set_joint_constraint(req);
        case ArmCmdType::GET_CURRENT_JOINTS:
            return handle_get_current_joints(req);
        case ArmCmdType::GET_CURRENT_POSE:
            return handle_get_current_pose(req);
        case ArmCmdType::MOVE_TO_ZERO:
            return handle_move_to_zero(req, cb);
        case ArmCmdType::MAX:
            return make_err(ErrorCode::FAILURE, "无效的命令类型：MAX");
    }

    return make_err(ErrorCode::FAILURE, "未知的命令类型");
}

/**
 * @brief 取消当前正在执行的命令
 */
void ArmCmdDispatcher::cancel() {
    _is_cancelled_.store(true);

    if(_arm_) {
        _arm_->cancel_async();
        _arm_->stop();
    }
}

/**
 * @brief 检查当前命令是否已被取消
 * @return 如果命令已取消则返回 true，否则返回 false
 */
bool ArmCmdDispatcher::is_cancelled() const {
    return _is_cancelled_.load();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

void ArmCmdDispatcher::fill_current_state(ArmCmdResult& result) const {
    if(!_arm_) {
        result.current_pose = geometry_msgs::Pose{};
        result.current_joints.clear();
        return;
    }

    result.current_pose = _arm_->get_current_pose();
    result.current_joints = _arm_->get_current_joints();

    const auto& q = result.current_pose.orientation;
    const double q_norm = std::sqrt(
        q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    );

    if(q_norm < 1e-9) {
        result.current_pose.orientation.x = 0.0;
        result.current_pose.orientation.y = 0.0;
        result.current_pose.orientation.z = 0.0;
        result.current_pose.orientation.w = 1.0;
    }
}

/**
 * @brief 构造一个成功的命令结果
 * @param msg 结果消息，默认为 "命令执行成功"
 * @return ArmCmdResult 结构体，表示命令执行成功
 */
ArmCmdResult ArmCmdDispatcher::make_ok(const std::string& msg) {
    ArmCmdResult result;
    result.success = true;
    result.message = msg;
    result.error_code = ErrorCode::SUCCESS;
    fill_current_state(result);

    return result;
}

/**
 * @brief 构造一个失败的命令结果
 * @param code 错误码，默认为 ErrorCode::FAILURE
 * @param msg 结果消息，默认为 "命令执行失败"
 * @return ArmCmdResult 结构体，表示命令执行失败
 */
ArmCmdResult ArmCmdDispatcher::make_err(ErrorCode code, const std::string& msg) {
    ArmCmdResult result;
    result.success = false;
    result.message = msg;
    result.error_code = code;
    fill_current_state(result);

    return result;
}

/**
 * @brief 构造一个被取消的命令结果
 * @return ArmCmdResult 结构体，表示命令已取消
 */
ArmCmdResult ArmCmdDispatcher::make_cancelled() {
    ArmCmdResult result;
    result.success = false;
    result.message = "命令已取消";
    result.error_code = ErrorCode::CANCELLED;
    fill_current_state(result);

    return result;
}

/**
 * @brief 通过回调函数报告命令执行进度
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @param stage 当前执行阶段，例如 "start"、"setting"、"planning"、"done" 等
 * @param progress 当前阶段的进度，范围 [0.0, 1.0]
 * @param message 当前阶段的提示信息
 */
void ArmCmdDispatcher::report(FeedbackCb cb, const std::string& stage, double progress, const std::string& message) {
    if(!cb) return;

    cb(ArmCmdFeedback{ stage, progress, message });
}

/**
 * @brief 如果命令未被取消，则继续执行后续操作
 * @param code 前一步操作的错误码，如果不为 ErrorCode::SUCCESS 则表示前一步失败
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::execute_if_not_cancelled(ErrorCode code, FeedbackCb cb) {
    if(is_cancelled()) {
        return make_cancelled();
    }

    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "设置目标失败");
    }

    report(cb, "planning", 0.5, "开始规划与执行");
    code = _arm_->plan_and_execute();
    if(is_cancelled()) {
        return make_cancelled();
    }

    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "规划或执行失败：" + err_to_string(code));
    }

    report(cb, "done", 1.0, "命令执行成功");
    return make_ok();
}

/**
 * @brief 处理 HOME 命令，调用 ArmController 的 home() 方法将机械臂移动到初始位置
 * @param req 命令请求，HOME 命令不需要额外参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_home(const ArmCmdRequest& req) {
    (void)req;

    if(is_cancelled()) {
        return make_cancelled();
    }

    const ErrorCode code = _arm_->home();
    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "执行 HOME 失败：" + err_to_string(code));
    }

    return make_ok();
}

/**
 * @brief 处理 MOVE_JOINTS 命令，调用 ArmController 的 set_joints() 方法设置目标关节角，并执行
 * @param req 命令请求，要求 req.joints 包含目标关节角列表
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_move_joints(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_JOINTS 命令");
    if(req.joints.empty()) {
        return make_err(ErrorCode::FAILURE, "关节角列表不能为空");
    }

    report(cb, "setting", 0.2, "正在设置目标关节角");
    ErrorCode code = _arm_->set_joints(req.joints);
    return execute_if_not_cancelled(code, cb);
}

/**
 * @brief 处理 MOVE_TARGET 命令，调用 ArmController 的 set_target() 方法设置目标位姿，并执行
 * @param req 命令请求，要求 req.target 包含目标位姿
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_move_target(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_TARGET 命令");
    if(std::holds_alternative<std::monostate>(req.target)) {
        return make_err(ErrorCode::FAILURE, "目标不能为空");
    }

    report(cb, "setting", 0.2, "正在设置目标位姿");
    ErrorCode code = _arm_->set_target(req.target);
    return execute_if_not_cancelled(code, cb);
}

/**
 * @brief 处理 MOVE_TARGET_IN_EEF_FRAME 命令，调用 ArmController 的 set_target_in_eef_frame() 方法设置工具坐标系下的目标位姿，并执行
 * @param req 命令请求，要求 req.target 包含目标位姿
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_move_target_in_eef_frame(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_TARGET_IN_EEF_FRAME 命令");
    if(std::holds_alternative<std::monostate>(req.target)) {
        return make_err(ErrorCode::FAILURE, "目标不能为空");
    }

    report(cb, "setting", 0.2, "正在设置目标位姿（工具坐标系）");
    ErrorCode code = _arm_->set_target_in_eef_frame(req.target);
    return execute_if_not_cancelled(code, cb);
}

/**
 * @brief 处理 TELESCOPIC_END 命令，调用 ArmController 的 telescopic_end() 方法设置伸缩末端目标，并执行
 * @param req 命令请求，要求 req.values 包含一个参数，表示伸缩长度
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_telescopic_end(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 TELESCOPIC_END 命令");
    if(req.values.size() != 1) {
        return make_err(ErrorCode::FAILURE, "伸缩末端命令需要一个参数：伸缩长度");
    }

    report(cb, "setting", 0.2, "正在设置伸缩末端目标");
    ErrorCode code = _arm_->telescopic_end(req.values[0]);
    return execute_if_not_cancelled(code, cb);
}

/**
 * @brief 处理 ROTATE_END 命令，调用 ArmController 的 rotate_end() 方法设置旋转末端目标，并执行
 * @param req 命令请求，要求 req.values 包含一个参数，表示旋转角度（弧度）
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_rotate_end(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 ROTATE_END 命令");
    if(req.values.size() != 1) {
        return make_err(ErrorCode::FAILURE, "旋转末端命令需要一个参数：旋转角度（弧度）");
    }

    report(cb, "setting", 0.2, "正在设置旋转末端目标");
    ErrorCode code = _arm_->rotate_end(req.values[0]);
    return execute_if_not_cancelled(code, cb);
}

/**
 * @brief 处理 MOVE_LINE 命令，调用 ArmController 的 set_line() 方法设置线性路径的起点和终点，并执行
 * @param req 命令请求，要求 req.waypoints 包含两个位姿点，分别作为线性路径的起点和终点
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_move_line(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_LINE 命令");
    if(req.waypoints.size() != 2) {
        return make_err(ErrorCode::FAILURE, "MOVE_LINE 命令需要两个路径点");
    }

    report(cb, "setting", 0.2, "正在规划路径点");
    DescartesResult result = _arm_->set_line(req.waypoints[0], req.waypoints[1]);
    if(result.error_code != ErrorCode::SUCCESS) {
        return make_err(result.error_code, "规划路径点失败：" + err_to_string(result.error_code));
    }

    if(is_cancelled()) {
        return make_cancelled();
    }

    report(cb, "planning", 0.5, "正在执行");
    ErrorCode code = _arm_->execute(result.trajectory);

    if(is_cancelled()) {
        return make_cancelled();
    }

    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "执行失败：" + err_to_string(code));
    }
    return make_ok();
}

/**
 * @brief 处理 MOVE_BEZIER 命令，调用 ArmController 的 set_bezier_curve() 方法设置三阶贝塞尔曲线的起点、控制点和终点，并执行
 * @param req 命令请求，要求 req.waypoints 包含三个位姿点，分别作为贝塞尔曲线的起点、控制点和终点
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_move_bezier(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_BEZIER 命令");
    if(req.waypoints.size() != 3) {
        return make_err(ErrorCode::FAILURE, "MOVE_BEZIER 命令需要三个路径点");
    }

    report(cb, "setting", 0.2, "正在规划路径点");
    DescartesResult result = _arm_->set_bezier_curve(req.waypoints[0], req.waypoints[1], req.waypoints[2]);
    if(result.error_code != ErrorCode::SUCCESS) {
        return make_err(result.error_code, "规划路径点失败：" + err_to_string(result.error_code));
    }

    if(is_cancelled()) {
        return make_cancelled();
    }

    report(cb, "planning", 0.5, "正在执行");
    ErrorCode code = _arm_->execute(result.trajectory);

    if(is_cancelled()) {
        return make_cancelled();
    }

    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "执行失败：" + err_to_string(code));
    }
    return make_ok();
}

/**
 * @brief 处理 MOVE_DECARTES 命令，调用 ArmController 的 plan_decartes() 方法规划笛卡尔空间路径，并执行
 * @param req 命令请求，要求 req.waypoints 包含多个位姿点，作为笛卡尔空间路径的关键点
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_move_decartes(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_DECARTES 命令");
    if(req.waypoints.empty()) {
        return make_err(ErrorCode::FAILURE, "路径点列表不能为空");
    }

    report(cb, "setting", 0.2, "正在规划路径点");
    DescartesResult result = _arm_->plan_decartes(req.waypoints);
    if(result.error_code != ErrorCode::SUCCESS) {
        return make_err(result.error_code, "规划路径点失败：" + err_to_string(result.error_code));
    }

    if(is_cancelled()) {
        return make_cancelled();
    }

    report(cb, "planning", 0.5, "正在执行");
    ErrorCode code = _arm_->execute(result.trajectory);

    if(is_cancelled()) {
        return make_cancelled();
    }

    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "执行失败：" + err_to_string(code));
    }
    return make_ok();
}

/**
 * @brief 处理 SET_ORIENTATION_CONSTRAINT 命令，调用 ArmController 的 set_orientation_constraint() 方法设置姿态约束
 * @param req 命令请求，要求 req.target 包含一个 geometry_msgs::Quaternion 作为目标姿态
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_set_orientation_constraint(const ArmCmdRequest& req) {
    if(is_cancelled()) {
        return make_cancelled();
    }

    if(std::holds_alternative<std::monostate>(req.target) ||
        !std::holds_alternative<geometry_msgs::Quaternion>(req.target)) {
        return make_err(ErrorCode::INVALID_PARAMETER, "目标必须是一个 Quaternion");
    }

    const geometry_msgs::Quaternion target_orientation =
        std::get<geometry_msgs::Quaternion>(req.target);

    _arm_->set_orientation_constraint(target_orientation);
    _arm_->apply_constraints();

    return make_ok("姿态约束已设置并应用");
}

/**
 * @brief 处理 SET_POSITION_CONSTRAINT 命令，调用 ArmController 的 set_position_constraint() 方法设置位置约束
 * @param req 命令请求，要求 req.target 包含一个 geometry_msgs::Point 作为目标位置，req.values 包含一个三维向量参数，表示约束范围大小
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_set_position_constraint(const ArmCmdRequest& req) {
    if(is_cancelled()) {
        return make_cancelled();
    }

    if(std::holds_alternative<std::monostate>(req.target) ||
        !std::holds_alternative<geometry_msgs::Point>(req.target)) {
        return make_err(ErrorCode::INVALID_PARAMETER, "目标必须是一个 Point");
    }

    if(req.values.size() != 3) {
        return make_err(ErrorCode::INVALID_PARAMETER, "位置约束需要 3 个范围参数：x/y/z");
    }

    geometry_msgs::Point target_position = std::get<geometry_msgs::Point>(req.target);
    geometry_msgs::Vector3 scope_size;
    scope_size.x = req.values[0];
    scope_size.y = req.values[1];
    scope_size.z = req.values[2];

    _arm_->set_position_constraint(target_position, scope_size);
    _arm_->apply_constraints();

    return make_ok("位置约束已设置并应用");
}

/**
 * @brief 处理 SET_JOINT_CONSTRAINT 命令，调用 ArmController 的 set_joint_constraint() 方法设置关节约束
 * @param req 命令请求，要求 req.joint_names 包含一个或多个关节名称，req.values 包含三个参数，分别表示约束范围的最小值、最大值和权重
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_set_joint_constraint(const ArmCmdRequest& req) {
    if(is_cancelled()) {
        return make_cancelled();
    }

    if(req.joint_names.empty()) {
        return make_err(ErrorCode::INVALID_PARAMETER, "关节约束名称列表不能为空");
    }

    if(req.joints.size() != req.joint_names.size()) {
        return make_err(ErrorCode::INVALID_PARAMETER,
            "关节约束 joint_names 与 joints 数量不一致");
    }

    if(req.values.size() != req.joint_names.size() * 2) {
        return make_err(ErrorCode::INVALID_PARAMETER,
            "关节约束 values 数量必须等于 joint_names.size() * 2");
    }

    for(std::size_t i = 0; i < req.joint_names.size(); ++i) {
        _arm_->set_joint_constraint(
            req.joint_names[i],
            req.joints[i],
            req.values[i * 2],
            req.values[i * 2 + 1]);
    }

    _arm_->apply_constraints();

    return make_ok("关节约束已设置并应用");
}

/**
 * @brief 处理 GET_CURRENT_JOINTS 命令，调用 ArmController 的 get_current_joints() 方法获取当前关节角
 * @param req 命令请求，GET_CURRENT_JOINTS 命令不需要额外参数
 * @return ArmCmdResult 结构体，表示命令执行结果，current_joints 字段包含当前关节角列表
 */
ArmCmdResult ArmCmdDispatcher::handle_get_current_joints(const ArmCmdRequest& req) {
    if(is_cancelled()) {
        return make_cancelled();
    }

    auto current_joints = _arm_->get_current_joints();
    if(current_joints.empty()) {
        return make_err(ErrorCode::FAILURE, "获取当前关节角失败");
    }

    ArmCmdResult result = make_ok("获取当前关节角成功");
    result.current_joints = std::move(current_joints);
    return result;
}

/**
 * @brief 处理 GET_CURRENT_POSE 命令，调用 ArmController 的 get_current_pose() 方法获取当前位姿
 * @param req 命令请求，GET_CURRENT_POSE 命令不需要额外参数
 * @return ArmCmdResult 结构体，表示命令执行结果，current_pose 字段包含当前位姿
 */
ArmCmdResult ArmCmdDispatcher::handle_get_current_pose(const ArmCmdRequest& req) {
    if(is_cancelled()) {
        return make_cancelled();
    }

    ArmCmdResult result = make_ok("获取当前位姿成功");
    result.current_pose = _arm_->get_current_pose();
    return result;
}

/**
 * @brief 处理 MOVE_TO_ZERO 命令，调用 ArmController 的 reset_to_zero() 方法将机械臂重置到零点位置，并执行
 * @param req 命令请求，MOVE_TO_ZERO 命令不需要额外参数
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::handle_move_to_zero(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_TO_ZERO 命令");

    ErrorCode code = _arm_->reset_to_zero();
    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "重置到零点失败：" + err_to_string(code));
    }

    report(cb, "done", 1.0, "命令执行成功");
    return make_ok();
}

}
