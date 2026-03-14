#include "piper_commander/cmd_dispatcher.hpp"

namespace piper {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

ArmCmdResult ArmCmdDispatcher::dispatch(const ArmCmdRequest& req) {
    return dispatch(req, nullptr);
}

ArmCmdResult ArmCmdDispatcher::dispatch(const ArmCmdRequest& req, FeedbackCb cb) {
    if(!_arm_) {
        return make_err(ErrorCode::FAILURE, "ArmController 未初始化");
    }
    _is_cancelled_.store(false);

    switch(req.type) {
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
    }

    return make_err(ErrorCode::FAILURE, "未知的命令类型");
}

void ArmCmdDispatcher::cancel() {
    _is_cancelled_.store(true);

    if(_arm_) {
        _arm_->cancel_async();
        _arm_->stop();
    }
}

bool ArmCmdDispatcher::is_cancelled() const {
    return _is_cancelled_.load();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

ArmCmdResult ArmCmdDispatcher::make_ok(const std::string& msg) {
    ArmCmdResult result;
    result.success = true;
    result.message = msg;
    result.error_code = ErrorCode::SUCCESS;

    return result;
}

ArmCmdResult ArmCmdDispatcher::make_err(ErrorCode code, const std::string& msg) {
    ArmCmdResult result;
    result.success = false;
    result.message = msg;
    result.error_code = code;

    return result;
}

ArmCmdResult ArmCmdDispatcher::make_cancelled() {
    ArmCmdResult result;
    result.success = false;
    result.message = "命令已取消";
    result.error_code = ErrorCode::CANCELLED;

    return result;
}

void ArmCmdDispatcher::report(FeedbackCb cb, const std::string& stage, double progress, const std::string& message) {
    if(!cb) return;

    cb(ArmCmdFeedback{ stage, progress, message });
}

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

ArmCmdResult ArmCmdDispatcher::handle_home(const ArmCmdRequest& req) {
    if(is_cancelled()) {
        return make_cancelled();
    }

    _arm_->home();
    return make_ok();
}

ArmCmdResult ArmCmdDispatcher::handle_move_joints(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_JOINTS 命令");
    if(req.joints.empty()) {
        return make_err(ErrorCode::FAILURE, "关节角列表不能为空");
    }

    report(cb, "setting_target", 0.2, "正在设置目标关节角");
    ErrorCode code = _arm_->set_joints(req.joints);
    return execute_if_not_cancelled(code, cb);
}

ArmCmdResult ArmCmdDispatcher::handle_move_target(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_TARGET 命令");
    if(std::holds_alternative<std::monostate>(req.target)) {
        return make_err(ErrorCode::FAILURE, "目标不能为空");
    }

    report(cb, "setting_target", 0.2, "正在设置目标位姿");
    ErrorCode code = _arm_->set_target(req.target);
    return execute_if_not_cancelled(code, cb);
}

ArmCmdResult ArmCmdDispatcher::handle_move_target_in_eef_frame(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_TARGET_IN_EEF_FRAME 命令");
    if(std::holds_alternative<std::monostate>(req.target)) {
        return make_err(ErrorCode::FAILURE, "目标不能为空");
    }

    report(cb, "setting_target", 0.2, "正在设置目标位姿（工具坐标系）");
    ErrorCode code = _arm_->set_target_in_eef_frame(req.target);
    return execute_if_not_cancelled(code, cb);
}

ArmCmdResult ArmCmdDispatcher::handle_telescopic_end(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 TELESCOPIC_END 命令");
    if(req.values.size() != 1) {
        return make_err(ErrorCode::FAILURE, "伸缩末端命令需要一个参数：伸缩长度");
    }

    report(cb, "setting_target", 0.2, "正在设置伸缩末端目标");
    ErrorCode code = _arm_->telescopic_end(req.values[0]);
    return execute_if_not_cancelled(code, cb);
}

ArmCmdResult ArmCmdDispatcher::handle_rotate_end(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 ROTATE_END 命令");
    if(req.values.size() != 1) {
        return make_err(ErrorCode::FAILURE, "旋转末端命令需要一个参数：旋转角度（弧度）");
    }

    report(cb, "setting_target", 0.2, "正在设置旋转末端目标");
    ErrorCode code = _arm_->rotate_end(req.values[0]);
    return execute_if_not_cancelled(code, cb);
}

ArmCmdResult ArmCmdDispatcher::handle_move_line(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_LINE 命令");
    if(req.waypoints.size() != 2) {
        return make_err(ErrorCode::FAILURE, "MOVE_LINE 命令需要两个路径点");
    }

    report(cb, "setting_target", 0.2, "正在规划路径点");
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

ArmCmdResult ArmCmdDispatcher::handle_move_bezier(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_BEZIER 命令");
    if(req.waypoints.size() != 3) {
        return make_err(ErrorCode::FAILURE, "MOVE_BEZIER 命令需要三个路径点");
    }

    report(cb, "setting_target", 0.2, "正在规划路径点");
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
ArmCmdResult ArmCmdDispatcher::handle_move_decartes(const ArmCmdRequest& req, FeedbackCb cb) {
    report(cb, "start", 0.0, "开始执行 MOVE_DECARTES 命令");
    if(req.waypoints.empty()) {
        return make_err(ErrorCode::FAILURE, "路径点列表不能为空");
    }

    report(cb, "setting_target", 0.2, "正在规划路径点");
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

ArmCmdResult ArmCmdDispatcher::handle_set_orientation_constraint(const ArmCmdRequest& req) {
    if(is_cancelled()) {
        return make_cancelled();
    }
    if(std::holds_alternative<std::monostate>(req.target) || !std::holds_alternative<geometry_msgs::Quaternion>(req.target)) {
        return make_err(ErrorCode::FAILURE, "目标必须是一个 Quaternion");
    }

    geometry_msgs::Quaternion target_orientation = std::get<geometry_msgs::Quaternion>(req.target);
    _arm_->set_orientation_constraint(target_orientation);

    return make_ok();
}
ArmCmdResult ArmCmdDispatcher::handle_set_position_constraint(const ArmCmdRequest& req) {
    if(is_cancelled()) {
        return make_cancelled();
    }
    if(std::holds_alternative<std::monostate>(req.target) || !std::holds_alternative<geometry_msgs::Point>(req.target)) {
        return make_err(ErrorCode::FAILURE, "目标必须是一个 Point");
    }
    if(req.values.size() != 3) {
        return make_err(ErrorCode::FAILURE, "位置约束需要一个三维向量参数，表示约束范围大小");
    }

    geometry_msgs::Point target_position = std::get<geometry_msgs::Point>(req.target);
    geometry_msgs::Vector3 scope_size;
    scope_size.x = req.values[0];
    scope_size.y = req.values[1];
    scope_size.z = req.values[2];
    _arm_->set_position_constraint(target_position, scope_size);

    return make_ok();
}

ArmCmdResult ArmCmdDispatcher::handle_set_joint_constraint(const ArmCmdRequest& req) {
    if(is_cancelled()) {
        return make_cancelled();
    }
    if(req.joint_names.empty()) {
        return make_err(ErrorCode::FAILURE, "关节约束名称列表不能为空");
    }
    if(req.values.empty()) {
        return make_err(ErrorCode::FAILURE, "关节约束值列表不能为空");
    }

    for(const std::string& joint_name : req.joint_names) {
        _arm_->set_joint_constraint(joint_name, req.values[0], req.values[1], req.values[2]);
    }

    return make_ok();
}

ArmCmdResult ArmCmdDispatcher::handle_get_current_joints(const ArmCmdRequest& req) {
    if(is_cancelled()) {
        return make_cancelled();
    }

    ArmCmdResult result = make_ok("获取当前关节角成功");
    result.current_joints = _arm_->get_current_joints();
    return result;
}

ArmCmdResult ArmCmdDispatcher::handle_get_current_pose(const ArmCmdRequest& req) {
    if(is_cancelled()) {
        return make_cancelled();
    }

    ArmCmdResult result = make_ok("获取当前位姿成功");
    result.current_pose = _arm_->get_current_pose();
    return result;
}

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
