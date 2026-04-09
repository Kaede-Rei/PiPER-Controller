#include "piper_task/pick_action.hpp"

namespace piper {

// ! ========================= 私 有 函 数 实 现 ========================= ! //

namespace {

#define X(name, desc) case PickStage::PICK_##name: return desc;
std::string pick_stage_to_text(PickStage stage) {
    switch(stage) {
        PICK_STAGE_TABLE
        default: return "未知阶段";
    }
}
#undef X

}  // namespace

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief PickTaskAction 构造函数
 * @param nh ROS 节点句柄
 * @param tasks_manager 任务管理器
 * @param arm 机械臂控制器
 * @param action_name Action 名称
 */
PickTaskAction::PickTaskAction(ros::NodeHandle& nh,
    std::shared_ptr<TasksManager> tasks_manager,
    std::shared_ptr<ArmController> arm,
    const std::string& action_name)
    : _tasks_manager_(std::move(tasks_manager)), _arm_(std::move(arm)) {
    _as_ = std::make_unique<PickTaskAS>(nh, action_name, false);
    _as_->registerGoalCallback(boost::bind(&PickTaskAction::on_goal, this));
    _as_->registerPreemptCallback(boost::bind(&PickTaskAction::on_preempt, this));
    _as_->start();
}

/**
 * @brief 处理新的 Goal
 */
void PickTaskAction::on_goal() {
    if(!_tasks_manager_ || !_arm_) {
        piper_msgs2::PickTaskResult res;
        res.success = false;
        res.error_code = static_cast<int32_t>(ErrorCode::INVALID_INTERFACE);
        res.message = "任务管理器或机械臂控制器未初始化";
        res.canceled = false;
        res.final_pose = get_current_pose_stamped();
        _as_->setAborted(res, res.message);
        return;
    }

    auto goal = _as_->acceptNewGoal();
    if(!goal) {
        piper_msgs2::PickTaskResult res;
        res.success = false;
        res.error_code = static_cast<int32_t>(ErrorCode::INVALID_PARAMETER);
        res.message = "空目标";
        res.canceled = false;
        res.final_pose = get_current_pose_stamped();
        _as_->setAborted(res, res.message);
        return;
    }

    switch(goal->request_type) {
        case piper_msgs2::PickTaskGoal::UPSERT_TASK:
            handle_upsert_task_request(*goal);
            return;
        case piper_msgs2::PickTaskGoal::EXECUTE_TASK_GROUP:
            handle_execute_task_group_request(*goal);
            return;
        default: {
            piper_msgs2::PickTaskResult res;
            res.success = false;
            res.error_code = static_cast<int32_t>(ErrorCode::INVALID_PARAMETER);
            res.message = "未知请求类型";
            res.canceled = false;
            res.final_pose = get_current_pose_stamped();
            _as_->setAborted(res, res.message);
            return;
        }
    }
}

/**
 * @brief 处理取消请求
 */
void PickTaskAction::on_preempt() {
    _cancel_requested_.store(true);
}

/**
 * @brief 处理“写入 / 更新任务”请求
 * @param goal Goal 请求
 */
void PickTaskAction::handle_upsert_task_request(const piper_msgs2::PickTaskGoal& goal) {
    const std::string group_name = resolve_group_name(goal);
    const unsigned int task_id = resolve_task_id(goal);

    TaskType task_type = TaskType::PICK;
    ErrorCode code = resolve_task_type(goal, task_type);
    if(code != ErrorCode::SUCCESS) {
        piper_msgs2::PickTaskResult res;
        res.success = false;
        res.error_code = static_cast<int32_t>(code);
        res.message = "任务类型无效";
        res.canceled = false;
        res.final_pose = get_current_pose_stamped();
        _as_->setAborted(res, res.message);
        return;
    }

    code = _tasks_manager_->create_task_group(group_name, SortType::ID);
    if(code != ErrorCode::SUCCESS && code != ErrorCode::TASK_GROUP_EXISTS) {
        piper_msgs2::PickTaskResult res;
        res.success = false;
        res.error_code = static_cast<int32_t>(code);
        res.message = "创建任务组失败";
        res.canceled = false;
        res.final_pose = get_current_pose_stamped();
        _as_->setAborted(res, res.message);
        return;
    }

    code = _tasks_manager_->delete_task(group_name, task_id);
    if(code != ErrorCode::SUCCESS && code != ErrorCode::TASK_NOT_FOUND) {
        piper_msgs2::PickTaskResult res;
        res.success = false;
        res.error_code = static_cast<int32_t>(code);
        res.message = "清理同 ID 历史任务失败";
        res.canceled = false;
        res.final_pose = get_current_pose_stamped();
        _as_->setAborted(res, res.message);
        return;
    }

    code = _tasks_manager_->add_task(group_name, task_id, task_type, goal.description);
    if(code != ErrorCode::SUCCESS) {
        piper_msgs2::PickTaskResult res;
        res.success = false;
        res.error_code = static_cast<int32_t>(code);
        res.message = "添加任务失败";
        res.canceled = false;
        res.final_pose = get_current_pose_stamped();
        _as_->setAborted(res, res.message);
        return;
    }

    tl::optional<TargetVariant> target;
    std::string target_frame;
    code = resolve_goal_target(goal, target, target_frame);
    if(code != ErrorCode::SUCCESS || !target) {
        piper_msgs2::PickTaskResult res;
        res.success = false;
        res.error_code = static_cast<int32_t>(code == ErrorCode::SUCCESS ? ErrorCode::INVALID_PARAMETER : code);
        res.message = "解析任务目标失败";
        res.canceled = false;
        res.final_pose = get_current_pose_stamped();
        _as_->setAborted(res, res.message);
        return;
    }

    code = _tasks_manager_->set_task_target(group_name, task_id, target, target_frame);
    if(code != ErrorCode::SUCCESS) {
        piper_msgs2::PickTaskResult res;
        res.success = false;
        res.error_code = static_cast<int32_t>(code);
        res.message = "设置任务目标失败";
        res.canceled = false;
        res.final_pose = get_current_pose_stamped();
        _as_->setAborted(res, res.message);
        return;
    }

    if(task_type == TaskType::PICK) {
        PickTaskParams pick_params;
        code = resolve_goal_pick_params(goal, pick_params);
        if(code != ErrorCode::SUCCESS) {
            piper_msgs2::PickTaskResult res;
            res.success = false;
            res.error_code = static_cast<int32_t>(code);
            res.message = "解析采摘参数失败";
            res.canceled = false;
            res.final_pose = get_current_pose_stamped();
            _as_->setAborted(res, res.message);
            return;
        }

        code = _tasks_manager_->set_task_pick_params(group_name, task_id, pick_params);
        if(code != ErrorCode::SUCCESS) {
            piper_msgs2::PickTaskResult res;
            res.success = false;
            res.error_code = static_cast<int32_t>(code);
            res.message = "设置采摘参数失败";
            res.canceled = false;
            res.final_pose = get_current_pose_stamped();
            _as_->setAborted(res, res.message);
            return;
        }
    }

    piper_msgs2::PickTaskResult res;
    res.success = true;
    res.error_code = static_cast<int32_t>(ErrorCode::SUCCESS);
    res.message = "任务组 '" + group_name + "' 中任务 ID " + std::to_string(task_id) + " 已写入，等待执行任务组请求";
    res.canceled = false;
    res.completed_steps = 0;
    res.final_pose = get_current_pose_stamped();
    _as_->setSucceeded(res, res.message);
}

/**
 * @brief 处理“执行任务组”请求
 * @param goal Goal 请求
 */
void PickTaskAction::handle_execute_task_group_request(const piper_msgs2::PickTaskGoal& goal) {
    _cancel_requested_.store(false);

    const std::string group_name = resolve_group_name(goal);
    uint32_t completed_steps = 0;
    PickStage last_stage = PickStage::PICK_IDLE;
    unsigned int last_task_id = 0;
    bool has_feedback = false;
    const uint32_t total_steps = estimate_total_steps(group_name);

    TasksManager::ExecutionContext ctx;
    ctx.cancel_requested = &_cancel_requested_;
    ctx.feedback_cb = [this, &completed_steps, &last_stage, &last_task_id, &has_feedback, total_steps](const Task& task, PickStage stage, bool last_success, ErrorCode last_code, const std::string& text) {
        if(!has_feedback || task.id != last_task_id || stage != last_stage) {
            ++completed_steps;
            last_stage = stage;
            last_task_id = task.id;
            has_feedback = true;
        }

        piper_msgs2::PickTaskFeedback fb;
        fb.current_stage = static_cast<uint8_t>(stage);
        fb.current_step_index = completed_steps;
        fb.total_steps = total_steps;
        fb.stage_text = text.empty() ? pick_stage_to_text(stage) : text;
        fb.last_substep_success = last_success;
        fb.last_error_code = static_cast<int32_t>(last_code);
        fb.current_pose = get_current_pose_stamped();
        _as_->publishFeedback(fb);

        if(_as_->isPreemptRequested()) {
            _cancel_requested_.store(true);
        }
        };

    const ErrorCode code = _tasks_manager_->execute_task_group(group_name, &ctx);

    piper_msgs2::PickTaskResult res;
    res.error_code = static_cast<int32_t>(code);
    res.failed_stage = static_cast<uint8_t>(last_stage);
    res.completed_steps = completed_steps;
    res.final_pose = get_current_pose_stamped();

    if(code == ErrorCode::CANCELLED || _cancel_requested_.load()) {
        res.success = false;
        res.canceled = true;
        res.message = "任务组 '" + group_name + "' 执行已取消";
        _as_->setPreempted(res, res.message);
        return;
    }

    if(code != ErrorCode::SUCCESS) {
        res.success = false;
        res.canceled = false;
        res.message = "任务组 '" + group_name + "' 执行失败";
        _as_->setAborted(res, res.message);
        return;
    }

    res.success = true;
    res.canceled = false;
    res.message = "任务组 '" + group_name + "' 执行成功";
    _as_->setSucceeded(res, res.message);
}

/**
 * @brief 从 Goal 中解析任务目标
 * @param goal Goal 请求
 * @param target 输出目标
 * @param target_frame 输出目标所属坐标系
 * @return 错误码
 */
ErrorCode PickTaskAction::resolve_goal_target(const piper_msgs2::PickTaskGoal& goal,
    tl::optional<TargetVariant>& target,
    std::string& target_frame) const {
    target = tl::nullopt;
    target_frame.clear();

    switch(goal.target_type) {
        case piper_msgs2::PickTaskGoal::TARGET_NONE:
            return ErrorCode::SUCCESS;
        case piper_msgs2::PickTaskGoal::TARGET_POSE:
            if(goal.target_frame_id.empty()) {
                ROS_WARN("Pose 目标必须显式提供 target_frame_id");
                return ErrorCode::INVALID_PARAMETER;
            }
            target = goal.target_pose;
            target_frame = goal.target_frame_id;
            return ErrorCode::SUCCESS;
        case piper_msgs2::PickTaskGoal::TARGET_POINT:
            if(goal.target_frame_id.empty()) {
                ROS_WARN("Point 目标必须显式提供 target_frame_id");
                return ErrorCode::INVALID_PARAMETER;
            }
            target = goal.target_point;
            target_frame = goal.target_frame_id;
            return ErrorCode::SUCCESS;
        case piper_msgs2::PickTaskGoal::TARGET_QUATERNION:
            if(goal.target_frame_id.empty()) {
                ROS_WARN("Quaternion 目标必须显式提供 target_frame_id");
                return ErrorCode::INVALID_PARAMETER;
            }
            target = goal.target_quaternion;
            target_frame = goal.target_frame_id;
            return ErrorCode::SUCCESS;
        case piper_msgs2::PickTaskGoal::TARGET_POSE_STAMPED:
            if(goal.target_pose_stamped.header.frame_id.empty()) {
                ROS_WARN("PoseStamped 目标必须显式提供 header.frame_id");
                return ErrorCode::INVALID_PARAMETER;
            }
            target = goal.target_pose_stamped;
            target_frame = goal.target_pose_stamped.header.frame_id;
            return ErrorCode::SUCCESS;
        default:
            ROS_WARN("未知任务目标类型：%u", static_cast<unsigned int>(goal.target_type));
            return ErrorCode::INVALID_PARAMETER;
    }
}

/**
 * @brief 从 Goal 中解析放置目标
 * @param goal Goal 请求
 * @param pick_params 输出采摘参数
 * @return 错误码
 */
ErrorCode PickTaskAction::resolve_goal_pick_params(const piper_msgs2::PickTaskGoal& goal,
    PickTaskParams& pick_params) const {
    pick_params.use_place_pose = goal.use_place_pose;
    pick_params.use_eef = goal.use_eef;
    pick_params.go_home_after_finish = goal.go_home_after_finish;
    pick_params.go_safe_after_cancel = goal.go_safe_after_cancel;
    pick_params.retry_times = goal.retry_times;
    pick_params.place_frame = _arm_->get_base_link();
    pick_params.place_target = tl::nullopt;

    if(!goal.use_place_pose) return ErrorCode::SUCCESS;

    switch(goal.place_target_type) {
        case piper_msgs2::PickTaskGoal::PLACE_TARGET_POSE:
            if(goal.place_frame_id.empty()) {
                ROS_WARN("放置 Pose 目标必须显式提供 place_frame_id");
                return ErrorCode::INVALID_PARAMETER;
            }
            if(goal.place_frame_id != _arm_->get_base_link()) {
                ROS_WARN("放置 Pose 目标必须位于 base_link 坐标系，当前为 '%s'", goal.place_frame_id.c_str());
                return ErrorCode::INVALID_PARAMETER;
            }
            pick_params.place_target = goal.place_pose;
            pick_params.place_frame = goal.place_frame_id;
            return ErrorCode::SUCCESS;
        case piper_msgs2::PickTaskGoal::PLACE_TARGET_POINT:
            if(goal.place_frame_id.empty()) {
                ROS_WARN("放置 Point 目标必须显式提供 place_frame_id");
                return ErrorCode::INVALID_PARAMETER;
            }
            if(goal.place_frame_id != _arm_->get_base_link()) {
                ROS_WARN("放置 Point 目标必须位于 base_link 坐标系，当前为 '%s'", goal.place_frame_id.c_str());
                return ErrorCode::INVALID_PARAMETER;
            }
            pick_params.place_target = goal.place_point;
            pick_params.place_frame = goal.place_frame_id;
            return ErrorCode::SUCCESS;
        case piper_msgs2::PickTaskGoal::PLACE_TARGET_POSE_STAMPED:
            if(goal.place_pose_stamped.header.frame_id.empty()) {
                ROS_WARN("放置 PoseStamped 目标必须显式提供 header.frame_id");
                return ErrorCode::INVALID_PARAMETER;
            }
            if(goal.place_pose_stamped.header.frame_id != _arm_->get_base_link()) {
                ROS_WARN("放置 PoseStamped 目标必须位于 base_link 坐标系，当前为 '%s'", goal.place_pose_stamped.header.frame_id.c_str());
                return ErrorCode::INVALID_PARAMETER;
            }
            pick_params.place_target = goal.place_pose_stamped;
            pick_params.place_frame = goal.place_pose_stamped.header.frame_id;
            return ErrorCode::SUCCESS;
        default:
            ROS_WARN("启用了放置动作，但放置目标类型无效");
            return ErrorCode::INVALID_PARAMETER;
    }
}

/**
 * @brief 获取当前机械臂位姿
 * @return 当前位姿（base_link）
 */
geometry_msgs::PoseStamped PickTaskAction::get_current_pose_stamped() const {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = _arm_ ? _arm_->get_base_link() : "base_link";
    pose_stamped.header.stamp = ros::Time::now();
    if(_arm_) pose_stamped.pose = _arm_->get_current_pose();
    return pose_stamped;
}

/**
 * @brief 解析任务组名称
 * @param goal Goal 请求
 * @return 任务组名称
 */
std::string PickTaskAction::resolve_group_name(const piper_msgs2::PickTaskGoal& goal) const {
    if(!goal.group_name.empty()) return goal.group_name;
    return "pick_default_group";
}

/**
 * @brief 解析任务 ID
 * @param goal Goal 请求
 * @return 任务 ID
 */
unsigned int PickTaskAction::resolve_task_id(const piper_msgs2::PickTaskGoal& goal) {
    if(goal.id != 0) return goal.id;
    return _id_seed_.fetch_add(1);
}

/**
 * @brief 解析任务类型
 * @param goal Goal 请求
 * @param task_type 输出任务类型
 * @return 错误码
 */
ErrorCode PickTaskAction::resolve_task_type(const piper_msgs2::PickTaskGoal& goal, TaskType& task_type) const {
    switch(goal.task_type) {
        case piper_msgs2::PickTaskGoal::TASK_MOVE_ONLY:
            task_type = TaskType::MOVE_ONLY;
            return ErrorCode::SUCCESS;
        case piper_msgs2::PickTaskGoal::TASK_PICK:
            task_type = TaskType::PICK;
            return ErrorCode::SUCCESS;
        default:
            return ErrorCode::INVALID_PARAMETER;
    }
}

/**
 * @brief 估算任务组总步骤数量
 * @param group_name 任务组名称
 * @return 预计总步骤数量
 */
uint32_t PickTaskAction::estimate_total_steps(const std::string& group_name) const {
    if(!_tasks_manager_) return 0;
    return _tasks_manager_->estimate_task_group_steps(group_name);
}

}  // namespace piper
