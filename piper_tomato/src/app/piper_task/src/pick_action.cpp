#include "piper_task/pick_action.hpp"

namespace piper {

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

    _cancel_requested_.store(false);

    const std::string group_name = resolve_group_name(*goal);
    const unsigned int task_id = resolve_task_id(*goal);

    ErrorCode code = _tasks_manager_->create_task_group(group_name, SortType::ID);
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

    code = _tasks_manager_->add_task(group_name, task_id, TaskType::PICK, goal->description);
    if(code != ErrorCode::SUCCESS) {
        piper_msgs2::PickTaskResult res;
        res.success = false;
        res.error_code = static_cast<int32_t>(code);
        res.message = "添加采摘任务失败";
        res.canceled = false;
        res.final_pose = get_current_pose_stamped();
        _as_->setAborted(res, res.message);
        return;
    }

    code = _tasks_manager_->set_task_target(group_name, task_id, goal->target_pose);
    if(code != ErrorCode::SUCCESS) {
        piper_msgs2::PickTaskResult res;
        res.success = false;
        res.error_code = static_cast<int32_t>(code);
        res.message = "设置采摘目标失败";
        res.canceled = false;
        res.final_pose = get_current_pose_stamped();
        _as_->setAborted(res, res.message);
        return;
    }

    PickTaskParams pick_params;
    pick_params.use_place_pose = goal->use_place_pose;
    pick_params.use_eef = goal->use_eef;
    pick_params.go_home_after_finish = goal->go_home_after_finish;
    pick_params.go_safe_after_cancel = goal->go_safe_after_cancel;
    pick_params.retry_times = goal->retry_times;
    if(goal->use_place_pose) {
        pick_params.place_target = goal->place_pose;
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

    uint32_t completed_steps = 0;
    PickStage last_stage = PickStage::PICK_IDLE;
    const uint32_t total_steps = estimate_total_steps(*goal);

    TasksManager::ExecutionContext ctx;
    ctx.cancel_requested = &_cancel_requested_;
    ctx.feedback_cb = [this, &completed_steps, &last_stage, total_steps](const Task&,
        PickStage stage,
        bool last_success,
        ErrorCode last_code,
        const std::string& text) {
            if(stage != last_stage) {
                ++completed_steps;
                last_stage = stage;
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

    code = _tasks_manager_->execute_task(group_name, task_id, &ctx);

    piper_msgs2::PickTaskResult res;
    res.error_code = static_cast<int32_t>(code);
    res.failed_stage = static_cast<uint8_t>(last_stage);
    res.completed_steps = completed_steps;
    res.final_pose = get_current_pose_stamped();

    if(code == ErrorCode::CANCELLED || _cancel_requested_.load()) {
        res.success = false;
        res.canceled = true;
        res.message = "采摘任务已取消";
        _as_->setPreempted(res, res.message);
        return;
    }

    if(code != ErrorCode::SUCCESS) {
        res.success = false;
        res.canceled = false;
        res.message = "采摘任务执行失败";
        _as_->setAborted(res, res.message);
        return;
    }

    res.success = true;
    res.canceled = false;
    res.message = "采摘任务执行成功";
    _as_->setSucceeded(res, res.message);
}

void PickTaskAction::on_preempt() {
    _cancel_requested_.store(true);
}

geometry_msgs::PoseStamped PickTaskAction::get_current_pose_stamped() const {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "base_link";
    pose_stamped.header.stamp = ros::Time::now();
    if(_arm_) pose_stamped.pose = _arm_->get_current_pose();
    return pose_stamped;
}

std::string PickTaskAction::resolve_group_name(const piper_msgs2::PickTaskGoal& goal) const {
    if(!goal.start_group.empty()) return goal.start_group;
    if(!goal.group_name.empty()) return goal.group_name;
    return "pick_default_group";
}

unsigned int PickTaskAction::resolve_task_id(const piper_msgs2::PickTaskGoal& goal) {
    if(goal.id != 0) return goal.id;
    return _id_seed_.fetch_add(1);
}

uint32_t PickTaskAction::estimate_total_steps(const piper_msgs2::PickTaskGoal& goal) const {
    uint32_t steps = 6;
    if(goal.use_eef) steps += 1;
    if(goal.use_place_pose) {
        steps += 1;
        if(goal.use_eef) steps += 1;
    }
    if(goal.go_home_after_finish) steps += 1;
    return steps;
}

}  // namespace piper
