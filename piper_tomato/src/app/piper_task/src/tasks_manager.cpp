#include "piper_task/tasks_manager.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

#include <ros/ros.h>

namespace piper {

// ! ========================= 私 有 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 转字符串工具函数
 */
#define X(name, desc) case PickStage::PICK_##name: return desc;
const char* pick_stage_to_string(PickStage stage) {
    switch(stage) {
        PICK_STAGE_TABLE
        default: return "未知阶段";
    }
}
#undef X

/**
 * @brief 将底座坐标系下的目标补全为 Pose 以便用于距离计算
 * @param target 输入目标（已在底座坐标系）
 * @param reference_pose 缺失位置或姿态时使用的参考位姿
 * @return 补全后的 Pose
 */
geometry_msgs::Pose target_to_effective_pose_in_base(const TargetVariant& target, const geometry_msgs::Pose& reference_pose) {
    return std::visit(variant_visitor{
        [&](const std::monostate&) -> geometry_msgs::Pose {
            return reference_pose;
        },
        [&](const geometry_msgs::Pose& pose) -> geometry_msgs::Pose {
            return pose;
        },
        [&](const geometry_msgs::Point& point) -> geometry_msgs::Pose {
            geometry_msgs::Pose pose = reference_pose;
            pose.position = point;
            return pose;
        },
        [&](const geometry_msgs::Quaternion& quat) -> geometry_msgs::Pose {
            geometry_msgs::Pose pose = reference_pose;
            pose.orientation = quat;
            return pose;
        },
        [&](const geometry_msgs::PoseStamped& pose_stamped) -> geometry_msgs::Pose {
            return pose_stamped.pose;
        }
        }, target);
}

}  // namespace

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief TasksManager 构造函数
 * @param arm 机械臂控制器
 * @param eef 末端执行器
 */
TasksManager::TasksManager(std::shared_ptr<ArmController> arm, std::shared_ptr<EndEffector> eef)
    : _arm_(std::move(arm)), _eef_(std::move(eef)) {
    if(!_arm_) {
        ROS_ERROR("机械臂控制器不能为空，TasksManager 初始化失败");
        throw std::invalid_argument("ArmController pointer cannot be null");
    }

    _arm_name_ = _arm_->get_arm_name();
    if(_eef_) _eef_name_ = _eef_->get_eef_name();
}

/**
 * @brief 创建任务组
 * @param group_name 任务组名称
 * @param sort_type 排序方式
 * @return 错误码
 */
ErrorCode TasksManager::create_task_group(const std::string& group_name, SortType sort_type) {
    if(_task_groups_.find(group_name) != _task_groups_.end()) {
        return ErrorCode::TASK_GROUP_EXISTS;
    }

    TaskGroup new_group;
    new_group.sort_type = sort_type;
    _task_groups_[group_name] = std::move(new_group);
    ROS_INFO("成功创建任务组 '%s'", group_name.c_str());
    return ErrorCode::SUCCESS;
}

/**
 * @brief 删除任务组
 * @param group_name 任务组名称
 * @return 错误码
 */
ErrorCode TasksManager::delete_task_group(const std::string& group_name) {
    auto group = find_task_group(group_name);
    if(!group) return group.error();

    _task_groups_.erase(group_name);
    ROS_INFO("成功删除任务组 '%s'", group_name.c_str());
    return ErrorCode::SUCCESS;
}

/**
 * @brief 清空任务组
 * @param group_name 任务组名称
 * @return 错误码
 */
ErrorCode TasksManager::clear_task_group(const std::string& group_name) {
    auto group = find_task_group(group_name);
    if(!group) return group.error();

    group.value()->tasks.clear();
    group.value()->sorted_tasks.clear();
    ROS_INFO("成功清空任务组 '%s'", group_name.c_str());
    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置 DIST 排序姿态权重
 * @param group_name 任务组名称
 * @param weight_orient 姿态权重
 * @return 错误码
 */
ErrorCode TasksManager::set_dist_sort_weight_orient(const std::string& group_name, float weight_orient) {
    auto group = find_task_group(group_name);
    if(!group) return group.error();

    if(weight_orient < 0.0f || weight_orient > 1.0f) {
        ROS_WARN("排序权重必须在 [0, 1] 范围内，当前值：%f", weight_orient);
        return ErrorCode::INVALID_PARAMETER;
    }

    group.value()->weight_orient = weight_orient;
    ROS_INFO("成功设置任务组 '%s' 的距离排序姿态权重为 %f", group_name.c_str(), weight_orient);
    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置任务组排序方式
 * @param group_name 任务组名称
 * @param sort_type 排序方式
 * @return 错误码
 */
ErrorCode TasksManager::set_task_group_sort_type(const std::string& group_name, SortType sort_type) {
    auto group = find_task_group(group_name);
    if(!group) return group.error();

    group.value()->sort_type = sort_type;
    ROS_INFO("成功设置任务组 '%s' 的排序方式为 %s",
        group_name.c_str(),
        sort_type == SortType::ID ? "ID" : "DIST");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置任务组完成后是否回到初始位
 * @param group_name 任务组名称
 * @param go_home_after_finish 是否回到初始位
 * @return 错误码
 */
ErrorCode TasksManager::set_task_group_go_home_after_finish(const std::string& group_name, bool go_home_after_finish) {
    auto group = find_task_group(group_name);
    if(!group) return group.error();

    group.value()->go_home_after_finish = go_home_after_finish;
    ROS_INFO("成功设置任务组 '%s' 的完成后回原点选项为 %s",
        group_name.c_str(),
        go_home_after_finish ? "true" : "false");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 添加任务
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @param task_type 任务类型
 * @param task_description 任务描述
 * @return 错误码
 */
ErrorCode TasksManager::add_task(const std::string& group_name, unsigned int id, TaskType task_type, const std::string& task_description) {
    auto group = find_task_group(group_name);
    if(!group) return group.error();

    if(group.value()->tasks.find(id) != group.value()->tasks.end()) {
        ROS_WARN("任务组 '%s' 中已存在任务 ID %u，无法添加", group_name.c_str(), id);
        return ErrorCode::TASK_EXISTS;
    }

    Task new_task;
    new_task.id = id;
    new_task.desc = task_description;
    new_task.type = task_type;
    new_task.target_frame = _arm_->get_base_link();
    group.value()->tasks[id] = std::move(new_task);

    ROS_INFO("成功向任务组 '%s' 添加任务 ID %u", group_name.c_str(), id);
    return ErrorCode::SUCCESS;
}

/**
 * @brief 删除任务
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @return 错误码
 */
ErrorCode TasksManager::delete_task(const std::string& group_name, unsigned int id) {
    auto group = _task_groups_.find(group_name);
    if(group == _task_groups_.end()) return ErrorCode::TASK_GROUP_NOT_FOUND;

    auto task = group->second.tasks.find(id);
    if(task == group->second.tasks.end()) return ErrorCode::TASK_NOT_FOUND;

    group->second.tasks.erase(id);
    ROS_INFO("成功从任务组 '%s' 删除任务 ID %u", group_name.c_str(), id);
    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置任务目标，并冻结到底座坐标系
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @param target 任务目标
 * @param target_frame 任务目标所属坐标系（当 target 不带 frame 信息时生效）
 * @return 错误码
 */
ErrorCode TasksManager::set_task_target(const std::string& group_name, unsigned int id, const tl::optional<TargetVariant>& target, const std::string& target_frame) {
    auto task = find_task(group_name, id);
    if(!task) return task.error();

    tl::optional<TargetVariant> frozen_target;
    std::string frozen_frame;
    const ErrorCode code = freeze_target_to_base(target, target_frame, frozen_target, frozen_frame);
    if(code != ErrorCode::SUCCESS) {
        ROS_WARN("任务组 '%s' 中任务 ID %u 的目标冻结失败，错误码：%s", group_name.c_str(), id, err_to_string(code).c_str());
        return code;
    }

    task.value()->target = frozen_target;
    task.value()->target_frame = frozen_frame;
    if(!frozen_target) {
        ROS_INFO("成功清除任务组 '%s' 中任务 ID %u 的目标", group_name.c_str(), id);
    }
    else {
        ROS_INFO("成功设置任务组 '%s' 中任务 ID %u 的目标，并冻结到 '%s'",
            group_name.c_str(), id, task.value()->target_frame.c_str());
    }
    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置任务采摘参数
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @param pick_params 采摘参数
 * @return 错误码
 */
ErrorCode TasksManager::set_task_pick_params(const std::string& group_name, unsigned int id, const PickTaskParams& pick_params) {
    auto task = find_task(group_name, id);
    if(!task) return task.error();
    if(pick_params.use_place_pose && !pick_params.place_target) {
        ROS_WARN("任务组 '%s' 中任务 ID %u 启用了放置动作，但未设置放置目标", group_name.c_str(), id);
        return ErrorCode::INVALID_PARAMETER;
    }

    PickTaskParams normalized_pick_params = pick_params;
    if(!normalized_pick_params.use_place_pose) {
        normalized_pick_params.place_target = tl::nullopt;
        normalized_pick_params.place_frame = _arm_->get_base_link();
        task.value()->pick_params = normalized_pick_params;
        ROS_INFO("成功设置任务组 '%s' 中任务 ID %u 的采摘参数", group_name.c_str(), id);
        return ErrorCode::SUCCESS;
    }

    tl::optional<TargetVariant> frozen_place_target;
    std::string frozen_place_frame;
    const ErrorCode code = freeze_target_to_base(
        normalized_pick_params.place_target,
        normalized_pick_params.place_frame,
        frozen_place_target,
        frozen_place_frame);
    if(code != ErrorCode::SUCCESS) {
        ROS_WARN("任务组 '%s' 中任务 ID %u 的放置目标冻结失败，错误码：%s",
            group_name.c_str(),
            id,
            err_to_string(code).c_str());
        return code;
    }

    normalized_pick_params.place_target = frozen_place_target;
    normalized_pick_params.place_frame = frozen_place_frame;
    task.value()->pick_params = normalized_pick_params;

    ROS_INFO("成功设置任务组 '%s' 中任务 ID %u 的采摘参数", group_name.c_str(), id);
    return ErrorCode::SUCCESS;
}

/**
 * @brief 执行任务组中的指定任务
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @return 错误码
 */
ErrorCode TasksManager::execute_task(const std::string& group_name, unsigned int id) {
    auto task = find_task(group_name, id);
    if(!task) return task.error();
    return execute_task(*task.value(), nullptr);
}

/**
 * @brief 执行任务组中的指定任务（带上下文）
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @param ctx 执行上下文
 * @return 错误码
 */
ErrorCode TasksManager::execute_task(const std::string& group_name, unsigned int id, ExecutionContext* ctx) {
    auto task = find_task(group_name, id);
    if(!task) return task.error();
    return execute_task(*task.value(), ctx);
}

/**
 * @brief 执行任务对象
 * @param task 任务对象
 * @return 错误码
 */
ErrorCode TasksManager::execute_task(Task& task) {
    return execute_task(task, nullptr);
}

/**
 * @brief 执行任务对象（带上下文）
 * @param task 任务对象
 * @param ctx 执行上下文
 * @return 错误码
 */
ErrorCode TasksManager::execute_task(Task& task, ExecutionContext* ctx) {
    ROS_INFO("开始执行任务 ID %u: %s", task.id, task.desc.c_str());

    if(task.type == TaskType::MOVE_ONLY) {
        if(!task.target) {
            ROS_WARN("任务 ID %u 没有设置目标，无法执行", task.id);
            return ErrorCode::INVALID_PARAMETER;
        }

        TargetVariant target_in_base;
        ErrorCode code = get_frozen_task_target_in_base(task, target_in_base);
        if(code != ErrorCode::SUCCESS) {
            ROS_WARN("执行任务 ID %u 失败，无法获取冻结任务目标，错误码：%s", task.id, err_to_string(code).c_str());
            return code;
        }
        code = _arm_->set_target(target_in_base);
        if(code != ErrorCode::SUCCESS) {
            ROS_WARN("执行任务 ID %u 失败，无法设置目标，错误码：%s", task.id, err_to_string(code).c_str());
            return code;
        }

        const ErrorCode exec_code = _arm_->plan_and_execute();
        if(exec_code != ErrorCode::SUCCESS) {
            ROS_WARN("执行任务 ID %u 失败，无法规划执行，错误码：%s", task.id, err_to_string(exec_code).c_str());
            return exec_code;
        }

        ROS_INFO("成功执行任务 ID %u", task.id);
        return ErrorCode::SUCCESS;
    }

    if(task.type == TaskType::PICK) {
        return execute_pick_task(task, ctx);
    }

    return ErrorCode::INVALID_PARAMETER;
}

/**
 * @brief 执行整个任务组
 * @param group_name 任务组名称
 * @return 错误码
 */
ErrorCode TasksManager::execute_task_group(const std::string& group_name) {
    return execute_task_group(group_name, nullptr);
}

/**
 * @brief 执行整个任务组（带上下文）
 * @param group_name 任务组名称
 * @param ctx 执行上下文
 * @return 错误码
 */
ErrorCode TasksManager::execute_task_group(const std::string& group_name, ExecutionContext* ctx) {
    auto group = find_task_group(group_name);
    if(!group) {
        ROS_WARN("任务组 '%s' 不存在，无法执行", group_name.c_str());
        return ErrorCode::TASK_GROUP_NOT_FOUND;
    }

    ROS_INFO("开始执行任务组 '%s'", group_name.c_str());

    auto is_group_cancel_requested = [&]() -> bool {
        return ctx && ctx->cancel_requested && ctx->cancel_requested->load();
        };

    ErrorCode code = sort_tasks(*group.value());
    if(code != ErrorCode::SUCCESS) {
        ROS_WARN("执行任务组 '%s' 失败，无法排序任务，错误码：%s", group_name.c_str(), err_to_string(code).c_str());
        return code;
    }

    for(const auto& task_snapshot : group.value()->sorted_tasks) {
        if(is_group_cancel_requested()) {
            ROS_INFO("任务组 '%s' 在执行任务 ID %u 前收到取消请求", group_name.c_str(), task_snapshot.id);
            return ErrorCode::CANCELLED;
        }

        auto task = find_task(group_name, task_snapshot.id);
        if(!task) {
            ROS_WARN("执行任务组 '%s' 失败，任务 ID %u 已不存在", group_name.c_str(), task_snapshot.id);
            return task.error();
        }

        const ErrorCode err_code = execute_task(*task.value(), ctx);
        if(err_code != ErrorCode::SUCCESS) {
            ROS_WARN("执行任务组 '%s' 中任务 ID %u 失败，错误码：%s",
                group_name.c_str(),
                task_snapshot.id,
                err_to_string(err_code).c_str());
            return err_code;
        }
    }

    if(group.value()->go_home_after_finish) {
        if(is_group_cancel_requested()) {
            ROS_INFO("任务组 '%s' 在收尾回零前收到取消请求", group_name.c_str());
            return ErrorCode::CANCELLED;
        }

        ROS_INFO("任务组 '%s' 执行完成，回到初始位", group_name.c_str());

        Task feedback_task;
        feedback_task.id = 0;
        feedback_task.desc = "任务组收尾";
        report_pick_feedback(feedback_task, PickStage::PICK_GO_HOME, false, ErrorCode::SUCCESS, "任务组完成后回到初始位", ctx);

        code = _arm_->home();
        if(code != ErrorCode::SUCCESS) {
            report_pick_feedback(feedback_task, PickStage::PICK_GO_HOME, false, code, "任务组完成后回到初始位失败", ctx);
            ROS_WARN("任务组 '%s' 执行完成，但回到初始位失败，错误码：%s", group_name.c_str(), err_to_string(code).c_str());
            return code;
        }

        auto gripper_if = find_eef_interface<GripperEefInterface>(*_eef_);
        if(!gripper_if) ROS_WARN("末端执行器不支持夹爪接口，无法执行任务组完成后的夹爪复位");
        else gripper_if.value().close();

        report_pick_feedback(feedback_task, PickStage::PICK_GO_HOME, true, ErrorCode::SUCCESS, "任务组完成后回到初始位：完成", ctx);
    }

    return ErrorCode::SUCCESS;
}

/**
 * @brief 估算任务组总步骤数
 * @param group_name 任务组名称
 * @return 预计步骤数量
 */
uint32_t TasksManager::estimate_task_group_steps(const std::string& group_name) const {
    auto group_it = _task_groups_.find(group_name);
    if(group_it == _task_groups_.end()) return 0;

    uint32_t total_steps = 0;
    for(const auto& [id, task] : group_it->second.tasks) {
        (void)id;
        total_steps += estimate_task_steps(task);
    }
    if(group_it->second.go_home_after_finish) total_steps += 1;
    return total_steps;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 查找任务组对象
 * @param group_name 任务组名称
 * @return 任务组指针，失败返回错误码
 */
tl::expected<TaskGroup*, ErrorCode> TasksManager::find_task_group(const std::string& group_name) {
    auto task_group = _task_groups_.find(group_name);
    if(task_group == _task_groups_.end()) {
        ROS_WARN("任务组 '%s' 不存在", group_name.c_str());
        return tl::make_unexpected(ErrorCode::TASK_GROUP_NOT_FOUND);
    }
    return &task_group->second;
}

/**
 * @brief 查找任务对象
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @return 任务指针，失败返回错误码
 */
tl::expected<Task*, ErrorCode> TasksManager::find_task(const std::string& group_name, unsigned int id) {
    auto group = find_task_group(group_name);
    if(!group) return tl::make_unexpected(group.error());

    auto task = group.value()->tasks.find(id);
    if(task == group.value()->tasks.end()) {
        ROS_WARN("任务组 '%s' 中不存在任务 ID %u", group_name.c_str(), id);
        return tl::make_unexpected(ErrorCode::TASK_NOT_FOUND);
    }
    return &task->second;
}

/**
 * @brief 查找任务对象（const 版本）
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @return 任务指针，失败返回错误码
 */
tl::expected<const Task*, ErrorCode> TasksManager::find_task(const std::string& group_name, unsigned int id) const {
    auto task_group = _task_groups_.find(group_name);
    if(task_group == _task_groups_.end()) {
        ROS_WARN("任务组 '%s' 不存在", group_name.c_str());
        return tl::make_unexpected(ErrorCode::TASK_GROUP_NOT_FOUND);
    }

    auto task = task_group->second.tasks.find(id);
    if(task == task_group->second.tasks.end()) {
        ROS_WARN("任务组 '%s' 中不存在任务 ID %u", group_name.c_str(), id);
        return tl::make_unexpected(ErrorCode::TASK_NOT_FOUND);
    }

    return &task->second;
}

/**
 * @brief 获取任务目标在底座坐标系下的冻结目标
 * @param task 任务对象
 * @param target_in_base 输出目标（底座坐标系）
 * @return 错误码
 */
ErrorCode TasksManager::get_frozen_task_target_in_base(const Task& task, TargetVariant& target_in_base) const {
    if(!task.target) return ErrorCode::INVALID_PARAMETER;
    if(task.target_frame != _arm_->get_base_link()) {
        ROS_WARN("任务 ID %u 的目标坐标系 '%s' 与机械臂底座坐标系 '%s' 不一致，无法获取冻结目标",
            task.id, task.target_frame.c_str(), _arm_->get_base_link().c_str());
        return ErrorCode::INVALID_PARAMETER;
    }
    target_in_base = task.target.value();
    return ErrorCode::SUCCESS;
}

/**
 * @brief 获取采摘任务放置目标在底座坐标系下的冻结目标
 * @param task 任务对象
 * @param place_in_base 输出放置目标（底座坐标系）
 * @return 错误码
 */
ErrorCode TasksManager::get_frozen_place_target_in_base(const Task& task, TargetVariant& place_in_base) const {
    if(!task.pick_params || !task.pick_params->place_target) return ErrorCode::INVALID_PARAMETER;
    if(task.pick_params->place_frame != _arm_->get_base_link()) {
        ROS_WARN("任务 ID %u 的放置目标坐标系 '%s' 与机械臂底座坐标系 '%s' 不一致，无法获取冻结放置目标",
            task.id, task.pick_params->place_frame.c_str(), _arm_->get_base_link().c_str());
        return ErrorCode::INVALID_PARAMETER;
    }
    place_in_base = task.pick_params->place_target.value();
    return ErrorCode::SUCCESS;
}

/**
 * @brief 对任务组进行排序
 * @param task_group 任务组对象
 * @return 错误码
 */
ErrorCode TasksManager::sort_tasks(TaskGroup& task_group) {
    task_group.sorted_tasks.clear();

    if(task_group.sort_type == SortType::ID) {
        for(auto& [id, task] : task_group.tasks) {
            task_group.sorted_tasks.push_back(task);
            task_group.sorted_tasks.back().id = id;
        }
        ROS_INFO("任务组已按 ID 排序");
        return ErrorCode::SUCCESS;
    }

    if(task_group.sort_type != SortType::DIST) {
        return ErrorCode::SUCCESS;
    }

    std::vector<unsigned int> sortable_ids;
    std::vector<unsigned int> no_target_ids;

    for(auto& [id, task] : task_group.tasks) {
        if(task.target) sortable_ids.push_back(id);
        else no_target_ids.push_back(id);
    }

    if(sortable_ids.empty()) {
        ROS_WARN("任务组中没有可排序的任务，已按 ID 顺序执行");
        for(auto& [id, task] : task_group.tasks) {
            task_group.sorted_tasks.push_back(task);
            task_group.sorted_tasks.back().id = id;
        }
        return ErrorCode::SUCCESS;
    }

    if(!no_target_ids.empty()) {
        ROS_WARN("任务组中存在 %zu 个没有目标的任务，已自动排在末尾", no_target_ids.size());
        for(const unsigned int id : no_target_ids) {
            task_group.sorted_tasks.push_back(task_group.tasks.at(id));
            task_group.sorted_tasks.back().id = id;
        }
    }

    std::set<unsigned int> visited_ids;

    const geometry_msgs::Pose current_pose = _arm_->get_current_pose();
    unsigned int cur_id = sortable_ids.front();
    double min_dist = -1.0;
    for(const unsigned int id : sortable_ids) {
        TargetVariant target_in_base;
        const ErrorCode code = get_frozen_task_target_in_base(task_group.tasks.at(id), target_in_base);
        if(code != ErrorCode::SUCCESS) return code;

        const geometry_msgs::Pose target_pose = target_to_effective_pose_in_base(target_in_base, current_pose);
        const double pos_dist = std::sqrt(
            std::pow(current_pose.position.x - target_pose.position.x, 2) +
            std::pow(current_pose.position.y - target_pose.position.y, 2) +
            std::pow(current_pose.position.z - target_pose.position.z, 2));
        if(min_dist < 0.0 || pos_dist < min_dist) {
            min_dist = pos_dist;
            cur_id = id;
        }
    }

    while(task_group.sorted_tasks.size() < sortable_ids.size()) {
        const auto cur_task_it = task_group.tasks.find(cur_id);
        if(cur_task_it == task_group.tasks.end()) return ErrorCode::TASK_NOT_FOUND;

        task_group.sorted_tasks.push_back(cur_task_it->second);
        task_group.sorted_tasks.back().id = cur_id;
        visited_ids.insert(cur_id);

        if(visited_ids.size() == sortable_ids.size()) break;

        TargetVariant current_target_in_base;
        ErrorCode code = get_frozen_task_target_in_base(cur_task_it->second, current_target_in_base);
        if(code != ErrorCode::SUCCESS) return code;

        double min_dist_local = -1.0;
        for(const unsigned int id : sortable_ids) {
            if(visited_ids.count(id) > 0) continue;

            TargetVariant candidate_target_in_base;
            code = get_frozen_task_target_in_base(task_group.tasks.at(id), candidate_target_in_base);
            if(code != ErrorCode::SUCCESS) return code;

            const double dist = calculate_dist(current_target_in_base, candidate_target_in_base, task_group.weight_orient);
            if(min_dist_local < 0.0 || dist < min_dist_local) {
                min_dist_local = dist;
                cur_id = id;
            }
        }
    }

    optimize_with_2opt(task_group.sorted_tasks, task_group.weight_orient);
    ROS_INFO("任务组已按加权距离排序");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 计算两个目标之间的加权距离
 * @param base 基准目标（已在底座坐标系）
 * @param target 目标（已在底座坐标系）
 * @param weight_orient 姿态权重
 * @return 加权距离
 */
double TasksManager::calculate_dist(const TargetVariant& base, const TargetVariant& target, float weight_orient) {
    const geometry_msgs::Pose ref_pose = _arm_->get_current_pose();
    const geometry_msgs::Pose base_pose = target_to_effective_pose_in_base(base, ref_pose);
    const geometry_msgs::Pose target_pose = target_to_effective_pose_in_base(target, base_pose);

    auto quat_angle_distance = [](const geometry_msgs::Quaternion& a, const geometry_msgs::Quaternion& b) -> double {
        const double dot =
            a.x * b.x +
            a.y * b.y +
            a.z * b.z +
            a.w * b.w;

        const double dot_abs = std::min(1.0, std::max(0.0, std::fabs(dot)));
        return 2.0 * std::acos(dot_abs);
        };

    const double pos_dist = std::sqrt(
        std::pow(base_pose.position.x - target_pose.position.x, 2) +
        std::pow(base_pose.position.y - target_pose.position.y, 2) +
        std::pow(base_pose.position.z - target_pose.position.z, 2));

    const double orient_dist = quat_angle_distance(base_pose.orientation, target_pose.orientation);

    return (1.0 - weight_orient) * pos_dist + weight_orient * orient_dist;
}

/**
 * @brief 使用 2-opt 算法优化任务路径
 * @param path 任务路径
 * @param weight_orient 姿态权重
 */
void TasksManager::optimize_with_2opt(std::vector<Task>& path, float weight_orient) {
    bool improved = true;
    const int n = static_cast<int>(path.size());
    if(n < 4) return;

    while(improved) {
        improved = false;
        for(int i = 0; i < n - 3; ++i) {
            for(int j = i + 2; j < n - 1; ++j) {
                TargetVariant base_i, base_i1, base_j, base_j1;
                if(get_frozen_task_target_in_base(path[i], base_i) != ErrorCode::SUCCESS) continue;
                if(get_frozen_task_target_in_base(path[i + 1], base_i1) != ErrorCode::SUCCESS) continue;
                if(get_frozen_task_target_in_base(path[j], base_j) != ErrorCode::SUCCESS) continue;
                if(get_frozen_task_target_in_base(path[j + 1], base_j1) != ErrorCode::SUCCESS) continue;

                const double dist1 = calculate_dist(base_i, base_i1, weight_orient);
                const double dist2 = calculate_dist(base_j, base_j1, weight_orient);
                const double dist3 = calculate_dist(base_i, base_j, weight_orient);
                const double dist4 = calculate_dist(base_i1, base_j1, weight_orient);

                if(dist3 + dist4 < dist1 + dist2) {
                    std::reverse(path.begin() + i + 1, path.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
}

/**
 * @brief 判断任务是否收到取消请求
 * @param task 任务对象
 * @param ctx 执行上下文
 * @return 若已取消则返回 true
 */
bool TasksManager::is_cancel_requested(Task& task, ExecutionContext* ctx) {
    if(task.pick_params && task.pick_params->canceled) return true;
    if(ctx && ctx->cancel_requested && ctx->cancel_requested->load()) return true;
    return false;
}

/**
 * @brief 上报采摘任务反馈
 * @param task 任务对象
 * @param stage 当前阶段
 * @param ok 最近一次子步骤是否成功
 * @param code 最近一次子步骤返回码
 * @param text 反馈文本
 * @param ctx 执行上下文
 */
void TasksManager::report_pick_feedback(Task& task, PickStage stage, bool ok, ErrorCode code, const std::string& text, ExecutionContext* ctx) {
    if(task.pick_params) {
        task.pick_params->current_stage = stage;
    }

    if(ctx && ctx->feedback_cb) {
        ctx->feedback_cb(task, stage, ok, code, text);
    }
}

/**
 * @brief 将目标冻结到底座坐标系下
 * @param target 输入目标
 * @param target_frame 输入目标所属坐标系（当目标不带 frame 信息时必须显式提供）
 * @param frozen_target 输出冻结后的目标
 * @param frozen_frame 输出冻结后的坐标系
 * @return 错误码
 */
ErrorCode TasksManager::freeze_target_to_base(const TargetVariant& target,
    const std::string& target_frame,
    TargetVariant& frozen_target,
    std::string& frozen_frame) const {

    std::string source_frame;
    const ErrorCode code = std::visit(variant_visitor{
        [&](const std::monostate&) -> ErrorCode {
            ROS_WARN("目标为空，无法冻结到底座坐标系");
            return ErrorCode::INVALID_TARGET_TYPE;
        },
        [&](const geometry_msgs::Pose& pose) -> ErrorCode {
            if(target_frame.empty()) {
                ROS_WARN("Pose 目标必须显式提供 frame_id，无法冻结到底座坐标系");
                return ErrorCode::INVALID_PARAMETER;
            }
            source_frame = target_frame;
            return _arm_->resolve_target_to_base(pose, target_frame, frozen_target, nullptr);
        },
        [&](const geometry_msgs::Point& point) -> ErrorCode {
            if(target_frame.empty()) {
                ROS_WARN("Point 目标必须显式提供 frame_id，无法冻结到底座坐标系");
                return ErrorCode::INVALID_PARAMETER;
            }
            source_frame = target_frame;
            return _arm_->resolve_target_to_base(point, target_frame, frozen_target, nullptr);
        },
        [&](const geometry_msgs::Quaternion& quat) -> ErrorCode {
            if(target_frame.empty()) {
                ROS_WARN("Quaternion 目标必须显式提供 frame_id，无法冻结到底座坐标系");
                return ErrorCode::INVALID_PARAMETER;
            }
            source_frame = target_frame;
            return _arm_->resolve_target_to_base(quat, target_frame, frozen_target, nullptr);
        },
        [&](const geometry_msgs::PoseStamped& pose_stamped) -> ErrorCode {
            if(pose_stamped.header.frame_id.empty()) {
                ROS_WARN("PoseStamped 目标必须显式提供 header.frame_id，无法冻结到底座坐标系");
                return ErrorCode::INVALID_PARAMETER;
            }
            source_frame = pose_stamped.header.frame_id;
            return _arm_->resolve_target_to_base(pose_stamped, pose_stamped.header.frame_id, frozen_target, nullptr);
        }
        }, target);

    if(code != ErrorCode::SUCCESS) return code;

    frozen_frame = _arm_->get_base_link();
    ROS_INFO("目标已从 '%s' 冻结到 '%s'", source_frame.c_str(), frozen_frame.c_str());
    return ErrorCode::SUCCESS;
}

/**
 * @brief 将可选目标冻结到底座坐标系下
 * @param target 输入目标
 * @param target_frame 输入目标所属坐标系（当目标不带 frame 信息时必须显式提供）
 * @param frozen_target 输出冻结后的目标
 * @param frozen_frame 输出冻结后的坐标系
 * @return 错误码
 */
ErrorCode TasksManager::freeze_target_to_base(const tl::optional<TargetVariant>& target,
    const std::string& target_frame,
    tl::optional<TargetVariant>& frozen_target,
    std::string& frozen_frame) const {
    frozen_frame = _arm_->get_base_link();
    if(!target) {
        frozen_target = tl::nullopt;
        return ErrorCode::SUCCESS;
    }

    TargetVariant frozen_target_value;
    const ErrorCode code = freeze_target_to_base(target.value(), target_frame, frozen_target_value, frozen_frame);
    if(code != ErrorCode::SUCCESS) return code;

    frozen_target = frozen_target_value;
    return ErrorCode::SUCCESS;
}

/**
 * @brief 估算单个任务的步骤数量
 * @param task 任务对象
 * @return 预计步骤数量
 */
uint32_t TasksManager::estimate_task_steps(const Task& task) const {
    if(task.type == TaskType::MOVE_ONLY) return 1;

    if(task.type == TaskType::PICK) {
        uint32_t steps = 4;
        if(task.pick_params && task.pick_params->use_place_pose) steps += 2;
        return steps;
    }

    return 0;
}

/**
 * @brief 执行采摘任务
 * @param task 任务对象
 * @return 错误码
 */
ErrorCode TasksManager::execute_pick_task(Task& task) {
    return execute_pick_task(task, nullptr);
}

/**
 * @brief 执行采摘任务（带上下文）
 * @param task 任务对象
 * @param ctx 执行上下文
 * @return 错误码
 */
ErrorCode TasksManager::execute_pick_task(Task& task, ExecutionContext* ctx) {
    ROS_INFO("正在执行 PICK 任务（ID = %u）...", task.id);

    if(!task.target) {
        ROS_WARN("任务 ID %u 没有设置目标，无法执行 PICK 任务", task.id);
        return ErrorCode::INVALID_PARAMETER;
    }
    if(!task.pick_params) {
        ROS_WARN("任务 ID %u 没有设置采摘参数，无法执行 PICK 任务", task.id);
        return ErrorCode::INVALID_PARAMETER;
    }

    auto& pp = *task.pick_params;
    pp.completed = false;
    pp.canceled = false;
    pp.current_stage = PickStage::PICK_IDLE;

    auto check_cancel = [&]() -> ErrorCode {
        if(!is_cancel_requested(task, ctx)) return ErrorCode::SUCCESS;

        pp.canceled = true;
        pp.completed = false;
        report_pick_feedback(task, PickStage::PICK_CANCELED, false, ErrorCode::CANCELLED, "任务已取消", ctx);

        if(pp.go_safe_after_cancel) {
            const ErrorCode safe_code = _arm_->home();
            if(safe_code != ErrorCode::SUCCESS) {
                ROS_WARN("任务 ID %u 取消后回安全位失败，错误码：%s", task.id, err_to_string(safe_code).c_str());
            }
        }
        return ErrorCode::CANCELLED;
        };

    auto fail_stage = [&](PickStage stage, ErrorCode ec, const std::string& text) -> ErrorCode {
        pp.current_stage = PickStage::PICK_FAILED;
        pp.completed = false;
        report_pick_feedback(task, PickStage::PICK_FAILED, false, ec, text, ctx);
        ROS_WARN("任务 ID %u 在阶段[%s]失败：%s (%s)", task.id, pick_stage_to_string(stage), text.c_str(), err_to_string(ec).c_str());
        return ec;
        };

    auto run_with_retry = [&](const std::function<ErrorCode()>& fn, PickStage stage) -> ErrorCode {
        ErrorCode last = ErrorCode::FAILURE;
        const uint16_t total_attempts = static_cast<uint16_t>(pp.retry_times) + 1;
        for(uint16_t attempt = 1; attempt <= total_attempts; ++attempt) {
            last = fn();
            if(last == ErrorCode::SUCCESS) return ErrorCode::SUCCESS;
            if(last == ErrorCode::CANCELLED) return last;
            ROS_WARN("任务 ID %u 阶段[%s] 第 %u/%u 次尝试失败：%s", task.id, pick_stage_to_string(stage), static_cast<unsigned int>(attempt), static_cast<unsigned int>(total_attempts), err_to_string(last).c_str());
        }
        return last;
        };

    auto exec_move = [&](const TargetVariant& target,
        PickStage stage,
        const std::string& text) -> ErrorCode {
            report_pick_feedback(task, stage, false, ErrorCode::SUCCESS, text, ctx);
            ErrorCode ec = _arm_->set_target(target);
            if(ec != ErrorCode::SUCCESS) return fail_stage(stage, ec, text + "：设置目标失败");
            ec = _arm_->plan_and_execute();
            if(ec != ErrorCode::SUCCESS) return fail_stage(stage, ec, text + "：执行失败");
            report_pick_feedback(task, stage, true, ErrorCode::SUCCESS, text + "：完成", ctx);
            return ErrorCode::SUCCESS;
        };

    auto exec_eef = [&](bool open, PickStage stage, const std::string& text) -> ErrorCode {
        if(!_eef_) return fail_stage(stage, ErrorCode::INVALID_INTERFACE, text + "：末端执行器未初始化");
        auto gripper_if = find_eef_interface<GripperEefInterface>(*_eef_);
        if(!gripper_if) return fail_stage(stage, ErrorCode::INVALID_INTERFACE, text + "：末端执行器不支持夹爪接口");

        report_pick_feedback(task, stage, false, ErrorCode::SUCCESS, text, ctx);
        ErrorCode ec = open ? gripper_if.value().open() : gripper_if.value().close();
        if(ec != ErrorCode::SUCCESS) return fail_stage(stage, ec, text + "：执行失败");
        report_pick_feedback(task, stage, true, ErrorCode::SUCCESS, text + "：完成", ctx);
        return ErrorCode::SUCCESS;
        };

    if(!_eef_) return fail_stage(PickStage::PICK_START, ErrorCode::INVALID_INTERFACE, "末端执行器未初始化，无法执行采摘任务");
    auto gripper_if = find_eef_interface<GripperEefInterface>(*_eef_);
    if(!gripper_if) return fail_stage(PickStage::PICK_START, ErrorCode::INVALID_INTERFACE, "末端执行器不支持夹爪接口，无法执行采摘任务");
    gripper_if.value().open();
    report_pick_feedback(task, PickStage::PICK_START, true, ErrorCode::SUCCESS, "开始", ctx);

    ErrorCode code = check_cancel();
    if(code != ErrorCode::SUCCESS) return code;

    TargetVariant target_in_base;
    code = get_frozen_task_target_in_base(task, target_in_base);
    if(code != ErrorCode::SUCCESS) return fail_stage(PickStage::PICK_MOVE_TO_PICK, code, "获取冻结目标失败");
    code = run_with_retry([&]() { return exec_move(target_in_base, PickStage::PICK_MOVE_TO_PICK, "移动到采摘位"); }, PickStage::PICK_MOVE_TO_PICK);
    if(code != ErrorCode::SUCCESS) return code;

    code = check_cancel();
    if(code != ErrorCode::SUCCESS) return code;

    if(pp.use_eef) {
        code = run_with_retry([&]() { return exec_eef(false, PickStage::PICK_PICKING, "采摘中"); }, PickStage::PICK_PICKING);
        if(code != ErrorCode::SUCCESS) return code;
    }
    else {
        report_pick_feedback(task, PickStage::PICK_PICKING, true, ErrorCode::SUCCESS, "采摘中：跳过末端动作", ctx);
    }

    code = check_cancel();
    if(code != ErrorCode::SUCCESS) return code;

    if(pp.use_place_pose && pp.place_target) {
        TargetVariant place_target_in_base;
        code = get_frozen_place_target_in_base(task, place_target_in_base);
        if(code != ErrorCode::SUCCESS) return fail_stage(PickStage::PICK_MOVE_TO_PLACE, code, "获取冻结放置目标失败");
        code = run_with_retry([&]() { return exec_move(place_target_in_base, PickStage::PICK_MOVE_TO_PLACE, "移动到放置位"); }, PickStage::PICK_MOVE_TO_PLACE);
        if(code != ErrorCode::SUCCESS) return code;

        code = check_cancel();
        if(code != ErrorCode::SUCCESS) return code;

        if(pp.use_eef) {
            code = run_with_retry([&]() { return exec_eef(true, PickStage::PICK_PLACING, "放置中"); }, PickStage::PICK_PLACING);
            if(code != ErrorCode::SUCCESS) return code;
        }
        else {
            report_pick_feedback(task, PickStage::PICK_PLACING, true, ErrorCode::SUCCESS, "放置中：跳过末端动作", ctx);
        }
    }

    pp.completed = true;
    report_pick_feedback(task, PickStage::PICK_FINISH, true, ErrorCode::SUCCESS, "完成", ctx);

    return ErrorCode::SUCCESS;
}

} /* namespace piper */
