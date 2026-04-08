#include "piper_task/tasks_manager.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

#include <ros/ros.h>

namespace piper {

namespace {

#define X(name, desc) case PickStage::PICK_##name: return desc;
const char* pick_stage_to_string(PickStage stage) {
    switch(stage) {
        PICK_STAGE_TABLE
        default: return "未知阶段";
    }
}
#undef X

tl::optional<geometry_msgs::PoseStamped> to_pose_stamped(const TargetVariant& target) {
    return std::visit(variant_visitor{
        [](const std::monostate&) -> tl::optional<geometry_msgs::PoseStamped> {
            return tl::nullopt;
        },
        [](const geometry_msgs::Pose& pose) -> tl::optional<geometry_msgs::PoseStamped> {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "base_link";
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.pose = pose;
            return pose_stamped;
        },
        [](const geometry_msgs::Point&) -> tl::optional<geometry_msgs::PoseStamped> {
            return tl::nullopt;
        },
        [](const geometry_msgs::Quaternion&) -> tl::optional<geometry_msgs::PoseStamped> {
            return tl::nullopt;
        },
        [](const geometry_msgs::PoseStamped& pose_stamped) -> tl::optional<geometry_msgs::PoseStamped> {
            return pose_stamped;
        }
        }, target);
}

}  // namespace

TasksManager::TasksManager(std::shared_ptr<ArmController> arm, std::shared_ptr<EndEffector> eef)
    : _arm_(std::move(arm)), _eef_(std::move(eef)) {
    if(!_arm_) {
        ROS_ERROR("机械臂控制器不能为空，TasksManager 初始化失败");
        throw std::invalid_argument("ArmController pointer cannot be null");
    }

    _arm_name_ = _arm_->get_arm_name();
    if(_eef_) _eef_name_ = _eef_->get_eef_name();
}

ErrorCode TasksManager::create_task_group(const std::string& group_name, SortType sort_type) {
    auto group = find_task_group(group_name);
    if(group) {
        ROS_WARN("任务组 '%s' 已存在，无法创建", group_name.c_str());
        return ErrorCode::TASK_GROUP_EXISTS;
    }

    TaskGroup new_group;
    new_group.sort_type = sort_type;
    _task_groups_[group_name] = std::move(new_group);
    ROS_INFO("成功创建任务组 '%s'", group_name.c_str());
    return ErrorCode::SUCCESS;
}

ErrorCode TasksManager::delete_task_group(const std::string& group_name) {
    auto group = find_task_group(group_name);
    if(!group) return group.error();

    _task_groups_.erase(group_name);
    ROS_INFO("成功删除任务组 '%s'", group_name.c_str());
    return ErrorCode::SUCCESS;
}

ErrorCode TasksManager::clear_task_group(const std::string& group_name) {
    auto group = find_task_group(group_name);
    if(!group) return group.error();

    group.value()->tasks.clear();
    group.value()->sorted_tasks.clear();
    ROS_INFO("成功清空任务组 '%s'", group_name.c_str());
    return ErrorCode::SUCCESS;
}

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
    group.value()->tasks[id] = std::move(new_task);

    ROS_INFO("成功向任务组 '%s' 添加任务 ID %u", group_name.c_str(), id);
    return ErrorCode::SUCCESS;
}

ErrorCode TasksManager::delete_task(const std::string& group_name, unsigned int id) {
    auto group = find_task_group(group_name);
    if(!group) return group.error();
    auto task = find_task(group_name, id);
    if(!task) return task.error();

    group.value()->tasks.erase(id);
    ROS_INFO("成功从任务组 '%s' 删除任务 ID %u", group_name.c_str(), id);
    return ErrorCode::SUCCESS;
}

ErrorCode TasksManager::set_task_target(const std::string& group_name, unsigned int id, const tl::optional<TargetVariant>& target) {
    auto task = find_task(group_name, id);
    if(!task) return task.error();

    task.value()->raw_target = target;
    if(!target) {
        task.value()->target = tl::nullopt;
        if(task.value()->pick_params) {
            task.value()->pick_params->resolved_pick_pose = tl::nullopt;
            task.value()->pick_params->resolved_pre_pick_pose = tl::nullopt;
            task.value()->pick_params->resolved_retreat_pose = tl::nullopt;
            task.value()->pick_params->target_source_frame.clear();
        }
        ROS_INFO("成功清除任务组 '%s' 中任务 ID %u 的目标", group_name.c_str(), id);
        return ErrorCode::SUCCESS;
    }

    TargetVariant resolved;
    ErrorCode code = resolve_target_to_base(target.value(), resolved,
        task.value()->pick_params ? &task.value()->pick_params->target_source_frame : nullptr);
    if(code != ErrorCode::SUCCESS) {
        ROS_WARN("任务组 '%s' 中任务 ID %u 目标解析失败，错误码：%s", group_name.c_str(), id, err_to_string(code).c_str());
        return code;
    }

    task.value()->target = resolved;

    if(task.value()->type == TaskType::PICK && task.value()->pick_params) {
        code = rebuild_pick_task_cache(*task.value());
        if(code != ErrorCode::SUCCESS) return code;
    }

    ROS_INFO("成功设置任务组 '%s' 中任务 ID %u 的目标（已冻结到 base frame）", group_name.c_str(), id);
    return ErrorCode::SUCCESS;
}

ErrorCode TasksManager::set_task_pick_params(const std::string& group_name, unsigned int id, const PickTaskParams& pick_params) {
    auto task = find_task(group_name, id);
    if(!task) return task.error();

    task.value()->pick_params = pick_params;
    if(!task.value()->raw_target && task.value()->target) {
        task.value()->raw_target = task.value()->target;
    }

    if(task.value()->type == TaskType::PICK) {
        ErrorCode code = rebuild_pick_task_cache(*task.value());
        if(code != ErrorCode::SUCCESS) return code;
    }

    ROS_INFO("成功设置任务组 '%s' 中任务 ID %u 的采摘参数", group_name.c_str(), id);
    return ErrorCode::SUCCESS;
}

ErrorCode TasksManager::execute_task(const std::string& group_name, unsigned int id) {
    auto task = find_task(group_name, id);
    if(!task) return task.error();
    return execute_task(*task.value(), nullptr);
}

ErrorCode TasksManager::execute_task(const std::string& group_name, unsigned int id, ExecutionContext* ctx) {
    auto task = find_task(group_name, id);
    if(!task) return task.error();
    return execute_task(*task.value(), ctx);
}

ErrorCode TasksManager::execute_task(Task& task) {
    return execute_task(task, nullptr);
}

ErrorCode TasksManager::execute_task(Task& task, ExecutionContext* ctx) {
    ROS_INFO("开始执行任务 ID %u: %s", task.id, task.desc.c_str());

    if(task.type == TaskType::MOVE_ONLY) {
        if(!task.target) {
            ROS_WARN("任务 ID %u 没有设置目标，无法执行", task.id);
            return ErrorCode::INVALID_PARAMETER;
        }

        ErrorCode code = _arm_->set_target(task.target.value());
        if(code != ErrorCode::SUCCESS) {
            ROS_WARN("执行任务 ID %u 失败，无法设置目标，错误码：%s", task.id, err_to_string(code).c_str());
            return code;
        }

        code = _arm_->plan_and_execute();
        if(code != ErrorCode::SUCCESS) {
            ROS_WARN("执行任务 ID %u 失败，无法规划执行，错误码：%s", task.id, err_to_string(code).c_str());
            return code;
        }

        ROS_INFO("成功执行任务 ID %u", task.id);
        return ErrorCode::SUCCESS;
    }

    if(task.type == TaskType::PICK) {
        return execute_pick_task(task, ctx);
    }

    return ErrorCode::INVALID_PARAMETER;
}

ErrorCode TasksManager::execute_task_group(const std::string& group_name) {
    return execute_task_group(group_name, nullptr);
}

ErrorCode TasksManager::execute_task_group(const std::string& group_name, ExecutionContext* ctx) {
    auto group = find_task_group(group_name);
    if(!group) {
        ROS_WARN("任务组 '%s' 不存在，无法执行", group_name.c_str());
        return ErrorCode::TASK_GROUP_NOT_FOUND;
    }

    ROS_INFO("开始执行任务组 '%s'", group_name.c_str());

    ErrorCode code = sort_tasks(*group.value());
    if(code != ErrorCode::SUCCESS) {
        ROS_WARN("执行任务组 '%s' 失败，无法排序任务，错误码：%s", group_name.c_str(), err_to_string(code).c_str());
        return code;
    }

    for(auto& task : group.value()->sorted_tasks) {
        ErrorCode err_code = execute_task(task, ctx);
        if(err_code != ErrorCode::SUCCESS) {
            ROS_WARN("执行任务组 '%s' 中任务 ID %u 失败，错误码：%s", group_name.c_str(), task.id, err_to_string(err_code).c_str());
            return err_code;
        }
    }

    return ErrorCode::SUCCESS;
}

// ========================= private ========================= //

tl::expected<TaskGroup*, ErrorCode> TasksManager::find_task_group(const std::string& group_name) {
    auto task_group = _task_groups_.find(group_name);
    if(task_group == _task_groups_.end()) {
        ROS_WARN("任务组 '%s' 不存在", group_name.c_str());
        return tl::make_unexpected(ErrorCode::TASK_GROUP_NOT_FOUND);
    }
    return &task_group->second;
}

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

    if(task_group.sort_type == SortType::DIST) {
        std::vector<std::reference_wrapper<const Task>> sortable_tasks;
        for(auto& [id, task] : task_group.tasks) {
            if(task.target) sortable_tasks.push_back(std::cref(task));
        }

        if(sortable_tasks.empty()) {
            ROS_WARN("任务组中没有设置目标的任务，无法按距离排序，默认按 ID 排序");
            for(auto& [id, task] : task_group.tasks) {
                task_group.sorted_tasks.push_back(task);
                task_group.sorted_tasks.back().id = id;
            }
            return ErrorCode::SUCCESS;
        }

        std::set<unsigned int> visited_ids;
        unsigned int cur_id = 0;
        double min_dist = -1.0;
        for(auto& [id, task] : task_group.tasks) {
            if(!task.target) continue;
            double dist = calculate_dist(_arm_->get_current_pose(), task.target.value(), task_group.weight_orient);
            if(min_dist < 0 || dist < min_dist) {
                min_dist = dist;
                cur_id = id;
            }
        }

        while(task_group.sorted_tasks.size() < sortable_tasks.size()) {
            auto cur_task = task_group.tasks.find(cur_id);
            task_group.sorted_tasks.push_back(cur_task->second);
            task_group.sorted_tasks.back().id = cur_id;
            visited_ids.insert(cur_id);

            double min_dist_local = -1.0;
            bool found_next = false;
            for(const auto& [id, task] : task_group.tasks) {
                if(visited_ids.count(id) > 0 || !task.target) continue;
                double dist = calculate_dist(task_group.sorted_tasks.back().target.value(), task.target.value(), task_group.weight_orient);
                if(min_dist_local < 0 || dist < min_dist_local) {
                    min_dist_local = dist;
                    cur_id = id;
                    found_next = true;
                }
            }
            if(!found_next) break;
        }

        optimize_with_2opt(task_group.sorted_tasks, task_group.weight_orient);
        ROS_INFO("任务组已按加权距离排序");
        return ErrorCode::SUCCESS;
    }

    return ErrorCode::SUCCESS;
}

double TasksManager::calculate_dist(const TargetVariant& base, const TargetVariant& target, float weight_orient) {
    auto get_position = variant_visitor{
        [](std::monostate) {
            geometry_msgs::Point zero_point;
            zero_point.x = 0.0;
            zero_point.y = 0.0;
            zero_point.z = 0.0;
            return zero_point;
        },
        [](const geometry_msgs::Pose& pose) { return pose.position; },
        [](const geometry_msgs::Point& point) { return point; },
        [](const geometry_msgs::Quaternion&) {
            geometry_msgs::Point zero_point;
            zero_point.x = 0.0;
            zero_point.y = 0.0;
            zero_point.z = 0.0;
            return zero_point;
        },
        [](const geometry_msgs::PoseStamped& pose_stamped) { return pose_stamped.pose.position; }
    };

    geometry_msgs::Point base_pos = std::visit(get_position, base);
    geometry_msgs::Point target_pos = std::visit(get_position, target);
    const double pos_dist = std::sqrt(
        std::pow(base_pos.x - target_pos.x, 2) +
        std::pow(base_pos.y - target_pos.y, 2) +
        std::pow(base_pos.z - target_pos.z, 2));

    auto get_orientation = variant_visitor{
        [](std::monostate) {
            geometry_msgs::Quaternion zero_quat;
            zero_quat.x = 0.0;
            zero_quat.y = 0.0;
            zero_quat.z = 0.0;
            zero_quat.w = 1.0;
            return zero_quat;
        },
        [](const geometry_msgs::Pose& pose) { return pose.orientation; },
        [](const geometry_msgs::Point&) {
            geometry_msgs::Quaternion zero_quat;
            zero_quat.x = 0.0;
            zero_quat.y = 0.0;
            zero_quat.z = 0.0;
            zero_quat.w = 1.0;
            return zero_quat;
        },
        [](const geometry_msgs::Quaternion& quat) { return quat; },
        [](const geometry_msgs::PoseStamped& pose_stamped) { return pose_stamped.pose.orientation; }
    };

    geometry_msgs::Quaternion base_orient = std::visit(get_orientation, base);
    geometry_msgs::Quaternion target_orient = std::visit(get_orientation, target);
    const double orient_dist = std::sqrt(
        std::pow(base_orient.x - target_orient.x, 2) +
        std::pow(base_orient.y - target_orient.y, 2) +
        std::pow(base_orient.z - target_orient.z, 2) +
        std::pow(base_orient.w - target_orient.w, 2));

    return (1.0 - weight_orient) * pos_dist + weight_orient * orient_dist;
}

void TasksManager::optimize_with_2opt(std::vector<Task>& path, float weight_orient) {
    bool improved = true;
    const int n = static_cast<int>(path.size());
    if(n < 4) return;

    while(improved) {
        improved = false;
        for(int i = 0; i < n - 3; ++i) {
            for(int j = i + 2; j < n - 1; ++j) {
                if(!path[i].target || !path[i + 1].target || !path[j].target || !path[j + 1].target) continue;

                const double dist1 = calculate_dist(path[i].target.value(), path[i + 1].target.value(), weight_orient);
                const double dist2 = calculate_dist(path[j].target.value(), path[j + 1].target.value(), weight_orient);
                const double dist3 = calculate_dist(path[i].target.value(), path[j].target.value(), weight_orient);
                const double dist4 = calculate_dist(path[i + 1].target.value(), path[j + 1].target.value(), weight_orient);

                if(dist3 + dist4 < dist1 + dist2) {
                    std::reverse(path.begin() + i + 1, path.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
}

bool TasksManager::is_cancel_requested(Task& task, ExecutionContext* ctx) {
    if(task.pick_params && task.pick_params->canceled) return true;
    if(ctx && ctx->cancel_requested && ctx->cancel_requested->load()) return true;
    return false;
}

void TasksManager::report_pick_feedback(Task& task,
    PickStage stage,
    bool ok,
    ErrorCode code,
    const std::string& text,
    ExecutionContext* ctx) {
    if(task.pick_params) {
        task.pick_params->current_stage = stage;
    }

    if(ctx && ctx->feedback_cb) {
        ctx->feedback_cb(task, stage, ok, code, text);
    }
}

ErrorCode TasksManager::resolve_target_to_base(const TargetVariant& input, TargetVariant& resolved, std::string* source_frame) {
    return std::visit(variant_visitor{
        [&](const std::monostate&) -> ErrorCode {
            return ErrorCode::INVALID_TARGET_TYPE;
        },
        [&](const geometry_msgs::Pose& pose) -> ErrorCode {
            if(source_frame) *source_frame = "base_link";
            resolved = pose;
            return ErrorCode::SUCCESS;
        },
        [&](const geometry_msgs::Point& point) -> ErrorCode {
            if(source_frame) *source_frame = "base_link";
            resolved = point;
            return ErrorCode::SUCCESS;
        },
        [&](const geometry_msgs::Quaternion& quat) -> ErrorCode {
            if(source_frame) *source_frame = "base_link";
            resolved = quat;
            return ErrorCode::SUCCESS;
        },
        [&](const geometry_msgs::PoseStamped& pose_stamped) -> ErrorCode {
            if(source_frame) *source_frame = pose_stamped.header.frame_id;
            if(pose_stamped.header.frame_id.empty() || pose_stamped.header.frame_id == "base_link") {
                geometry_msgs::PoseStamped out = pose_stamped;
                out.header.frame_id = "base_link";
                if(out.header.stamp == ros::Time()) out.header.stamp = ros::Time::now();
                resolved = out;
                return ErrorCode::SUCCESS;
            }

            geometry_msgs::PoseStamped transformed;
            const ErrorCode code = _arm_->end_to_base_tf(pose_stamped, transformed);
            if(code != ErrorCode::SUCCESS) return code;
            transformed.header.frame_id = "base_link";
            resolved = transformed;
            return ErrorCode::SUCCESS;
        }
        }, input);
}

ErrorCode TasksManager::resolve_pose_target_to_base(const TargetVariant& input, geometry_msgs::PoseStamped& resolved, std::string* source_frame) {
    TargetVariant tmp;
    const ErrorCode code = resolve_target_to_base(input, tmp, source_frame);
    if(code != ErrorCode::SUCCESS) return code;

    auto pose_opt = to_pose_stamped(tmp);
    if(!pose_opt) return ErrorCode::INVALID_TARGET_TYPE;

    resolved = *pose_opt;
    resolved.header.frame_id = "base_link";
    if(resolved.header.stamp == ros::Time()) resolved.header.stamp = ros::Time::now();
    return ErrorCode::SUCCESS;
}

tl::optional<TargetVariant> TasksManager::clone_target_with_offset_z(const TargetVariant& input, double dz) const {
    return std::visit(variant_visitor{
        [](const std::monostate&) -> tl::optional<TargetVariant> {
            return tl::nullopt;
        },
        [dz](const geometry_msgs::Pose& pose) -> tl::optional<TargetVariant> {
            geometry_msgs::Pose out = pose;
            out.position.z += dz;
            return TargetVariant{ out };
        },
        [](const geometry_msgs::Point&) -> tl::optional<TargetVariant> {
            return tl::nullopt;
        },
        [](const geometry_msgs::Quaternion&) -> tl::optional<TargetVariant> {
            return tl::nullopt;
        },
        [dz](const geometry_msgs::PoseStamped& pose_stamped) -> tl::optional<TargetVariant> {
            geometry_msgs::PoseStamped out = pose_stamped;
            out.pose.position.z += dz;
            return TargetVariant{ out };
        }
        }, input);
}

ErrorCode TasksManager::rebuild_pick_task_cache(Task& task) {
    if(task.type != TaskType::PICK || !task.pick_params) return ErrorCode::SUCCESS;
    auto& pp = *task.pick_params;

    pp.resolved_pick_pose = tl::nullopt;
    pp.resolved_pre_pick_pose = tl::nullopt;
    pp.resolved_retreat_pose = tl::nullopt;
    pp.resolved_place_pose = tl::nullopt;

    if(!task.raw_target) return ErrorCode::SUCCESS;

    geometry_msgs::PoseStamped pick_pose;
    ErrorCode code = resolve_pose_target_to_base(task.raw_target.value(), pick_pose, &pp.target_source_frame);
    if(code != ErrorCode::SUCCESS) {
        ROS_WARN("任务 ID %u 采摘目标解析到 base frame 失败，错误码：%s", task.id, err_to_string(code).c_str());
        return code;
    }
    pp.resolved_pick_pose = pick_pose;
    task.target = pick_pose;  // 用于排序 / 执行

    if(auto pre_raw = clone_target_with_offset_z(task.raw_target.value(), pp.pre_approach_offset_z)) {
        geometry_msgs::PoseStamped pre_pose;
        code = resolve_pose_target_to_base(pre_raw.value(), pre_pose);
        if(code != ErrorCode::SUCCESS) return code;
        pp.resolved_pre_pick_pose = pre_pose;
    }
    else {
        geometry_msgs::PoseStamped pre_pose = pick_pose;
        pre_pose.pose.position.z += pp.pre_approach_offset_z;
        pp.resolved_pre_pick_pose = pre_pose;
    }

    if(auto retreat_raw = clone_target_with_offset_z(task.raw_target.value(), pp.retreat_offset_z)) {
        geometry_msgs::PoseStamped retreat_pose;
        code = resolve_pose_target_to_base(retreat_raw.value(), retreat_pose);
        if(code != ErrorCode::SUCCESS) return code;
        pp.resolved_retreat_pose = retreat_pose;
    }
    else {
        geometry_msgs::PoseStamped retreat_pose = pick_pose;
        retreat_pose.pose.position.z += pp.retreat_offset_z;
        pp.resolved_retreat_pose = retreat_pose;
    }

    if(pp.use_place_pose && pp.place_target) {
        geometry_msgs::PoseStamped place_pose;
        code = resolve_pose_target_to_base(pp.place_target.value(), place_pose, &pp.place_source_frame);
        if(code != ErrorCode::SUCCESS) {
            ROS_WARN("任务 ID %u 放置目标解析到 base frame 失败，错误码：%s", task.id, err_to_string(code).c_str());
            return code;
        }
        pp.resolved_place_pose = place_pose;
    }

    return ErrorCode::SUCCESS;
}

ErrorCode TasksManager::execute_pick_task(Task& task) {
    return execute_pick_task(task, nullptr);
}

ErrorCode TasksManager::execute_pick_task(Task& task, ExecutionContext* ctx) {
    ROS_INFO("正在执行 PICK 任务（ID = %u）...", task.id);

    if(!task.pick_params) {
        ROS_WARN("任务 ID %u 没有设置采摘参数，无法执行 PICK 任务", task.id);
        return ErrorCode::INVALID_PARAMETER;
    }

    auto& pp = *task.pick_params;
    pp.completed = false;

    ErrorCode code = rebuild_pick_task_cache(task);
    if(code != ErrorCode::SUCCESS) {
        report_pick_feedback(task, PickStage::PICK_FAILED, false, code, "重建采摘缓存失败", ctx);
        return code;
    }

    if(!pp.resolved_pick_pose || !pp.resolved_pre_pick_pose || !pp.resolved_retreat_pose) {
        ROS_WARN("任务 ID %u 缺少解析后的采摘位姿缓存", task.id);
        report_pick_feedback(task, PickStage::PICK_FAILED, false, ErrorCode::INVALID_TARGET_TYPE, "采摘位姿缓存无效", ctx);
        return ErrorCode::INVALID_TARGET_TYPE;
    }

    auto check_cancel = [&]() -> ErrorCode {
        if(!is_cancel_requested(task, ctx)) return ErrorCode::SUCCESS;

        ROS_INFO("任务 ID %u 已取消", task.id);
        if(pp.go_safe_after_cancel) {
            ROS_INFO("任务 ID %u 取消后回安全位", task.id);
            const ErrorCode safe_code = _arm_->home();
            if(safe_code != ErrorCode::SUCCESS) {
                ROS_WARN("任务 ID %u 取消后回安全位失败，错误码：%s", task.id, err_to_string(safe_code).c_str());
            }
        }

        pp.completed = false;
        report_pick_feedback(task, PickStage::PICK_CANCELED, false, ErrorCode::CANCELLED, "任务已取消", ctx);
        return ErrorCode::CANCELLED;
        };

    auto fail_stage = [&](PickStage stage, ErrorCode ec, const std::string& text) -> ErrorCode {
        pp.completed = false;
        report_pick_feedback(task, PickStage::PICK_FAILED, false, ec, text, ctx);
        ROS_WARN("任务 ID %u 在阶段[%s]失败：%s (%s)",
            task.id,
            pick_stage_to_string(stage),
            text.c_str(),
            err_to_string(ec).c_str());
        return ec;
        };

    auto run_with_retry = [&](const std::function<ErrorCode()>& fn, PickStage stage, const std::string& text) -> ErrorCode {
        ErrorCode last = ErrorCode::FAILURE;
        for(uint8_t attempt = 0; attempt <= pp.retry_times; ++attempt) {
            last = fn();
            if(last == ErrorCode::SUCCESS) return ErrorCode::SUCCESS;
            if(last == ErrorCode::CANCELLED) return last;
            ROS_WARN("任务 ID %u 阶段[%s] 第 %u/%u 次尝试失败：%s",
                task.id,
                pick_stage_to_string(stage),
                static_cast<unsigned int>(attempt + 1),
                static_cast<unsigned int>(pp.retry_times + 1),
                err_to_string(last).c_str());
        }
        return last;
        };

    auto exec_arm_pose = [&](const geometry_msgs::PoseStamped& pose,
        PickStage stage,
        const std::string& text) -> ErrorCode {
            report_pick_feedback(task, stage, true, ErrorCode::SUCCESS, text, ctx);
            ErrorCode ec = _arm_->set_target(pose);
            if(ec != ErrorCode::SUCCESS) return fail_stage(stage, ec, text + "：设置目标失败");
            ec = _arm_->plan_and_execute();
            if(ec != ErrorCode::SUCCESS) return fail_stage(stage, ec, text + "：执行失败");
            report_pick_feedback(task, stage, true, ErrorCode::SUCCESS, text + "：完成", ctx);
            return ErrorCode::SUCCESS;
        };

    auto exec_eef = [&](bool open, PickStage stage, const std::string& text) -> ErrorCode {
        if(!_eef_) return fail_stage(stage, ErrorCode::INVALID_INTERFACE, text + "：末端执行器未初始化");

        auto gripper_if = find_eef_interface<GripperEefInterface>(*_eef_);
        if(!gripper_if) {
            return fail_stage(stage, ErrorCode::INVALID_INTERFACE, text + "：末端执行器不支持夹爪接口");
        }

        report_pick_feedback(task, stage, true, ErrorCode::SUCCESS, text, ctx);
        ErrorCode ec = open ? gripper_if->open() : gripper_if->close();
        if(ec != ErrorCode::SUCCESS) return fail_stage(stage, ec, text + "：执行失败");
        report_pick_feedback(task, stage, true, ErrorCode::SUCCESS, text + "：完成", ctx);
        return ErrorCode::SUCCESS;
        };

    report_pick_feedback(task, PickStage::PICK_START, true, ErrorCode::SUCCESS, "开始采摘", ctx);

    code = check_cancel();
    if(code != ErrorCode::SUCCESS) return code;

    if(pp.use_eef) {
        code = run_with_retry([&]() { return exec_eef(true, PickStage::PICK_START, "打开夹爪"); }, PickStage::PICK_START, "打开夹爪");
        if(code != ErrorCode::SUCCESS) return code;
    }

    code = check_cancel();
    if(code != ErrorCode::SUCCESS) return code;

    code = run_with_retry([&]() { return exec_arm_pose(*pp.resolved_pre_pick_pose, PickStage::PICK_MOVE_TO_PICK, "移动到预抓取位"); },
        PickStage::PICK_MOVE_TO_PICK, "移动到预抓取位");
    if(code != ErrorCode::SUCCESS) return code;

    code = check_cancel();
    if(code != ErrorCode::SUCCESS) return code;

    code = run_with_retry([&]() { return exec_arm_pose(*pp.resolved_pick_pose, PickStage::PICK_MOVE_TO_PICK, "移动到抓取位"); },
        PickStage::PICK_MOVE_TO_PICK, "移动到抓取位");
    if(code != ErrorCode::SUCCESS) return code;

    code = check_cancel();
    if(code != ErrorCode::SUCCESS) return code;

    if(pp.use_eef) {
        code = run_with_retry([&]() { return exec_eef(false, PickStage::PICK_PICKING, "闭合夹爪"); },
            PickStage::PICK_PICKING, "闭合夹爪");
        if(code != ErrorCode::SUCCESS) return code;
    }

    code = check_cancel();
    if(code != ErrorCode::SUCCESS) return code;

    code = run_with_retry([&]() { return exec_arm_pose(*pp.resolved_retreat_pose, PickStage::PICK_MOVE_TO_PICK, "回撤到安全位"); },
        PickStage::PICK_MOVE_TO_PICK, "回撤到安全位");
    if(code != ErrorCode::SUCCESS) return code;

    code = check_cancel();
    if(code != ErrorCode::SUCCESS) return code;

    if(pp.use_place_pose && pp.resolved_place_pose) {
        code = run_with_retry([&]() { return exec_arm_pose(*pp.resolved_place_pose, PickStage::PICK_MOVE_TO_PLACE, "移动到放置位"); },
            PickStage::PICK_MOVE_TO_PLACE, "移动到放置位");
        if(code != ErrorCode::SUCCESS) return code;

        code = check_cancel();
        if(code != ErrorCode::SUCCESS) return code;

        if(pp.use_eef) {
            code = run_with_retry([&]() { return exec_eef(true, PickStage::PICK_PLACING, "释放目标"); },
                PickStage::PICK_PLACING, "释放目标");
            if(code != ErrorCode::SUCCESS) return code;
        }
    }

    code = check_cancel();
    if(code != ErrorCode::SUCCESS) return code;

    if(pp.go_home_after_finish) {
        report_pick_feedback(task, PickStage::PICK_GO_HOME, true, ErrorCode::SUCCESS, "回到初始位", ctx);
        code = _arm_->home();
        if(code != ErrorCode::SUCCESS) return fail_stage(PickStage::PICK_GO_HOME, code, "回到初始位失败");
        report_pick_feedback(task, PickStage::PICK_GO_HOME, true, ErrorCode::SUCCESS, "回到初始位：完成", ctx);
    }

    pp.completed = true;
    report_pick_feedback(task, PickStage::PICK_FINISH, true, ErrorCode::SUCCESS, "采摘任务完成", ctx);
    return ErrorCode::SUCCESS;
}

} /* namespace piper */
