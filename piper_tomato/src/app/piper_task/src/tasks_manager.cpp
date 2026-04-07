#include "piper_task/tasks_manager.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace piper {

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
    auto group = find_task_group(group_name);
    if(group) {
        ROS_WARN("任务组 '%s' 已存在，无法创建", group_name.c_str());
        return ErrorCode::TASK_GROUP_EXISTS;
    }

    TaskGroup new_group;
    new_group.sort_type = sort_type;
    new_group.tasks.clear();

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


    group.value()->tasks.clear();

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
    if(!group) {
        ROS_WARN("任务组 '%s' 不存在，无法设置排序权重", group_name.c_str());
        return ErrorCode::TASK_GROUP_NOT_FOUND;
    }

    if(weight_orient < 0.0f || weight_orient > 1.0f) {
        ROS_WARN("排序权重必须在 [0, 1] 范围内，当前值：%f", weight_orient);
        return ErrorCode::INVALID_PARAMETER;
    }

    group.value()->weight_orient = weight_orient;
    ROS_INFO("成功设置任务组 '%s' 的距离排序姿态权重为 %f", group_name.c_str(), weight_orient);

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
ErrorCode TasksManager::add_task(const std::string& group_name, int id, TaskType task_type, const std::string& task_description) {
    auto group = find_task_group(group_name);
    if(!group) return group.error();

    Task new_task;
    new_task.desc = task_description;
    new_task.type = task_type;
    new_task.id = id;

    if(group.value()->tasks.find(id) != group.value()->tasks.end()) {
        ROS_WARN("任务组 '%s' 中已存在任务 ID %d，无法添加", group_name.c_str(), id);
        return ErrorCode::TASK_EXISTS;
    }

    group.value()->tasks[id] = std::move(new_task);
    ROS_INFO("成功向任务组 '%s' 添加任务 ID %d", group_name.c_str(), id);
    return ErrorCode::SUCCESS;
}

/**
 * @brief 删除任务
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @return 错误码
 */
ErrorCode TasksManager::delete_task(const std::string& group_name, int id) {
    auto group = find_task_group(group_name);
    if(!group) return group.error();
    auto task = find_task(group_name, id);
    if(!task) return task.error();

    group.value()->tasks.erase(id);
    ROS_INFO("成功从任务组 '%s' 删除任务 ID %d", group_name.c_str(), id);

    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置任务目标
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @param target 任务目标
 * @return 错误码
 */
ErrorCode TasksManager::set_task_target(const std::string& group_name, int id, const tl::optional<TargetVariant>& target) {
    auto task = find_task(group_name, id);
    if(!task) return task.error();

    task.value()->target = target;
    ROS_INFO("成功设置任务组 '%s'中任务 ID %d 的目标", group_name.c_str(), id);

    return ErrorCode::SUCCESS;
}

/**
 * @brief 执行任务组中的指定任务
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @return 错误码
 */
ErrorCode TasksManager::execute_task(const std::string& group_name, int id) {
    auto task = find_task(group_name, id);
    if(!task) return task.error();

    return execute_task(*task.value());
}

/**
 * @brief 执行任务对象
 * @param task 任务对象
 * @return 错误码
 */
ErrorCode TasksManager::execute_task(Task& task) {
    ROS_INFO("开始执行任务: %s", task.desc.c_str());
    ROS_INFO("任务描述: %s", task.desc.c_str());

    if(task.type == TaskType::NONE) {
        if(!task.target) {
            ROS_WARN("任务 ID %d 没有设置目标，无法执行", task.id);
            return ErrorCode::INVALID_PARAMETER;
        }
        ErrorCode code = _arm_->set_target(task.target.value());
        if(code != ErrorCode::SUCCESS) {
            ROS_WARN("执行任务 ID %d 失败，无法设置目标，错误码：%s", task.id, err_to_string(code).c_str());
            return code;
        }
        code = _arm_->plan_and_execute();
        if(code != ErrorCode::SUCCESS) {
            ROS_WARN("执行任务 ID %d 失败，无法规划执行，错误码：%s", task.id, err_to_string(code).c_str());
            return code;
        }
        ROS_INFO("成功执行任务 ID %d", task.id);
    }
    else if(task.type == TaskType::PICK) {
        ROS_INFO("正在执行 PICK 任务...");
    }

    return ErrorCode::SUCCESS;
}

/**
 * @brief 执行整个任务组
 * @param group_name 任务组名称
 * @return 错误码
 */
ErrorCode TasksManager::execute_task_group(const std::string& group_name) {
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
        ErrorCode err_code = execute_task(task);
        if(err_code != ErrorCode::SUCCESS) {
            ROS_WARN("执行任务组 '%s' 中任务 ID %d 失败，错误码：%s", group_name.c_str(), task.id, err_to_string(err_code).c_str());
            return err_code;
        }
    }

    return ErrorCode::SUCCESS;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

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
 * @param error_code 输出错误码
 * @return 任务指针，失败返回 nullptr
 */
tl::expected<Task*, ErrorCode> TasksManager::find_task(const std::string& group_name, int id) {
    auto group = find_task_group(group_name);
    if(!group) {
        ROS_WARN("任务组 '%s' 不存在", group_name.c_str());
        return tl::make_unexpected(ErrorCode::TASK_GROUP_NOT_FOUND);
    }

    auto task = group.value()->tasks.find(id);
    if(task == group.value()->tasks.end()) {
        ROS_WARN("任务组 '%s' 中不存在任务 ID %d", group_name.c_str(), id);
        return tl::make_unexpected(ErrorCode::TASK_NOT_FOUND);
    }

    return &task->second;
}

/**
 * @brief 查找任务对象（const 版本）
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @param error_code 输出错误码
 * @return 任务指针，失败返回 nullptr
 */
tl::expected<const Task*, ErrorCode> TasksManager::find_task(const std::string& group_name, int id) const {
    auto task_group = _task_groups_.find(group_name);
    if(task_group == _task_groups_.end()) {
        ROS_WARN("任务组 '%s' 不存在", group_name.c_str());
        return tl::make_unexpected(ErrorCode::TASK_GROUP_NOT_FOUND);
    }

    auto task = task_group->second.tasks.find(id);
    if(task == task_group->second.tasks.end()) {
        ROS_WARN("任务组 '%s' 中不存在任务 ID %d", group_name.c_str(), id);
        return tl::make_unexpected(ErrorCode::TASK_NOT_FOUND);
    }

    return &task->second;
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
    }
    else if(task_group.sort_type == SortType::DIST) {
        std::vector<std::reference_wrapper<const Task>> sortable_tasks;
        for(auto& [id, task] : task_group.tasks) {
            if(task.target) {
                sortable_tasks.push_back(std::cref(task));
            }
        }
        if(sortable_tasks.empty()) {
            ROS_WARN("任务组中没有设置目标的任务，无法按距离排序，默认按 ID 排序");
            return ErrorCode::INVALID_PARAMETER;
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

        while(task_group.sorted_tasks.size() < task_group.tasks.size()) {
            auto cur_task = task_group.tasks.find(cur_id);
            task_group.sorted_tasks.push_back(cur_task->second);
            task_group.sorted_tasks.back().id = cur_id;
            visited_ids.insert(cur_id);

            if(visited_ids.size() == task_group.tasks.size()) break;

            double min_dist_local = -1.0;
            for(const auto& [id, task] : task_group.tasks) {
                if(visited_ids.count(id) > 0) continue;

                if(!task.target) continue;
                double dist = calculate_dist(task_group.sorted_tasks.back().target.value(), task.target.value(), task_group.weight_orient);
                if(min_dist_local < 0 || dist < min_dist_local) {
                    min_dist_local = dist;
                    cur_id = id;
                }
            }
        }

        optimize_with_2opt(task_group.sorted_tasks, task_group.weight_orient);

        ROS_INFO("任务组已按加权距离排序");
    }

    return ErrorCode::SUCCESS;
}

/**
 * @brief 计算两个目标之间的加权距离
 * @param base 基准目标
 * @param target 目标
 * @param weight_orient 姿态权重
 * @return 加权距离
 */
double TasksManager::calculate_dist(const TargetVariant& base, const TargetVariant& target, float weight_orient) {
    auto get_position = variant_visitor{
        [](std::monostate) {
            geometry_msgs::Point zero_point;
            zero_point.x = 0.0;
            zero_point.y = 0.0;
            zero_point.z = 0.0;
            return zero_point;
        },
        [](const geometry_msgs::Pose& pose) {return pose.position; },
        [](const geometry_msgs::Point& point) {return point; },
        [](const geometry_msgs::Quaternion&) {
            geometry_msgs::Point zero_point;
            zero_point.x = 0.0;
            zero_point.y = 0.0;
            zero_point.z = 0.0;
            return zero_point;
        },
        [](const geometry_msgs::PoseStamped& pose_stamped) {return pose_stamped.pose.position; }
    };
    geometry_msgs::Point base_pos = std::visit(get_position, base);
    geometry_msgs::Point target_pos = std::visit(get_position, target);
    double pos_dist = std::sqrt(std::pow(base_pos.x - target_pos.x, 2) + std::pow(base_pos.y - target_pos.y, 2) + std::pow(base_pos.z - target_pos.z, 2));

    double orient_dist = 0.0;
    auto get_orientation = variant_visitor{
        [](std::monostate) {
            geometry_msgs::Quaternion zero_quat;
            zero_quat.x = 0.0;
            zero_quat.y = 0.0;
            zero_quat.z = 0.0;
            zero_quat.w = 1.0;
            return zero_quat;
        },
        [](const geometry_msgs::Pose& pose) {return pose.orientation; },
        [](const geometry_msgs::Point&) {
            geometry_msgs::Quaternion zero_quat;
            zero_quat.x = 0.0;
            zero_quat.y = 0.0;
            zero_quat.z = 0.0;
            zero_quat.w = 1.0;
            return zero_quat;
        },
        [](const geometry_msgs::Quaternion& quat) {return quat; },
        [](const geometry_msgs::PoseStamped& pose_stamped) {return pose_stamped.pose.orientation; }
    };
    geometry_msgs::Quaternion base_orient = std::visit(get_orientation, base);
    geometry_msgs::Quaternion target_orient = std::visit(get_orientation, target);
    orient_dist = std::sqrt(std::pow(base_orient.x - target_orient.x, 2) + std::pow(base_orient.y - target_orient.y, 2) + std::pow(base_orient.z - target_orient.z, 2) + std::pow(base_orient.w - target_orient.w, 2));

    return (1.0 - weight_orient) * pos_dist + weight_orient * orient_dist;
}

/**
 * @brief 使用 2-opt 算法优化任务路径
 * @param path 任务路径
 * @param weight_orient 姿态权重
 */
void TasksManager::optimize_with_2opt(std::vector<Task>& path, float weight_orient) {
    bool improved = true;
    int n = path.size();

    if(n < 4) return;

    while(improved) {
        improved = false;
        for(int i = 0; i < n - 3; ++i) {
            for(int j = i + 2; j < n - 1; ++j) {
                if(!path[i].target || !path[i + 1].target || !path[j].target || !path[j + 1].target) continue;
                double dist1 = calculate_dist(path[i].target.value(), path[i + 1].target.value(), weight_orient);
                double dist2 = calculate_dist(path[j].target.value(), path[j + 1].target.value(), weight_orient);

                double dist3 = calculate_dist(path[i].target.value(), path[j].target.value(), weight_orient);
                double dist4 = calculate_dist(path[i + 1].target.value(), path[j + 1].target.value(), weight_orient);

                if(dist3 + dist4 < dist1 + dist2) {
                    std::reverse(path.begin() + i + 1, path.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
}

} /* namespace piper */
