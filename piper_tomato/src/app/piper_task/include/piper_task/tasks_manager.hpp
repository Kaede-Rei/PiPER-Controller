#ifndef _tasks_manager_hpp_
#define _tasks_manager_hpp_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tl_optional/optional.hpp"
#include "tl_expected/expected.hpp"

#include "piper_controller/types.hpp"
#include "piper_controller/arm_controller.hpp"
#include "piper_controller/eef_controller.hpp"  // IWYU pragma: keep

namespace piper {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 任务类型枚举
 */
enum class TaskType {
    NONE = 0,
    PICK,
};

/**
 * @brief 任务排序方式枚举
 * @param ID 按任务 ID 排序
 * @param DIST 按位置和姿态加权距离排序
 */
enum class SortType {
    ID,
    DIST,
};

/**
 * @brief 单个任务描述结构体
 */
struct Task {
    /// @brief 任务 ID
    unsigned int id{};
    /// @brief 任务描述
    std::string desc;
    /// @brief 任务类型
    TaskType type{ TaskType::NONE };

    /// @brief 任务目标
    tl::optional<TargetVariant> target;
};

/**
 * @brief 任务组结构体
 */
struct TaskGroup {
    /// @brief 原始任务集合，key 为任务 ID
    std::map<unsigned int, Task> tasks;
    /// @brief 排序后的任务序列
    std::vector<Task> sorted_tasks;

    /// @brief 排序方式
    SortType sort_type{ SortType::ID };
    /// @brief 姿态权重，仅 DIST 排序方式使用
    float weight_orient{ 0.3f };
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class TasksManager {
public:
    TasksManager(std::shared_ptr<ArmController> arm, std::shared_ptr<EndEffector> eef);
    ~TasksManager() = default;

    TasksManager(const TasksManager&) = delete;
    TasksManager& operator=(const TasksManager&) = delete;
    TasksManager(TasksManager&&) = delete;
    TasksManager& operator=(TasksManager&&) = delete;

    ErrorCode create_task_group(const std::string& group_name, SortType sort_type = SortType::ID);
    ErrorCode delete_task_group(const std::string& group_name);
    ErrorCode clear_task_group(const std::string& group_name);
    ErrorCode execute_task_group(const std::string& group_name);

    ErrorCode set_dist_sort_weight_orient(const std::string& group_name, float weight_orient);

    ErrorCode add_task(const std::string& group_name, unsigned int id, TaskType task_type = TaskType::NONE, const std::string& task_description = "");
    ErrorCode delete_task(const std::string& group_name, unsigned int id);
    ErrorCode set_task_target(const std::string& group_name, unsigned int id, const tl::optional<TargetVariant>& target);
    ErrorCode execute_task(const std::string& group_name, unsigned int id);
    ErrorCode execute_task(Task& task);

private:
    tl::expected<TaskGroup*, ErrorCode> find_task_group(const std::string& group_name);
    tl::expected<Task*, ErrorCode> find_task(const std::string& group_name, unsigned int id);
    tl::expected<const Task*, ErrorCode> find_task(const std::string& group_name, unsigned int id) const;
    ErrorCode sort_tasks(TaskGroup& task_group);
    double calculate_dist(const TargetVariant& base, const TargetVariant& target, float weight_orient = 0.3);
    void optimize_with_2opt(std::vector<Task>& path, float weight_orient = 0.3);

private:
    /// @brief 机械臂控制器
    std::shared_ptr<ArmController> _arm_;
    /// @brief 末端执行器
    std::shared_ptr<EndEffector> _eef_;
    /// @brief 机械臂名称
    std::string _arm_name_;
    /// @brief 末端执行器名称
    std::string _eef_name_;

    /// @brief 任务组集合
    std::map<std::string, TaskGroup> _task_groups_;
};

} /* namespace piper */

#endif
