#ifndef _tasks_manager_hpp_
#define _tasks_manager_hpp_

#include <atomic>
#include <functional>
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

/**
 * @brief 任务类型枚举
 */
enum class TaskType {
    MOVE_ONLY = 0,
    PICK,
};

/**
 * @brief 任务排序方式枚举
 */
enum class SortType {
    ID,
    DIST,
};

#define PICK_STAGE_TABLE \
    X(IDLE, "空闲") \
    X(START, "开始") \
    X(MOVE_TO_PICK, "移动到采摘位") \
    X(PICKING, "采摘中") \
    X(MOVE_TO_PLACE, "移动到放置位") \
    X(PLACING, "放置中") \
    X(FINISH, "完成") \
    X(GO_HOME, "回到初始位") \
    X(CANCELED, "已取消") \
    X(FAILED, "失败")

#define X(name, desc) PICK_##name,
enum class PickStage {
    PICK_STAGE_TABLE
};
#undef X

/**
 * @brief 单果采摘任务参数
 * @note raw 输入保留在 target / place_target 中；resolved_* 为在“加入任务时”冻结到底座坐标系的执行缓存。
 */
struct PickTaskParams {
    bool use_place_pose{ false };
    tl::optional<TargetVariant> place_target;  ///< 原始输入，可为 base / eef frame

    bool use_eef{ true };
    bool go_home_after_finish{ false };
    bool go_safe_after_cancel{ true };
    uint8_t retry_times{ 0 };

    PickStage current_stage{ PickStage::PICK_IDLE };
    bool completed{ false };
    bool canceled{ false };

    double pre_approach_offset_z{ 0.05 };
    double retreat_offset_z{ 0.05 };

    // ==== 运行缓存：统一冻结到 base_link ==== //
    tl::optional<geometry_msgs::PoseStamped> resolved_pick_pose;
    tl::optional<geometry_msgs::PoseStamped> resolved_pre_pick_pose;
    tl::optional<geometry_msgs::PoseStamped> resolved_retreat_pose;
    tl::optional<geometry_msgs::PoseStamped> resolved_place_pose;

    std::string target_source_frame;
    std::string place_source_frame;
};

/**
 * @brief 单个任务描述结构体
 * @note target 为“规范化后”的执行目标（默认冻结到底座坐标系），raw_target 保留原始输入用于重建缓存。
 */
struct Task {
    unsigned int id{};
    std::string desc;
    TaskType type{ TaskType::MOVE_ONLY };

    tl::optional<TargetVariant> target;      ///< 规范化后的目标，供排序 / 执行使用
    tl::optional<TargetVariant> raw_target;  ///< 原始输入目标，供调试 / 重新解析使用

    tl::optional<PickTaskParams> pick_params;
};

struct TaskGroup {
    std::map<unsigned int, Task> tasks;
    std::vector<Task> sorted_tasks;

    SortType sort_type{ SortType::ID };
    float weight_orient{ 0.3f };
};

class TasksManager {
public:
    struct ExecutionContext {
        std::atomic<bool>* cancel_requested{ nullptr };
        std::function<void(const Task& task,
            PickStage stage,
            bool last_success,
            ErrorCode code,
            const std::string& text)> feedback_cb;
    };

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
    ErrorCode execute_task_group(const std::string& group_name, ExecutionContext* ctx);

    ErrorCode set_dist_sort_weight_orient(const std::string& group_name, float weight_orient);

    ErrorCode add_task(const std::string& group_name, unsigned int id, TaskType task_type = TaskType::MOVE_ONLY, const std::string& task_description = "");
    ErrorCode delete_task(const std::string& group_name, unsigned int id);
    ErrorCode set_task_target(const std::string& group_name, unsigned int id, const tl::optional<TargetVariant>& target);
    ErrorCode set_task_pick_params(const std::string& group_name, unsigned int id, const PickTaskParams& pick_params);
    ErrorCode execute_task(const std::string& group_name, unsigned int id);
    ErrorCode execute_task(const std::string& group_name, unsigned int id, ExecutionContext* ctx);
    ErrorCode execute_task(Task& task);
    ErrorCode execute_task(Task& task, ExecutionContext* ctx);

private:
    tl::expected<TaskGroup*, ErrorCode> find_task_group(const std::string& group_name);
    tl::expected<Task*, ErrorCode> find_task(const std::string& group_name, unsigned int id);
    tl::expected<const Task*, ErrorCode> find_task(const std::string& group_name, unsigned int id) const;

    ErrorCode sort_tasks(TaskGroup& task_group);
    double calculate_dist(const TargetVariant& base, const TargetVariant& target, float weight_orient = 0.3f);
    void optimize_with_2opt(std::vector<Task>& path, float weight_orient = 0.3f);

    bool is_cancel_requested(Task& task, ExecutionContext* ctx);
    void report_pick_feedback(Task& task,
        PickStage stage,
        bool ok,
        ErrorCode code,
        const std::string& text,
        ExecutionContext* ctx);
    ErrorCode execute_pick_task(Task& task);
    ErrorCode execute_pick_task(Task& task, ExecutionContext* ctx);

    ErrorCode resolve_target_to_base(const TargetVariant& input, TargetVariant& resolved, std::string* source_frame = nullptr);
    ErrorCode resolve_pose_target_to_base(const TargetVariant& input, geometry_msgs::PoseStamped& resolved, std::string* source_frame = nullptr);
    tl::optional<TargetVariant> clone_target_with_offset_z(const TargetVariant& input, double dz) const;
    ErrorCode rebuild_pick_task_cache(Task& task);

private:
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<EndEffector> _eef_;
    std::string _arm_name_;
    std::string _eef_name_;

    std::map<std::string, TaskGroup> _task_groups_;
};

} /* namespace piper */

#endif
