#ifndef _pick_action_hpp_
#define _pick_action_hpp_

#include <atomic>
#include <memory>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>

#include "piper_controller/arm_controller.hpp"
#include "piper_msgs2/PickTaskAction.h"
#include "piper_task/tasks_manager.hpp"

namespace piper {

class PickTaskAction {
public:
    using PickTaskAS = actionlib::SimpleActionServer<piper_msgs2::PickTaskAction>;

    PickTaskAction(ros::NodeHandle& nh,
        std::shared_ptr<TasksManager> tasks_manager,
        std::shared_ptr<ArmController> arm,
        const std::string& action_name);
    ~PickTaskAction() = default;

    PickTaskAction(const PickTaskAction&) = delete;
    PickTaskAction& operator=(const PickTaskAction&) = delete;
    PickTaskAction(PickTaskAction&&) = delete;
    PickTaskAction& operator=(PickTaskAction&&) = delete;

private:
    void on_goal();
    void on_preempt();

    geometry_msgs::PoseStamped get_current_pose_stamped() const;
    std::string resolve_group_name(const piper_msgs2::PickTaskGoal& goal) const;
    unsigned int resolve_task_id(const piper_msgs2::PickTaskGoal& goal);
    uint32_t estimate_total_steps(const piper_msgs2::PickTaskGoal& goal) const;

private:
    std::unique_ptr<PickTaskAS> _as_;
    std::shared_ptr<TasksManager> _tasks_manager_;
    std::shared_ptr<ArmController> _arm_;

    std::atomic<bool> _cancel_requested_{ false };
    std::atomic<unsigned int> _id_seed_{ 1000 };
};

}  // namespace piper

#endif
