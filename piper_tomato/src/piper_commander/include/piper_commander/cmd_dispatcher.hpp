#ifndef _cmd_dispatcher_hpp_
#define _cmd_dispatcher_hpp_

#include <functional>
#include <atomic>

#include "piper_controller/arm_controller.hpp"

namespace piper {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

enum class ArmCmdType {
    HOME,
    MOVE_JOINTS,
    MOVE_TARGET,
    MOVE_TARGET_IN_EEF_FRAME,
    TELESCOPIC_END,
    ROTATE_END,
    MOVE_LINE,
    MOVE_BEZIER,
    MOVE_DECARTES,
    SET_ORIENTATION_CONSTRAINT,
    SET_POSITION_CONSTRAINT,
    SET_JOINT_CONSTRAINT,
    GET_CURRENT_JOINTS,
    GET_CURRENT_POSE,
    MOVE_TO_ZERO
};

struct ArmCmdRequest {
    ArmCmdType type;
    std::vector<double> values;

    TargetVariant target;
    std::vector<std::string> joint_names;
    std::vector<double> joints;
    std::vector<geometry_msgs::Pose> waypoints;
};

struct ArmCmdResult {
    bool success{ false };
    std::string message;

    std::vector<double> values;
    ErrorCode error_code{ ErrorCode::SUCCESS };

    geometry_msgs::Pose current_pose;
    std::vector<double> current_joints;

};

struct ArmCmdFeedback {
    std::string stage;
    double progress{ 0.0 };
    std::string message;
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class ArmCmdDispatcher {
public:
    explicit ArmCmdDispatcher(std::shared_ptr<ArmController> arm) : _arm_(std::move(arm)) {};
    ~ArmCmdDispatcher() = default;

    ArmCmdDispatcher(const ArmCmdDispatcher&) = delete;
    ArmCmdDispatcher& operator=(const ArmCmdDispatcher&) = delete;
    ArmCmdDispatcher(ArmCmdDispatcher&&) = delete;
    ArmCmdDispatcher& operator=(ArmCmdDispatcher&&) = delete;

    using FeedbackCb = std::function<void(const ArmCmdFeedback&)>;
    ArmCmdResult dispatch(const ArmCmdRequest& req);
    ArmCmdResult dispatch(const ArmCmdRequest& req, FeedbackCb cb);

    void cancel();
    bool is_cancelled() const;

private:
    std::shared_ptr<ArmController> _arm_;
    std::atomic_bool _is_cancelled_{ false };

    ArmCmdResult make_ok(const std::string& msg = "命令执行成功");
    ArmCmdResult make_err(ErrorCode code = ErrorCode::FAILURE, const std::string& msg = "命令执行失败");
    ArmCmdResult make_cancelled();

    void report(FeedbackCb cb, const std::string& stage, double progress, const std::string& message);

    ArmCmdResult execute_if_not_cancelled(ErrorCode code, FeedbackCb cb = nullptr);

    ArmCmdResult handle_home(const ArmCmdRequest& req);
    ArmCmdResult handle_move_joints(const ArmCmdRequest& req, FeedbackCb cb = nullptr);
    ArmCmdResult handle_move_target(const ArmCmdRequest& req, FeedbackCb cb = nullptr);
    ArmCmdResult handle_move_target_in_eef_frame(const ArmCmdRequest& req, FeedbackCb cb = nullptr);
    ArmCmdResult handle_telescopic_end(const ArmCmdRequest& req, FeedbackCb cb = nullptr);
    ArmCmdResult handle_rotate_end(const ArmCmdRequest& req, FeedbackCb cb = nullptr);
    ArmCmdResult handle_move_line(const ArmCmdRequest& req, FeedbackCb cb = nullptr);
    ArmCmdResult handle_move_bezier(const ArmCmdRequest& req, FeedbackCb cb = nullptr);
    ArmCmdResult handle_move_decartes(const ArmCmdRequest& req, FeedbackCb cb = nullptr);
    ArmCmdResult handle_set_orientation_constraint(const ArmCmdRequest& req);
    ArmCmdResult handle_set_position_constraint(const ArmCmdRequest& req);
    ArmCmdResult handle_set_joint_constraint(const ArmCmdRequest& req);
    ArmCmdResult handle_get_current_joints(const ArmCmdRequest& req);
    ArmCmdResult handle_get_current_pose(const ArmCmdRequest& req);
    ArmCmdResult handle_move_to_zero(const ArmCmdRequest& req, FeedbackCb cb = nullptr);
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}

#endif
