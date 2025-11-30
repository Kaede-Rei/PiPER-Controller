#ifndef _d_eef_pos_cmd_hpp_
#define _d_eef_pos_cmd_hpp_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_ros/transform_listener.h>

namespace piper
{
    /* ========================= Typedef / 量 定 义 ========================= */

    /**
     * @brief 任务目标动作枚举类型
     * @param NONE 无动作
     * @param PICK 采摘动作
     */
    enum class TargetAction_t{
        NONE = 0,
        PICK,
    };

    /**
     * @brief 任务目标结构体
     * @param pose 目标位姿
     * @param wait_time 等待时间
     * @param action 目标动作
     */
    typedef struct{
        geometry_msgs::Pose pose;
        double wait_time;
        TargetAction_t action;
    } TaskTarget_t;

    /**
     * @brief A*节点结构体
     * @param droll 滚转角偏移量
     * @param dpitch 俯仰角偏移量
     * @param g 起点到当前节点的代价
     * @param h 当前节点到目标节点的启发式代价
     * @param f 总代价（f = g + h）
     * @note 用于机械臂末端位姿微调的A*搜索算法中的节点表示
     */
    typedef struct{
        double droll, dpitch;
        double g, h, f;
    } AStarNode_t;

    /**
     * @brief 二元组哈希函数结构体实例
     * @param t 二元组
     * @return 哈希值（size_t类型）
     * @note 使用示例：size_t hash_value = _tuple_hash(std::tuple<int, int>{roll, pitch});
     */
    typedef struct{
        std::size_t operator()(const std::pair<int, int>& t) const noexcept
        {
            return std::hash<int>()(t.first) ^ (std::hash<int>()(t.second) << 1);
        }
    } PairHash_t;

    /**
     * @brief A*节点比较器结构体
     * @note 用于优先队列中节点的比较，按照f值从小到大排序
     */
    struct AstarNodeCmper{
        bool operator()(const AStarNode_t& a, const AStarNode_t& b) const noexcept {
            return a.f > b.f;
        }
    };

    /* ========================= 接 口 A P I 声 明 ========================= */

    /**
     * @brief 机械臂末端执行器位姿控制类
     */
    class EefPoseCmd{
    public:
        EefPoseCmd(ros::NodeHandle& nh, const std::string& plan_group_name);

        void eefTfBase(geometry_msgs::PoseStamped& target_pose_eef, geometry_msgs::PoseStamped& target_pose_base);
        bool isIkValid(const geometry_msgs::Pose& target_pose);
        bool searchReachablePose(geometry_msgs::Pose& target_pose, double step, double radius);
        bool setGoalPoseEef(geometry_msgs::PoseStamped& target_pose, bool allow_tweak = true);
        void resetToZero(void);
        geometry_msgs::Pose getCurrentEefPose(void);
        std::vector<double> getCurrentJointPose(void);

    private:
        // 节点句柄
        ros::NodeHandle _nh_;
        // 机械臂接口
        moveit::planning_interface::MoveGroupInterface _arm_;
        // 规划结果
        moveit::planning_interface::MoveGroupInterface::Plan _plan_;
        // 场景监控器
        planning_scene_monitor::PlanningSceneMonitorPtr _scene_monitor_;
        // 规划场景指针
        planning_scene::PlanningScenePtr _planning_scene_;
        // 规划参考坐标系(基坐标系)名称
        std::string _plan_frame_;
        // 末端坐标系名称
        std::string _eef_frame_;
        // 规划结束执行标志
        bool _is_success_;
        // TF缓存
        tf2_ros::Buffer _tf_buffer_;
        // TF监听器
        tf2_ros::TransformListener _tf_listener_;
        // 机械臂关节模型组指针
        const robot_state::JointModelGroup* _jmg_ = nullptr;
        // 当前机械臂状态指针
        moveit::core::RobotStatePtr _current_state_ = nullptr;
    };

    /**
     * @brief 任务组规划类
     */
    class TaskGroupPlanner{
    public:
        TaskGroupPlanner(EefPoseCmd& eef_cmd);
        void add(const TaskTarget_t& target);
        void clear();
        void executeAll();

    private:
        EefPoseCmd& _eef_cmd_;
        std::vector<TaskTarget_t> _task_list_;
    };

    /**
     * @brief 障碍物管理类
     */
    class Barrier{
    public:

    };

}

#endif
