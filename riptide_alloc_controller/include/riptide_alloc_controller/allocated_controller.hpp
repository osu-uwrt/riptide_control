#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>



#include <nav_msgs/msg/odometry.hpp>
#include <riptide_msgs2/msg/controller_command.hpp>
#include <riptide_msgs2/msg/dshot_rpm_feedback.hpp>
#include <riptide_msgs2/action/follow_path.hpp>
#include <geometry_msgs/msg/transform.hpp>

#define ROBOT_COM_FRAME "puddles/com"
#define ROBOT_THRUSTER_PATTERN "puddles/thruster_" // append index to this
#define ROBOT_COB_FRAME "puddles/cob" 

typedef riptide_msgs2::msg::ControllerCommand ControllerCmdMsg;
typedef riptide_msgs2::msg::DshotRPMFeedback FeedbackMsg;
typedef nav_msgs::msg::Odometry OdomMsg;
typedef geometry_msgs::msg::Transform TfMsg;

namespace riptide_alloc_controller
{
    struct RobotData {
        std::vector<TfMsg> thruster_tfs;
        std::vector<bool> thruster_dir;

        TfMsg cob_tf;

        double mass, volume;
    };

    class AllocController : public rclcpp::Node {
    public:
        AllocController();
        ~AllocController();

        bool loadConfig(std::string path);

        void updateConfig();

        // ROS callbacks for changing command
        void updateLinCmd(const ControllerCmdMsg cmd);
        void updateAngCmd(const ControllerCmdMsg cmd);

        // ROS callback for thruster RPM feedback
        void updateRpmFeedback(const FeedbackMsg data);

        // ROS callback for new Odom data and output calcs
        void updateOdom(const OdomMsg data);

    private:
        // config data container
        std::shared_ptr<RobotData> robot_data;

        // message data containers for last infos
        ControllerCmdMsg linear_cmd, angular_cmd;
        FeedbackMsg thruster_rpm_data;

        // subscribe to the odom output
        rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub;

        // subscribe to both command inputs
        rclcpp::Subscription<ControllerCmdMsg>::SharedPtr lin_cmd_sub, ang_cmd_sub;

        // subscribe to the feedback info
        rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub;
    };
} // namespace riptide_alloc_controller