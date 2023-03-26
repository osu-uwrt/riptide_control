#include "riptide_alloc_controller/allocated_controller.hpp"
#include <yaml-cpp/yaml.h>

#include <memory>
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace riptide_alloc_controller
{
    AllocController::AllocController(): rclcpp::Node(""){
        // declare parameters
        declare_parameter<std::string>("robot", "");
        declare_parameter<std::string>("vehicle_config", "");

        // get param values
        auto robot_name = get_parameter("robot").as_string();
        auto config_file = get_parameter("vehicle_config").as_string();

        // make sure that the two are valid
        if(robot_name == ""){
            RCLCPP_FATAL(get_logger(), "Robot name parameter un-set!");
            exit(-1);
        }
        if(config_file == ""){
            RCLCPP_FATAL(get_logger(), "Robot yaml parameter un-set!");
            exit(-2);
        }

        loadConfig(config_file);

        // make the odom subcription to get EKF odom
        odom_sub = create_subscription<OdomMsg>("odometry/filtered", rclcpp::SensorDataQoS(), std::bind(&AllocController::updateOdom, this, _1));

        // subscribe to the input commands
        lin_cmd_sub = create_subscription<ControllerCmdMsg>("command/linear", rclcpp::SystemDefaultsQoS(), std::bind(&AllocController::updateLinCmd, this, _1));
        lin_cmd_sub = create_subscription<ControllerCmdMsg>("command/angular", rclcpp::SystemDefaultsQoS(), std::bind(&AllocController::updateAngCmd, this, _1));

        // feedback sub
        feedback_sub = create_subscription<FeedbackMsg>("dshot/rpm", rclcpp::SensorDataQoS(), std::bind(&AllocController::updateRpmFeedback, this, _1));
    }

    AllocController::~AllocController(){

    }

    // load the info from the file on startup
    bool AllocController::loadConfig(std::string path){
        std::string err = "";
        robot_data = std::make_shared<RobotData>();

        // parse the yaml
        auto config = YAML::LoadFile(path);

        // check the keys we want
        if(!config["mass"]) err = "Missing mass in vehicle config!";
        if(!config["volume"]) err = "Missing volume in vehicle config!";
        if(!config["com"]) err = "Missing COM in vehicle config!";
        if(!config["cob"]) err = "Missing COB in vehicle config!";
        if(!config["controller"]) err = "Missing controller data in vehicle config!";
        if(!config["controller"]["linear"]) err = "Missing controller linear data in vehicle config!";
        if(!config["controller"]["angular"]) err = "Missing controller angular data in vehicle config!";

        // load the info into the struct
        robot_data->mass = config["mass"].as<double>();
        robot_data->volume = config["volume"].as<double>();


        if(err.size() > 0){
            RCLCPP_FATAL(get_logger(), err.c_str());
            exit(-1);
        }  
    }

    // parameter update callback
    void AllocController::updateConfig(){

    }

        // generic callbacks
    void AllocController::updateLinCmd(const ControllerCmdMsg cmd){ linear_cmd = cmd; }
    void AllocController::updateAngCmd(const ControllerCmdMsg cmd){ angular_cmd = cmd; }
    void AllocController::updateRpmFeedback(const FeedbackMsg data){ thruster_rpm_data = data; }

    // This is the big brain callback function. Do the calcs here and send the thrusters
    void AllocController::updateOdom(const OdomMsg data){
        
    }

} // namespace riptide_alloc_controller