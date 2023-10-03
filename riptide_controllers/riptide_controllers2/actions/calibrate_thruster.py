#! /usr/bin/env python3

import time
from datetime import datetime
import csv
import yaml
import os

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from riptide_msgs2.action import CalibrateThruster
from riptide_msgs2.msg import DshotCommand

NUETRAL_DSHOT = 0


class CalibrateThrusterAction(Node):

    def __init__(self):
        # Create Node
        super().__init__('calibrate_thruster')

        self.running = False
        self.dshot = 0
        self.thruster_num = 0

        self.dshot_pub = self.create_publisher(
            DshotCommand, "command/dshot", qos_profile_sensor_data)

        self._action_server = ActionServer(
            self,
            CalibrateThruster,
            'calibrate_thruster',
            self.collect_data,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        # Create timer to keep publishing the dshot command to prevent firmware from timing out
        self.timer = self.create_timer(0.075, self.publish_dshot_command)

    def collect_data(self, goal_handle):
        self.thruster_num = goal_handle.request.thruster_num
        feedback_msg = CalibrateThruster.Feedback()

        self.get_logger().info('Press any key to tare load cell:')
        wait_for_input = input()
        ##############################
        ####  ethans_code.tare()  ####
        ##############################

        # Start calibration, looping through dshot values and saving data
        self.get_logger().info('Thruster calibration starting...')
        flag = True
        rows_data = []
        for dshot_value in range(DshotCommand.DSHOT_MIN, DshotCommand.DSHOT_MAX, goal_handle.request.step_size):
            # Once egative dshot commands have finished, operator needs to flip thruster before continueing
            if dshot_value > NUETRAL_DSHOT and flag:
                flag = False
                self.get_logger().info('Please flip thruster, press any key once complete:')
                wait_for_input = input()

            # Send dshot command to thruster ### DO I NEED A -1??????
            self.dshot = dshot_value
            self.publish_dshot_command()

            # Send action feedback on current status
            feedback_msg.current_dshot = dshot_value
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(
                f"Current dshot value: {dshot_value}")

            # Delay for transients to setting and for thruster to respond
            time.sleep(0.5)

            # Collect data and store it
            ##################################
            ####  ethans_code.get_data()  ####
            ##################################
            (avg_force, avg_rpm) = (0, 0)
            rows_data.append([dshot_value, avg_rpm, avg_force])

        # Data collection finished, send command to turn off thruster
        self.dshot = NUETRAL_DSHOT
        self.publish_dshot_command()

        # Save data and report results
        result = CalibrateThruster.Result()
        result.results_file_name = self.save_data(rows_data)
        if result.results_file_name:
            goal_handle.succeed(result)
        else:
            goal_handle.abort(result)
        self.running = False

    def goal_callback(self, goal_request):
        # Accept or reject a client request to begin an action
        if self.running:
            return GoalResponse.REJECT
        else:
            self.running = True
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal):
        return CancelResponse.ACCEPT

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def publish_dshot_command(self):
        # Publishes dshot value to specified thruster and 0 to all other thrusters
        # Function is called on timer to prevent firmware from timing out
        dshot_msg = DshotCommand()
        dshot_values = self.zeros_thruster_len()
        dshot_values[self.thruster_num] = self.dshot
        dshot_msg.values = dshot_values
        self.dshot_pub.publish(self.dshot_msg)

    def save_data(self, data):
        # Saves each row of data into a csv file and returns the file name
        try:
            # Create unique file name to store data to
            file_name = self.create_filename()
            with open(file_name, 'w', newline='') as csv_file:
                # Write data in file
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(["dshot", "rpm", "force (N)"])
                csv_writer.writerows(data)

            # File saved successfully, return the file name
            return file_name
        except Exception as e:
            # Error opening file, report error
            self.get_logger().error(str(e))
            self.get_logger().info(
                f"File could not open, here's the data: {data}")
            return ""  # Return an empty string to indicate failure

    def create_filename(self) -> str:
        # Creates unique filename from current date and time
        base_filename = f"thruster{self.thruster_num}_cal_data"
        current_datetime = datetime.now()
        date_string = current_datetime.strftime("%Y-%m-%d-%H-%M")

        # Get the user's home directory
        home_dir = os.path.expanduser("~")
        folder_path = os.path.join(home_dir, "thruster_cal_data")
        # Check if the user has a thruster_cal_data folder, if not make one
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        # Return joined file name in path
        return os.path.join(folder_path, f"{base_filename}_{date_string}.csv")

    def zeros_thruster_len(self):
        # Load thruster info
        self.declare_parameter("vehicle_config", "")
        config_path = self.get_parameter("vehicle_config").value
        if (config_path == ''):
            self.get_logger().fatal("vehicle config file param not set or empty, exiting")
        with open(config_path, 'r') as stream:
            config_file = yaml.safe_load(stream)
        thruster_info = config_file['thrusters']

        # Return zero list that is the number of thrusters long
        return [0]*len(thruster_info)


def main(args=None):
    rclpy.init(args=args)

    thruster_cal_action_server = CalibrateThrusterAction()

    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_cal_action_server, executor=executor)

    thruster_cal_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
