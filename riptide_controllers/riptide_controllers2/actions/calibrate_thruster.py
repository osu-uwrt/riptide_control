#! /usr/bin/env python3

"""
NOTES FROM NATHAN:
- Need to change RPM subscriber name and type on line 59; I don't know what it's supposed to be
- I assumed the RPM message was stored as a list, so I accessed the correct thruster in callback with index, change if assumption was garbo
- Change constants as desired
- I have a MATLAB script to curve fit the data to any function imaginable and to do other data processing from csv file. Please send me data once collected
- Pizza is delecious
"""

import time
from datetime import datetime
from statistics import mean
import csv
import yaml
import os

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from std_msgs.msg import Empty, Int32
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from riptide_msgs2.action import CalibrateThruster
from riptide_msgs2.msg import DshotCommand

NUETRAL_DSHOT = 0           # Dshot value which is "off" for the thrusters
DSHOT_PUB_PERIOD = 0.075    # Time in seconds between dshot timer publishes
DELAY_TIME = 0.5            # Delay in seconds between sending command and collecting data
N_SAMPLES = 15              # Number of samples to collect at each dshot value
COLLECTION_TIMEOUT = 15     # Allowed data collection time in seconds before timeout


class CalibrateThrusterAction(Node):

    def __init__(self):
        # Create Node
        super().__init__('calibrate_thruster')

        # Variables
        self.running = False
        self.collecting_data = False
        self.triggered = False
        self.dshot = NUETRAL_DSHOT
        self.thruster_length = self.get_thruster_length()
        self.thruster_num = 0
        self.rpm = []
        self.force = []

        # Create publishers and subscribers
        self.dshot_pub = self.create_publisher(
            DshotCommand, "command/dshot", qos_profile_sensor_data)
        self.trigger_sub = self.create_subscription(
            Empty, "command/trigger", self.trigger_callback, qos_profile_system_default)
        self.thruster_rpm_sub = self.create_subscription(
            Empty, "ENTER_RPM_TOPIC_NAME_HERE", self.rpm_callback, qos_profile_sensor_data)
        self.thruster_force_sub = self.create_subscription(
            Int32, "force_gauge/force", self.force_callback, qos_profile_sensor_data)

        # Create action server
        self._action_server = ActionServer(
            self,
            CalibrateThruster,
            'calibrate_thruster',
            self.collect_data_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        # Create timer to keep publishing the dshot command to prevent firmware from timing out
        self.timer = self.create_timer(
            DSHOT_PUB_PERIOD, self.publish_dshot_command)

    def collect_data_callback(self, goal_handle):
        # Method walks data through collection process and saves the result to csv file

        self.thruster_num = goal_handle.request.thruster_num
        feedback_msg = CalibrateThruster.Feedback()

        # Wait for user to tare scale by reseting the arduino
        self.get_logger().info(
            'Please reset arduino to tare the scale, publish command/trigger once complete')
        self.wait_for_input()

        # Start calibration, looping through dshot values and saving data
        self.get_logger().info('Thruster calibration starting...')
        flag = True
        rows_data = []
        flip_force = -1
        for dshot_value in range(DshotCommand.DSHOT_MIN, DshotCommand.DSHOT_MAX, goal_handle.request.step_size):
            # Once negative dshot commands have finished, operator needs to flip thruster before continueing
            if dshot_value > NUETRAL_DSHOT and flag:
                flag = False
                flip_force = 1
                self.get_logger().info('Please flip thruster, publish command/trigger once complete')
                self.wait_for_input()

            # Send dshot command to thruster
            self.dshot = dshot_value
            self.publish_dshot_command()

            # Send action feedback on current status
            feedback_msg.current_dshot = dshot_value
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(
                f"Current dshot value: {dshot_value}")

            # Delay for transients to settle and for thruster to respond
            time.sleep(DELAY_TIME)

            # Collect data and store it
            (avg_rpm, avg_force) = self.get_data(N_SAMPLES)
            rows_data.append([dshot_value, avg_rpm, flip_force*avg_force])

        # Data collection finished, send command to turn off thruster
        self.dshot = NUETRAL_DSHOT
        self.publish_dshot_command()

        # Save data and report results
        result = CalibrateThruster.Result()
        result.results_file_name = self.save_data(rows_data)
        if result.results_file_name:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.running = False
        self.get_logger().info(result.results_file_name)
        return result

    def get_data(self, N_samples):
        # Method starts data collect, and returns average results after N_samples have been collected
        # Once collecting_data is True, the rpm and force lists will get appended by subscriber callbacks

        # Clear previous data and start collecting
        self.rpm = []
        self.force = []
        self.collecting_data = True

        # Wait for samples to collect, report error if the collection is taking too long
        start_time = time.time()
        while rclpy.ok() and min(len(self.rpm), len(self.force)) < N_samples:
            if time.time()-start_time > COLLECTION_TIMEOUT:
                self.get_logger().error(
                    f"Data collection timed out at dshot value {self.dshot}")
                # Data failed to collect, return imaginary numbers to indicate failure
                return (1j, 1j)
        # Stop data collection and return averages
        self.collecting_data = False
        return (mean(self.rpm), mean(self.force))

    def save_data(self, data) -> str:
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
        date_string = current_datetime.strftime("%Y-%m-%d-%H-%M-%S")

        # Get the user's home directory
        home_dir = os.path.expanduser("~")
        folder_path = os.path.join(home_dir, "thruster_cal_data")
        # Check if the user has a thruster_cal_data folder, if not make one
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        # Return joined file name in path
        return os.path.join(folder_path, f"{base_filename}_{date_string}.csv")

    def get_thruster_length(self) -> int:
        # Returns amount of thrusters on vehicle
        # Load thruster info
        self.declare_parameter("vehicle_config", "")
        config_path = self.get_parameter("vehicle_config").value
        if (config_path == ''):
            self.get_logger().fatal(
                "vehicle config file param not set or empty, using assumed thruster count of 8")
            return 8
        with open(config_path, 'r') as stream:
            config_file = yaml.safe_load(stream)
        thruster_info = config_file['thrusters']
        return len(thruster_info)

    def wait_for_input(self):
        # Goes into loop until Node has recieved a trigger command
        self.triggered = False
        while rclpy.ok() and not self.triggered:
            pass

    #######################################
    #    Callback and timer functions     #
    #######################################

    def publish_dshot_command(self):
        # Publishes dshot value to specified thruster and NUETRAL_DSHOT to all other thrusters
        # Function is called on timer to prevent firmware from timing out
        dshot_msg = DshotCommand()
        dshot_values = [NUETRAL_DSHOT]*self.thruster_length
        dshot_values[self.thruster_num] = self.dshot
        dshot_msg.values = dshot_values
        self.dshot_pub.publish(dshot_msg)

    def trigger_callback(self, msg: Empty):
        # Indicated user has triggered to move on in code
        self.triggered = True

    def rpm_callback(self, msg):
        # Adds rpm data to list if data is collecting
        if self.collecting_data:
            self.rpm.append(msg[self.thruster_num])

    def force_callback(self, msg: Int32):
        # Adds force data to list if data is collecting
        if self.collecting_data:
            self.force.append(msg)

    ##########################################
    #     Boiler plate action functions      #
    ##########################################

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


def main(args=None):
    rclpy.init(args=args)

    thruster_cal_action_server = CalibrateThrusterAction()

    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_cal_action_server, executor=executor)

    thruster_cal_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
