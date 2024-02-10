#! /usr/bin/env python3

import evdev
from evdev import categorize, ecodes
import asyncio
from queue import Queue
import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import Float32, Bool, UInt16
from geometry_msgs.msg import Twist
from riptide_msgs2.msg import ControllerCommand
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from tf_transformations import quaternion_from_euler
from time import sleep

#Rocket Leauge drive?

eventQueue = Queue()

#ROS node def
class RocketLeague(Node):

    boost_triggered = False
    stunt_triggered = False
    current_target_depth = 0.0
    x_nudge = 0.0
    y_nudge = 0.0

    def __init__(self, queue: Queue):
        super().__init__("RocketLeague")

        self.create_timer(0.033, self.handleEvents)

        self.event_queue = queue

        self.left_joy_x = 0
        self.left_joy_y = 0
        self.left_trigger = 0
        self.right_joy_x = 0
        self.right_joy_y = 0
        self.right_trigger = 0

        #create pubs
        self.throttle_pub = self.create_publisher(Float32, "/talos/teleop/throttle", qos_profile=qos_profile_sensor_data)
        self.brake_pub = self.create_publisher(Float32, "/talos/teleop/brake", qos_profile=qos_profile_sensor_data)
        self.joy_L_X_pub = self.create_publisher(Float32, "/talos/teleop/joy_R_X", qos_profile=qos_profile_sensor_data)
        self.set_point_pub_linear = self.create_publisher(ControllerCommand, "/talos/controller/linear", qos_profile=qos_profile_system_default)
        self.set_point_pub_angular = self.create_publisher(ControllerCommand, "/talos/controller/angular", qos_profile=qos_profile_system_default)
        self.apply_nudge_pub = self.create_publisher(Twist, "/talos/teleop/apply_nudges", qos_profile=qos_profile_sensor_data)

        #action pubs
        self.boost_trigger_pub = self.create_publisher(Bool, "/talos/teleop/trigger_boost", qos_profile=qos_profile_system_default)
        self.stunt_trigger_pub = self.create_publisher(UInt16, "/talos/controller/stunt_state", qos_profile=qos_profile_system_default)

        #parameters
        self.declare_parameter("default_depth", value= 1.0)
        self.declare_parameter("depth_increment", value= 0.1)
        self.declare_parameter("roll_max", value = .2 )
        self.declare_parameter("pitch_max", value = .2 )

        #load in params
        self.load_parameters()

        #set parameter callback
        self.add_on_set_parameters_callback(self.load_parameters)

    def load_parameters(self):
        #load in parameters

        self.default_depth = self.get_parameter("default_depth").value
        self.depth_increment = self.get_parameter("depth_increment").value
        self.roll_max = self.get_parameter("roll_max").value
        self.pitch_max = self.get_parameter("pitch_max").value

    def handleEvents(self):
        #self.get_logger().info("Here")

        #go through events in queue
        while not self.event_queue.empty():
            event = eventQueue.get()

            if(event.type == ecodes.EV_ABS):
                #event is axis

                #map axis and normalize -1 to 1
                match event.code:

                    case ecodes.ABS_X:
                        self.left_joy_x = (event.value - 128) / 128

                        #apply a deazone 
                        if(abs(self.left_joy_x) < .05):
                            self.left_joy_x = 0

                    case ecodes.ABS_Y:
                        self.left_joy_y = -(event.value - 128) / 128

                        #apply a deazone 
                        if(abs(self.left_joy_y) < .05):
                            self.left_joy_y = 0

                    case ecodes.ABS_Z:
                        self.left_trigger = (event.value) / 255
                    case ecodes.ABS_RX:
                        self.right_joy_x = (event.value - 128) / 128
                    case ecodes.ABS_RY:
                        self.right_joy_y = -(event.value - 128) / 128
                    case ecodes.ABS_RZ:
                        self.right_trigger = (event.value) / 255
                
            elif event.type == ecodes.EV_KEY:
                #event is a button
                match event.code:
                    case ecodes.BTN_A:
                        #handle X button

                        self.get_logger().info("1")
                    case ecodes.BTN_B:
                        #handle circle button
                        if(event.value == 1):
                            if(self.stunt_triggered == False):
                                #barrel roll
                                msg = UInt16()
                                msg.data = 2
                                self.stunt_trigger_pub.publish(msg)

                                self.stunt_cancel_timer = self.create_timer(1, self.detriggerStunt)

                                self.stunt_triggered = True

                    case ecodes.BTN_X:
                        #handle triangle button

                        if(event.value == 1):
                            if(self.stunt_triggered == False):
                                #front roll
                                msg = UInt16()
                                msg.data = 1
                                self.stunt_trigger_pub.publish(msg)

                                self.stunt_cancel_timer = self.create_timer(1, self.detriggerStunt)

                                self.stunt_triggered = True

                    case ecodes.BTN_Y:
                        #handle square button
                        self.get_logger().info("4")


                    case ecodes.BTN_Z:
                        #handle A button
                        self.get_logger().info("5")


                    case ecodes.BTN_THUMBR:
                        #handle right joy button
                        if(event.value == 1):
                            #handle left bumper button
                            self.current_target_depth =  self.current_target_depth - self.depth_increment 

                    case ecodes.BTN_THUMBL:
                        #handle left joy button
                        if(event.value == 1):
                            self.current_target_depth = self.current_target_depth + self.depth_increment       

                            #don't try to go above zero
                            if(self.current_target_depth > 0):
                                self.current_target_depth = 0.0

                    case ecodes.BTN_TL:
                        self.get_logger().info("7")

                    case ecodes.BTN_TR:
                        #handle right bumper button Int16
                        #trigger boost
                        if(event.value == 1):
                            if(not self.boost_triggered):
                                #mark boost as triggered
                                self.boost_triggered = True

                                #pub msg to trigger boost
                                boost_msg = Bool()
                                boost_msg.data = True
                                self.boost_trigger_pub.publish(boost_msg)

                                print(event.value)


                                self.boost_cancel_timer = self.create_timer(5, self.detriggerBoost)

                    case ecodes.BTN_DPAD_UP:
                        #handle dpad up
                        if(event.value == 1):
                            self.x_nudge = 1.0
                        elif(self.x_nudge == 1.0):
                            self.x_nudge = 0.0

                    case ecodes.BTN_DPAD_DOWN:
                        #handle dpad up

                        if(event.value == 1):
                            self.x_nudge = -1.0
                        elif(self.x_nudge == -1.0):
                            self.x_nudge = 0.0

                    case ecodes.BTN_DPAD_LEFT:
                        #handle dpad up

                        if(event.value == 1):
                            self.y_nudge = 1.0
                        elif(self.y_nudge == 1.0):
                            self.y_nudge = 0.0

                    case ecodes.BTN_DPAD_RIGHT:
                        #handle dpad up

                        if(event.value == 1):
                            self.y_nudge = -1.0
                        elif(self.y_nudge == -1.0):
                            self.y_nudge = 0.0      

        #pub values
        msg = Float32()

        #throttle msg
        msg.data = float(self.right_trigger)
        self.throttle_pub.publish(msg)

        #brake msg
        msg.data = float(self.left_trigger)
        self.brake_pub.publish(msg)

        #joy_l_x msg
        msg.data = float(self.right_joy_x)
        self.joy_L_X_pub.publish(msg)

        #pub new setpoint
        set_point_msg_linear = ControllerCommand()
        set_point_msg_angular = ControllerCommand()
        set_point_msg_linear.mode = 3
        set_point_msg_angular.mode = 3

        #calculate orientation quat
        qaut = quaternion_from_euler(self.left_joy_y * self.pitch_max, self.left_joy_x * self.roll_max, 0, 'sxyz')

        #x,y and yaw should be ignored
        set_point_msg_linear.setpoint_vect.x = 0.0  
        set_point_msg_linear.setpoint_vect.y = 0.0
        set_point_msg_linear.setpoint_vect.z = self.current_target_depth

        set_point_msg_angular.setpoint_quat.x = qaut[0]
        set_point_msg_angular.setpoint_quat.y = qaut[1]
        set_point_msg_angular.setpoint_quat.z = qaut[2]
        set_point_msg_angular.setpoint_quat.w = qaut[3]

        self.set_point_pub_linear.publish(set_point_msg_linear)
        self.set_point_pub_angular.publish(set_point_msg_angular)

        #nudge msg
        nudge_msg = Twist()
        nudge_msg.linear.x = self.x_nudge
        nudge_msg.linear.y = self.y_nudge
        nudge_msg.linear.z = 0.0

        self.apply_nudge_pub.publish(nudge_msg)


    def detriggerBoost(self):
        #detrigger the boost topic
        self.boost_cancel_timer.cancel()

        msg = Bool()
        msg.data = False
        self.boost_trigger_pub.publish(msg)

        self.boost_triggered = False

    def detriggerStunt(self):
        #detrigger the boost topic
        self.stunt_cancel_timer.cancel()

        msg = UInt16()
        msg.data = 0
        self.stunt_trigger_pub.publish(msg)

        self.stunt_triggered = False





#read controller async callback
async def read_controller_events():

    controller = None

    while True:

        if controller is None:
            sleep(5.0)

            #connect a controller

            controllerType = "PLAYSTATION(R)3"  #name of the controller trying to find

            devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

            for device in devices:

                if(controllerType in device.name):
                    controller = device

                    print("Controller connected: ", device.path, device.name, device.phys)

        else:
            #controller found

            try:
                async for event in controller.async_read_loop():

                    #add events to be handled by queue
                    eventQueue.put(event)
            except OSError as e:
                print("Disconnected controller, trying again in 5 seconds.")

                controller = None


#start sync
asyncio.ensure_future(read_controller_events())
loop = asyncio.get_event_loop()

#startup ros node
def main(args=None):

    rclpy.init(args=args)

    rl = RocketLeague(eventQueue)

    rclpy.spin(rl)

if __name__ == "__main__":

    thread = Thread(target=main)
    thread.start()

loop.run_forever()



    

