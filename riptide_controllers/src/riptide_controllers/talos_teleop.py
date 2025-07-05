#! /usr/bin/env python3

import evdev
from evdev import ecodes
import asyncio
from queue import Queue
import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import Float32, Bool, UInt16, Empty
from geometry_msgs.msg import Twist
from riptide_msgs2.msg import ControllerCommand
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from tf_transformations import quaternion_from_euler
from time import sleep

#Rocket Leauge drive?

eventQueue = Queue() # the queue for events
typeQueue = Queue() # thread saf4e object representing the type of the queue

#controller mappings - dependent on controller
mappings = {
    "PLAYSTATION(R)3":{
        "left_joy_x":ecodes.ABS_X,
        "left_joy_y":ecodes.ABS_Y,
        "left_trigger":ecodes.ABS_Z,        
        "right_joy_x":ecodes.ABS_RX,
        "right_joy_y":ecodes.ABS_RY,
        "right_trigger":ecodes.ABS_RZ,
        "a_btn":ecodes.BTN_A,        
        "b_btn":ecodes.BTN_B,        
        "x_btn":ecodes.BTN_X,        
        "y_btn":ecodes.BTN_Y,
        "z_btn":ecodes.BTN_Z,
        "left_bumper":ecodes.BTN_TL,
        "right_bumper":ecodes.BTN_TR,
        "left_joy_btn":ecodes.BTN_THUMBL,
        "right_joy_btn":ecodes.BTN_THUMBR,
        "dpad_up":ecodes.BTN_DPAD_UP,
        "dpad_down":ecodes.BTN_DPAD_DOWN,
        "dpad_left":ecodes.BTN_DPAD_LEFT,
        "dpad_right":ecodes.BTN_DPAD_RIGHT,
    },
    "SHANWAN":{
        "left_joy_x":ecodes.ABS_Z,
        "left_joy_y":ecodes.ABS_RX,
        "left_trigger":ecodes.ABS_BRAKE,        
        "right_joy_x":ecodes.ABS_X,
        "right_joy_y":ecodes.ABS_Y,
        "right_trigger":ecodes.ABS_GAS,
        "a_btn":ecodes.BTN_A,        
        "b_btn":ecodes.BTN_B,        
        "x_btn":ecodes.BTN_X,        
        "y_btn":ecodes.BTN_Y,
        "z_btn":ecodes.BTN_Z,
        "left_bumper":ecodes.BTN_TL,
        "right_bumper":ecodes.BTN_TR,
        "left_joy_btn":ecodes.BTN_THUMBL,
        "right_joy_btn":ecodes.BTN_THUMBR,
        "dpad_up":ecodes.BTN_DPAD_UP,
        "dpad_down":ecodes.BTN_DPAD_DOWN,
        "dpad_left":ecodes.BTN_DPAD_LEFT,
        "dpad_right":ecodes.BTN_DPAD_RIGHT,
    }
}

#print off keys 
# keys = ecodes.ecodes.keys()
# for key in keys:

#     try:
#         print(f"{key}: {ecodes.ecodes[key]}")
#     except:
#         print(f"Failed to find key{key}")

#ROS node def
class RocketLeague(Node):

    boost_triggered = False
    stunt_triggered = False
    current_target_depth = 0.0
    x_nudge = 0.0
    y_nudge = 0.0

    def __init__(self, queue: Queue, typeQueue: Queue):
        super().__init__("RocketLeague")

        self.create_timer(0.033, self.handleEvents)

        self.event_queue = queue
        self.type_queue = typeQueue
        self.controller_type = None

        self.left_joy_x = 0
        self.left_joy_y = 0
        self.left_trigger = 0
        self.right_joy_x = 0
        self.right_joy_y = 0
        self.right_trigger = 0

        #create pubs
        self.throttle_pub = self.create_publisher(Float32, "/talos/teleop/throttle", qos_profile=qos_profile_sensor_data)
        self.reverse_pub = self.create_publisher(Float32, "/talos/teleop/reverse", qos_profile=qos_profile_sensor_data)
        self.joy_L_X_pub = self.create_publisher(Float32, "/talos/teleop/joy_R_X", qos_profile=qos_profile_sensor_data)
        self.set_point_pub_linear = self.create_publisher(ControllerCommand, "/talos/controller/linear", qos_profile=qos_profile_system_default)
        self.set_point_pub_angular = self.create_publisher(ControllerCommand, "/talos/controller/angular", qos_profile=qos_profile_system_default)
        self.apply_nudge_pub = self.create_publisher(Twist, "/talos/teleop/apply_nudges", qos_profile=qos_profile_sensor_data)

        #action pubs
        self.boost_trigger_pub = self.create_publisher(Bool, "/talos/teleop/trigger_boost", qos_profile=qos_profile_system_default)
        self.stunt_trigger_pub = self.create_publisher(UInt16, "/talos/controller/stunt_state", qos_profile=qos_profile_system_default)
        self.torpedo_trigger_pub = self.create_publisher(Empty, "/talos/command/simple_torpedo_fire", qos_profile_system_default)
        self.dropper_trigger_pub = self.create_publisher(Empty, "/talos/command/simple_dropper_fire", qos_profile_system_default)

        #parameters
        self.declare_parameter("default_depth", value= 1.0)
        self.declare_parameter("depth_increment", value= 0.1)
        self.declare_parameter("roll_max", value = .5 )
        self.declare_parameter("pitch_max", value = .5 )

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

        #check for a new controller type
        while not self.type_queue.empty():
            self.controller_type = self.type_queue.get()

        #don't attempt to handle events without a known controller type
        if(self.controller_type is None):
            return

        #go through events in queue
        while not self.event_queue.empty():
            event = eventQueue.get()

            #print(event.code)

            if(event.type == ecodes.EV_ABS):
                #event is axis

                #map axis and normalize -1 to 1
                if(event.code == mappings[self.controller_type]["left_joy_x"]):
                    self.left_joy_x = -(event.value - 128) / 128

                    #apply a deazone 
                    if(abs(self.left_joy_x) < .05):
                        self.left_joy_x = 0

                elif(event.code == mappings[self.controller_type]["left_joy_y"]):
                    self.left_joy_y = -(event.value - 128) / 128

                    #apply a deazone 
                    if(abs(self.left_joy_y) < .05):
                        self.left_joy_y = 0

                elif(event.code == mappings[self.controller_type]["left_trigger"]):
                    self.left_trigger = (event.value) / 255

                elif(event.code == mappings[self.controller_type]["right_joy_x"]):
                    self.right_joy_x = -(event.value - 128) / 128

                elif(event.code == mappings[self.controller_type]["right_joy_y"]):
                    self.right_joy_y = -(event.value - 128) / 128

                elif(event.code == mappings[self.controller_type]["right_trigger"]):
                    self.right_trigger = (event.value) / 255
                
            elif event.type == ecodes.EV_KEY:
                #event is a button
                if(event.code == mappings[self.controller_type]["a_btn"]):
                    #handle X button - droppper

                    if(event.value == 1):
                        msg = Empty()
                        self.dropper_trigger_pub.publish(msg)


                elif(event.code == mappings[self.controller_type]["b_btn"]):
                    #handle circle button
                    if(event.value == 1):
                        if(self.stunt_triggered == False):
                            #barrel roll
                            msg = UInt16()
                            msg.data = 2
                            self.stunt_trigger_pub.publish(msg)

                            self.stunt_cancel_timer = self.create_timer(1, self.detriggerStunt)

                            self.stunt_triggered = True

                elif(event.code == mappings[self.controller_type]["y_btn"]):
                    #handle triangle button

                    if(event.value == 1):
                        if(self.stunt_triggered == False):
                            #front roll
                            msg = UInt16()
                            msg.data = 1
                            self.stunt_trigger_pub.publish(msg)

                            self.stunt_cancel_timer = self.create_timer(1, self.detriggerStunt)

                            self.stunt_triggered = True

                elif(event.code == mappings[self.controller_type]["x_btn"]):
                    #handle square button - trigger torps
                    if(event.value == 1):
                        msg = Empty()
                        self.torpedo_trigger_pub.publish(msg)


                elif(event.code == mappings[self.controller_type]["z_btn"]):
                    #handle A button - trigger marker

                    self.get_logger().info("Congrats, you have found a wild Z button!")
                elif(event.code == mappings[self.controller_type]["right_joy_btn"]):
                    #handle right joy button
                    if(event.value == 1):
                        #handle left bumper button
                        self.current_target_depth =  self.current_target_depth - self.depth_increment 

                elif(event.code == mappings[self.controller_type]["left_joy_btn"]):
                    #handle left joy button
                    if(event.value == 1):
                        self.current_target_depth = self.current_target_depth + self.depth_increment       

                        #don't try to go above zero
                        if(self.current_target_depth > 0):
                            self.current_target_depth = 0.0

                elif(event.code == mappings[self.controller_type]["left_bumper"]):
                    self.get_logger().info("7")

                elif(event.code == mappings[self.controller_type]["right_bumper"]):
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

                elif(event.code == mappings[self.controller_type]["dpad_up"]):
                    #handle dpad up
                    if(event.value == 1):
                        self.x_nudge = 1.0
                    elif(self.x_nudge == 1.0):
                        self.x_nudge = 0.0

                elif(event.code == mappings[self.controller_type]["dpad_down"]):
                    #handle dpad down

                    if(event.value == 1):
                        self.x_nudge = -1.0
                    elif(self.x_nudge == -1.0):
                        self.x_nudge = 0.0

                elif(event.code == mappings[self.controller_type]["dpad_right"]):
                    #handle dpad left

                    if(event.value == 1):
                        self.y_nudge = -1.0
                    elif(self.y_nudge == -1.0):
                        self.y_nudge = 0.0

                elif(event.code == mappings[self.controller_type]["dpad_left"]):
                    #handle dpad right

                    if(event.value == 1):
                        self.y_nudge = 1.0
                    elif(self.y_nudge == 1.0):
                        self.y_nudge = 0.0      

        #pub values
        msg = Float32()

        #throttle msg
        msg.data = float(self.right_trigger)
        self.throttle_pub.publish(msg)

        #brake msg
        msg.data = float(self.left_trigger)
        self.reverse_pub.publish(msg)

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

            controllerTypes = ["PLAYSTATION(R)3", "SHANWAN"]  #name of the controller trying to find

            devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

            for device in devices:
                for type in controllerTypes:
                    if(type in device.name):
                        controller = device
                        typeQueue.put(type)

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

    rl = RocketLeague(eventQueue, typeQueue)

    rclpy.spin(rl)

if __name__ == "__main__":

    thread = Thread(target=main)
    thread.start()

loop.run_forever()



    

