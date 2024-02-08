import evdev
import asyncio
from queue import Queue
import rclpy
from rclpy.node import Node

#Rocket Leauge drive?

#ROS node def
class RocketLauge(Node):

    def __init__(self):
        super().__init__("RocketLauge")

        self.create_timer(lambda:self.get_logger().info("Hello"))

# #Register devices
# controller = None
# controllerType = ""  #name of the controller trying to find

# devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

# for device in devices:
#     print(device.path, device.name, device.phys)

#     if(controllerType in device.name):
#         controller = device

#         #print device capabilites
#         print(controller.capabilities(verbose=True))



# #see if a controler matching the required type was found
# if(controller is None):
#     print(f"Could not find controller of type: {controllerType}. Quitting!")
#     quit()
# else:
#     print(f"Running Rocket Leauge Using: {controller.name}")

# #read controller async callback
# async def read_controller_events(device):
#     async for event in device.async_read_loop():
#         print(repr(event))

# async def periodic():

#     while True:
#         print("This is periodic")
#         await asyncio.sleep(1.0)

# #start sync
# asyncio.ensure_future(read_controller_events(controller))
# loop = asyncio.get_event_loop()
# loop.create_task(periodic())
# loop.run_forever()


#startup ros node
def main(args):
    rclpy.init(args=args)

    rl = RocketLauge()

    rclpy.spin(rl)

if __name__ == "__main__":
    main()



    

