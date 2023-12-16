import evdev

#Rocket Leauge drive?

#Register devices
controller = None
controllerType = ""  #name of the controller trying to find

devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

for device in devices:
    print(device.path, device.name, device.phys)

    if(controllerType in device.name):
        controller = device

#see if a controler matching the required type was found
if(controller is None):
    print(f"Could not find controller of type: {controllerType}. Quitting!")
    quit()
else:
    print(f"Running Rocket Leauge Using: {controller.name}")

    

