import numpy as np

#Constants?
base_init       = np.pi/3       #angular offset form x-axis to first pair of actuator bases
base_interval   = 2*np.pi/3     #angular interval between pairs of actuator bases
base_offset     = np.pi/6       #angular offset between actuator bases in a pair

phi             = np.pi/3       #angle between actuator and z-axis (vertical)

head_init       = 0             #angular offset from x-axis to first pair of actuator heads
head_interval   = 2*np.pi/3     #angular interval between pairs of actuator heads
head_offset     = np.pi/6       #angular offset between actuator heads in a pair

radius          = 1             #distance to actuator target from center
thickness       = 2             #thickness of the platform
joint_height    = 1             #vertical distance between actuator head and bottom of platform

precision       = pow(10, 4)    #number of sig figs to keep

#given the magnitudes of the forces produced by the linear actuators, calculate the resultant wrench on the platform

def calc_wrench_1(magnitudes: np.ndarray) -> np.ndarray:

    #x,y,z
    forces = np.ndarray((6,3))

    i = 0
    while(i < 6):
        dir = pow(-1,i)

        y_factor = np.sin(head_init + dir*head_offset/2 + head_interval*(np.floor((i+1)/2))) - np.sin(base_init + dir*base_offset/-2 + base_interval*(np.floor(i/2)))
        x_factor = np.cos(head_init + dir*head_offset/2 + head_interval*(np.floor((i+1)/2))) - np.cos(base_init + dir*base_offset/-2 + base_interval*(np.floor(i/2)))
        theta = np.arctan2(y_factor, x_factor)
        
        forces[i][0] = magnitudes[i] * np.cos(theta) * np.sin(phi)
        forces[i][1] = magnitudes[i] * np.sin(theta) * np.sin(phi)
        forces[i][2] = magnitudes[i]                 * np.cos(phi)
        i+=1
    
    return calc_wrench_2(forces)
    end


#given the components of the forces produced by the linear actuators, calculate the resultant wrench on the platform

def calc_wrench_2(forces : np.ndarray) -> np.ndarray:

    #x,y,z
    distances = np.ndarray((6,3))

    i = 0
    while(i < 6):
        dir = pow(-1,i)

        distances[i][0] = radius * np.cos(head_init + dir*head_offset/2 + head_interval*(np.floor((i+1)/2)))
        distances[i][1] = radius * np.sin(head_init + dir*head_offset/2 + head_interval*(np.floor((i+1)/2)))
        distances[i][2] = -thickness/2 - joint_height

        i+=1

    return calc_wrench_3(forces, distances)
    end


#given the components and positions of the forces produced by the linear actuators, calculate the resultant wrench on the platform

def calc_wrench_3(forces: np.ndarray, distances: np.ndarray) -> np.ndarray:
    
    #pitch, roll, yaw
    torques = np.cross(distances, forces)

    #x, y, z, pitch, roll, yaw
    wrenches = np.hstack((forces, torques))

    resultant_wrench = np.round_(precision*np.sum(wrenches, 0), decimals = 0)/precision
    
    print(np.array2string(resultant_wrench))
    return resultant_wrench
    end

i = 0
while(i < 100):
    calc_wrench_1(np.array(np.random.random((6,1))))
    i+=1
calc_wrench_1(np.array([2,2,2,2,2,2]))