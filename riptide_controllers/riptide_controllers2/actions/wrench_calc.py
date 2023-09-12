import numpy as np

#Constants?
base_init       = np.pi/3       #angular offset from x-axis to first pair of load cell bases
base_interval   = 2*np.pi/3     #angular interval between pairs of load cell bases
base_offset     = np.pi/6       #angular offset between load cell bases in a pair

phi             = np.pi/3       #angle between load cell and z-axis (vertical)

head_init       = 0             #angular offset from x-axis to first pair of load cell heads
head_interval   = 2*np.pi/3     #angular interval between pairs of load cell heads
head_offset     = np.pi/6       #angular offset between load cell heads in a pair

radius          = 1             #distance to load cell head from center
thickness       = 2             #thickness of the platform
joint_height    = 1             #vertical distance between load cell head and bottom of platform
apparent_weight = 5             #(WEIGHT - BUOYANT FORCE) of platform + load

precision       = pow(10, 4)    #number of sig figs to keep

#given the magnitudes of the forces recieved by the load cells, calculate the resultant wrench on the platform

def calc_wrench_1(magnitudes: np.ndarray) -> np.ndarray:

    #x,y,z
    forces = np.ndarray((6,3))

    i = 0
    while(i < 6):
        dir = pow(-1,i)
        
        #find angle of force vector from x-axis (in xy plane)
        y_factor = np.sin(base_init + dir*base_offset/-2 + base_interval*(np.floor(i/2))) - np.sin(head_init + dir*head_offset/2 + head_interval*(np.floor((i+1)/2))) 
        x_factor = np.cos(base_init + dir*base_offset/-2 + base_interval*(np.floor(i/2))) - np.cos(head_init + dir*head_offset/2 + head_interval*(np.floor((i+1)/2)))
        theta = np.arctan2(y_factor, x_factor)
        
        #calculate force components
        forces[i][0] = magnitudes[i] * np.cos(theta) * np.sin(phi)
        forces[i][1] = magnitudes[i] * np.sin(theta) * np.sin(phi)
        forces[i][2] = magnitudes[i]                 *-np.cos(phi)
        i+=1
    
    return calc_wrench_2(forces)
    end


#given the components of the forces recieved by the load cells, calculate the resultant wrench on the platform

def calc_wrench_2(forces : np.ndarray) -> np.ndarray:

    #x,y,z
    distances = np.ndarray((6,3))

    i = 0
    while(i < 6):
        dir = pow(-1,i)

        #calculate distance vectors from center of platform to the load cells
        distances[i][0] = radius * np.cos(head_init + dir*head_offset/2 + head_interval*(np.floor((i+1)/2)))
        distances[i][1] = radius * np.sin(head_init + dir*head_offset/2 + head_interval*(np.floor((i+1)/2)))
        distances[i][2] = -thickness/2 - joint_height

        i+=1

    return calc_wrench_3(forces, distances)
    end


#given the components and positions of the forces recieved by the load cells, calculate the resultant wrench on the platform

def calc_wrench_3(forces: np.ndarray, distances: np.ndarray) -> np.ndarray:
    
    #calculate torques (tau = r x F)
    #pitch, roll, yaw
    torques = np.cross(distances, forces)

    #stack forces on torques
    #x, y, z, pitch, roll, yaw
    wrenches = np.hstack((forces, torques))

    #sum wrenches of each load cell + trim to precision
    resultant_wrench = np.sum(wrenches, 0)
    
    #account for weight of platform and load
    resultant_wrench[2] += apparent_weight

    #trim to precision
    resultant_wrench = np.round_(precision*resultant_wrench, decimals=0)/precision

    return resultant_wrench
    end

#calibrate for apparent weight
def zeroWeight(zeroWeight : float):
    global apparent_weight 
    apparent_weight = zeroWeight
    return
    end

#testing
i = 0
while(i < 100):
    #print(np.array2string(calc_wrench_1(np.array(np.random.random((6,1))))) + "\n")
    i+=1
print(np.array2string(calc_wrench_1(np.array([-2,2,-2,2,-2,2]))) + "\n")