from enum import Enum
from ossaudiodev import control_labels
import numpy as np
from abc import ABC, abstractmethod
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_multiply, quaternion_inverse, quaternion_conjugate, quaternion_slerp

def msgToNumpy(msg):
    if hasattr(msg, "w"):
        return np.array([msg.x, msg.y, msg.z, msg.w])
    return np.array([msg.x, msg.y, msg.z])

def worldToBody(orientation, vector):
    """ 
    Rotates world-frame vector to body-frame

    Rotates vector to body frame

    Parameters:
    orientation (np.array): The current orientation of the robot as a quaternion
    vector (np.array): The 3D vector to rotate

    Returns: 
    np.array: 3 dimensional rotated vector

    """

    vector = np.append(vector, 0)
    orientationInv = quaternion_inverse(orientation)
    newVector = quaternion_multiply(orientationInv, quaternion_multiply(vector, orientation))
    return np.array(newVector[:3])

def applyMax(vector, max_vector):
    """ 
    Scales a vector to obey maximums

    Parameters:
    vector (np.array): The unscaled vector
    max_vector (np.array): The maximum values for each element

    Returns: 
    np.array: Vector that obeys the maximums

    """

    scale = 1
    for i in range(len(vector)):
        if abs(vector[i]) > max_vector[i]:
            element_scale = max_vector[i] / abs(vector[i])
            if element_scale < scale:
                scale = element_scale

    return vector * scale

class ControlMode(Enum):
    DISABLED = 0
    FEEDFORWARD = 1
    VELOCITY = 2
    POSITION = 3

class CascadedPController(ABC):

    def __init__(self):
        self.controlMode = ControlMode.DISABLED
        self.setPoint = np.zeros(3)
        self.positionP = np.zeros(3)
        self.velocityP = np.zeros(3)
        self.maxVelocity = np.zeros(3)
        self.maxAccel = np.zeros(3)
        self.steady = True
        self.name = ''

    @abstractmethod
    def computeCorrectiveVelocity(self, odom: Odometry):
        """ 
        Computes a corrective velocity.
    
        If self.targetPosition is not None, will return a corrective body-frame velocity that moves the robot in the direction of self.targetPosiiton. Otherwise returns 0 vector.
    
        Parameters:
        odom (Odometry): The latest odometry message

        Returns: 
        np.array: 3 dimensional vector representing corrective body-frame velocity.
    
        """
        pass

    @abstractmethod
    def computeCorrectiveAcceleration(self, odom: Odometry, correctiveVelocity):
        """ 
        Computes a corrective acceleration.
    
        If self.targetVelocity is not None, will return a corrective body-frame acceleration that transitions the robot twoards the desired self.targetVelocity. Otherwise returns 0 vector.
    
        Parameters:
        correctiveVelocity (np.array): Body-frame velocity vector that adds on to the self.targetVelocity. Is used in position correction.
        odom (Odometry): The latest odometry message

        Returns: 
        np.array: 3 dimensional vector representing corrective body-frame acceleration.
    
        """
        pass

    def setTargetPosition(self, target, mode: ControlMode):
        """ 
        Sets target position
    
        Puts the controller in the Position state and sets self.targetPosition to targetPosition
    
        Parameters:
        targetPosition (np.array or Vector3): World-frame vector or quaternion to be achieved by the controller

        """

        self.setPoint = msgToNumpy(target)
        self.controlMode = mode

    def update(self, odom: Odometry) -> JointState:
        """ 
        Updates the controller
    
        Will compute an output acceleration to achieve the desired state
    
        Parameters:
        odom (Odometry): The latest odometry message

        Returns: 
        np.array: 3 dimensional vector representing net body-frame acceleration.

        """
        state = JointState()
        state.name = [self.name+'_x', self.name+'_y', self.name+'_z']
        netAccel = np.zeros(3)
        correctiveVelocity = np.zeros(3)

        if(self.controlMode == ControlMode.POSITION or self.controlMode == ControlMode.VELOCITY):
            correctiveVelocity = self.computeCorrectiveVelocity(odom)
            netAccel = self.computeCorrectiveAcceleration(odom, correctiveVelocity)

        netAccel = applyMax(netAccel, self.maxAccel)

        # pack the joint state
        state.position = self.setPoint.tolist() if self.controlMode == ControlMode.POSITION else np.zeros(3).tolist()
        state.velocity = self.setPoint.tolist() if self.controlMode == ControlMode.VELOCITY else correctiveVelocity.tolist()
        state.effort = netAccel.tolist()

        if(self.controlMode == ControlMode.POSITION or self.controlMode == ControlMode.VELOCITY):
            self.steady = np.allclose(netAccel, np.zeros(3), atol=0.01) and np.allclose(correctiveVelocity, np.zeros(3), atol=0.01)
        else:
            self.steady = True
    
        return state

class LinearCascadedPController(CascadedPController):

    def __init__(self):
        super(LinearCascadedPController, self).__init__()
        self.name = 'lin'

    def computeCorrectiveVelocity(self, odom: Odometry):
        if self.controlMode == ControlMode.POSITION:
            currentPosition = msgToNumpy(odom.pose.pose.position) # [1 0 0]
            outputVel = (self.setPoint - currentPosition) * self.positionP # [-1 0 1]
            orientation = msgToNumpy(odom.pose.pose.orientation)
            return worldToBody(orientation, outputVel)
        else:
            return np.zeros(3)

    def computeCorrectiveAcceleration(self, odom: Odometry, correctiveVelocity):
        if(self.controlMode == ControlMode.DISABLED or self.controlMode == ControlMode.FEEDFORWARD):
            return np.zeros(3)

        # the target velocity is either the setpoint in velocity control mode, or the corrective velocity if in position mode
        targetVelocity = self.setPoint if(self.controlMode == ControlMode.VELOCITY) else correctiveVelocity
        targetVelocity = applyMax(targetVelocity, self.maxVelocity)
        currentVelocity = msgToNumpy(odom.twist.twist.linear)
        outputAccel = (targetVelocity - currentVelocity) * self.velocityP
        return outputAccel      
        

class AngularCascadedPController(CascadedPController):

    def __init__(self):
        super(AngularCascadedPController, self).__init__()
        self.name = 'ang'

    def computeCorrectiveVelocity(self, odom: Odometry):
        if self.controlMode == ControlMode.POSITION:
            currentOrientation = msgToNumpy(odom.pose.pose.orientation)

            # OLD WAY, only works to find a direction, causes high gains in position
            # Find an orientation in the right direction but with a small angle
            intermediateOrientation = quaternion_slerp(currentOrientation, self.setPoint, 0.01)

            # This math only works for small angles, so the direction is more important
            dq = (intermediateOrientation - currentOrientation)
            outputVel = np.array(quaternion_multiply(quaternion_inverse(currentOrientation), dq)[:3]) * self.positionP
            return outputVel 
        else:
            return np.zeros(3)

    def computeCorrectiveAcceleration(self, odom, correctiveVelocity):
        if(self.controlMode == ControlMode.DISABLED or self.controlMode == ControlMode.FEEDFORWARD):
            return np.zeros(3)

        # the target velocity is either the setpoint in velocity control mode, or the corrective velocity if in position mode
        targetVelocity = self.setPoint if(self.controlMode == ControlMode.VELOCITY) else correctiveVelocity
        targetVelocity = applyMax(targetVelocity, self.maxVelocity)   
        currentVelocity = msgToNumpy(odom.twist.twist.angular)
        outputAccel = (targetVelocity - currentVelocity) * self.velocityP
        return outputAccel 

class AccelerationCalculator:
    def __init__(self, config):
        # general constants
        self.gravity = 9.8 # (m/sec^2)
        self.density = 1000 # density of water (kg/m^3)

        # Vehicle mass properties
        self.com = np.array(config["com"])
        self.mass = np.array(config["mass"])
        self.inertia = np.array(config["inertia"])

        # Bouyancy force calculation
        self.cob = np.array(config["cob"])
        self.volume = np.array(config["volume"])
        self.buoyancy = np.array([0, 0, self.volume * self.density * self.gravity])
        
        # damping force coeficients
        controller = config['controller']
        self.linearDrag = np.array([*controller['linear']['damping']['linear'], *controller['angular']['damping']['linear']])
        self.quadraticDrag = np.array([*controller['linear']['damping']['quadratic'], *controller['angular']['damping']['quadratic']])

        # maximum thruster force that we can request in any direction
        self.maxForce = 4 * config['thruster']["max_force"]

    def accelToNetForce(self, odom, linearAccel, angularAccel):
        """ 
        Converts vehicle acceleration into required net force.
    
        Will take the required acceleration and consider mass, buoyancy, drag, and precession to compute the required net force.
    
        Parameters:
        odom (Odometry): The latest odometry message.
        linearAccel (np.array): The linear body-frame acceleration.
        angularAccel (np.array): The angular body-frame acceleration.

        Returns: 
        np.array: 3 dimensional vector representing net body-frame force.
        np.array: 3 dimensional vector representing net body-frame torque.

        """

        linearVelo = msgToNumpy(odom.twist.twist.linear)
        angularVelo = msgToNumpy(odom.twist.twist.angular)
        orientation = msgToNumpy(odom.pose.pose.orientation)

        # Requested force and torque on the body from controllers
        netForce = linearAccel * self.mass
        netTorque = angularAccel * self.inertia
        
        # calaculation of bouyancy forces
        bodyFrameBuoyancy = worldToBody(orientation, self.buoyancy)
        buoyancyTorque = np.cross((self.cob-self.com), bodyFrameBuoyancy)

        # Torque due to gyroscopic precession on the the current rotation of the vehicle
        precessionTorque = -np.cross(angularVelo, (self.inertia * angularVelo))

        # calculation of the forces due to drag applied on the body
        dragForce = self.linearDrag[:3] * linearVelo + self.quadraticDrag[:3] * abs(linearVelo) * linearVelo
        dragTorque = self.linearDrag[3:] * angularVelo + self.quadraticDrag[3:] * abs(angularVelo) * angularVelo

        # calculation of the gravitational force in the body frame
        gravityForce = worldToBody(orientation, np.array([0, 0, - self.gravity * self.mass]))
                
        # Net Calculation
        netForce = netForce - bodyFrameBuoyancy - dragForce - gravityForce
        netTorque = netTorque - buoyancyTorque - precessionTorque - dragTorque
        
        # make sure the force doesnt exceed maximums of the vehicle
        # THIS IS CURRENTLY DISABLE AS THIS IS AN ARBITRARY LIMIT
        # if(np.linalg.norm(netForce) > self.maxForce):
        #     netForce = netForce / np.linalg.norm(netForce) * self.maxForce

        return netForce, netTorque
