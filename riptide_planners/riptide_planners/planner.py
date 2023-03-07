#!/bin/python3

import numpy as np
from transforms3d import quaternions as quat
import math

# now rclpy and messages
import rclpy
from rclpy.node import Node
from rclpy.time import Duration, Time
from riptide_msgs2.srv import PlanPath
from riptide_msgs2.msg import TrajPoint
from geometry_msgs.msg import Pose, Quaternion, Point 
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose_stamped

# also need TF to support common frame
from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

def main():
    rclpy.init()

    node = PlannerNode()

    rclpy.spin(node)

    rclpy.shutdown()



class PlannerNode(Node):
    def __init__(self):
        super().__init__('riptide_planner')

        # create the TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # create the service server
        self.service = self.create_service(PlanPath, "plan_path", self.handleRequest)

        # parameters
        self.common_frame = ""
        self.interp_density = 15
        self.max_lin_accel = 1.0
        self.max_lin_vel = 1.0
        self.max_ang_accel = 1.0
        self.max_ang_vel = 1.0
        self.traj_max_exec_time = 50.0

        self.get_logger().info("Started riptide_planner")

    def handleRequest(self, request: PlanPath.Request, response: PlanPath.Response):
        points = []
        orients = []

        # handle empty request
        if len(request.path_points) < 2:
            response.error_code = PlanPath.Response.BAD_WYPTS
            response.error_msg = "Waypoints list is empty"

            self.get_logger().error("Recieved request with too few waypoints")

            return response

        self.get_logger().info("Processing path plan")

        # run the tf lookup back to a common frame
        for point in request.path_points:
            # Look up for the transformation between target_frame and turtle2 frames
            # and send velocity commands for turtle2 to reach target_frame
            tf = TransformStamped()
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.common_frame,
                    point.header.frame_id,
                    self.get_clock().now())
            except TransformException as ex:
                self.get_logger().error(
                    f'Could not transform {self.common_frame} to {point.header.frame_id}: {ex}')

                response.error_code = PlanPath.Response.MISSING_FRAME_ID
                response.error_msg = f'Could not transform "{self.common_frame}" to "{point.header.frame_id}": {ex}'
                return response

            tfPoint = do_transform_pose_stamped(point, tf)

            # put it on the list
            points.append(np.array([tfPoint.pose.position.x, tfPoint.pose.position.y, tfPoint.pose.position.z]))
            orients.append(np.array([tfPoint.pose.orientation.w, tfPoint.pose.orientation.x, tfPoint.pose.orientation.y, tfPoint.pose.orientation.z]))

        # now interp the path
        position_path, points_per_seg = spline_path_spaced(points, self.interp_density)
        orients_path = []

        for i in range(1, len(orients)):
            sample_pts = points_per_seg[i]
            
            # start and end orientation for lerp
            start = orients[i-1]
            end = orients[i]

            # run the lerp for each point in between
            for sampleIdx in range(sample_pts):
                point = sampleIdx / sample_pts

                # quaternion slerp from here https://en.wikipedia.org/wiki/Slerp
                lerped_quat = quat.qmult(quat.qpow(quat.qmult(end, quat.qinverse(start)), point), start)
                orients_path.append(lerped_quat)


        self.get_logger().info("path planned, generating trajectory")

        # convert path to trajectory points
        fullPath = [
            TrajPoint(
                pose=Pose(
                    position=Point(x=position_path[i][0], y=position_path[i][1], z=position_path[i][2]), 
                    orientation=Quaternion(w=orients_path[i][0], x=orients_path[i][1], y=orients_path[i][2], z=orients_path[i][3])
                ),
                header=Header(
                    frame_id=self.common_frame
                )
            ) for i in range(len(position_path))]

        # begin the two pass timing algorthim
        # the forward pass is designed to ramp up to max velocities
        # the backward pass is designed to obey constraints and ramp down
        lin_vel = 0.0
        lin_vel_prev = 0.0

        # begin the forward pass
        self.get_logger().info("Beginning trajectory forward pass")
        for i in range(1, len(fullPath)):
            # compute cartesian distance
            distance = point_dist(fullPath[i], fullPath[i-1])

            # compute the point velocity magnitude and clamp to below max vel
            lin_vel = math.sqrt(lin_vel_prev ** 2 + 2 * self.max_lin_accel * distance)
            if lin_vel > self.max_lin_vel: 
                lin_vel = self.max_lin_vel

            # TODO scale the unit vector in the twist by the magnitude
            fullPath[i].lin_veloc.linear.x = lin_vel
            
            # update the predecessor velocity
            lin_vel_prev = lin_vel
        

        # now run the backward pass
        self.get_logger().info("Beginning trajectory backward pass")
        for i in range(0, len(fullPath))[::-1]:
            # set the final point to zero velocity
            if i == len(fullPath) - 1:
                print(i, 0)
                fullPath[i].lin_veloc.linear.x = 0.0
                continue

            # compute cartesian distance
            distance = point_dist(fullPath[i], fullPath[i-1])

            # compute the point velocity magnitude and clamp to below max vel
            lin_vel = math.sqrt(fullPath[i+1].lin_veloc.linear.x ** 2 + 2 * self.max_lin_accel * distance)
            if lin_vel < fullPath[i].lin_veloc.linear.x:
                # this means we need to retime as this point has been adjusted down

                print(f"adjusting {i} to vel {lin_vel}")

                if lin_vel > self.max_lin_vel: 
                    lin_vel = self.max_lin_vel

                # TODO scale the unit vector in the twist by the magnitude
                fullPath[i].lin_veloc.linear.x = lin_vel

        # run the timing pass
        time_elapsed = 0.0
        plan_start = self.get_clock().now()

        # assign for the first point
        fullPath[0].header.stamp = plan_start.to_msg()

        for i in range(1, len(fullPath)):
            # compute cartesian distance
            distance = point_dist(fullPath[i], fullPath[i-1])

            # compute the time difference caused by this motion
            time_delta = (2 / (fullPath[i].lin_veloc.linear.x  + fullPath[i-1].lin_veloc.linear.x)) * distance
            time_elapsed += time_delta

            # set the time point
            elapsed_sec = int(time_elapsed)
            elapsed_nanos = int((time_elapsed - elapsed_sec)*1e9)
            real_time = (plan_start+Duration(seconds=elapsed_sec, nanoseconds=elapsed_nanos)).to_msg()
            fullPath[i].header.stamp = real_time
            
            # make sure that the trajectory isnt taking too much time
            if time_elapsed > self.traj_max_exec_time:
                self.get_logger().warning(f"Trajectory duration too long. Excceded limit of {self.traj_max_exec_time}s while still planning")

                response.traj_points = []
                response.error_code = PlanPath.Response.BAD_WYPTS
                response.error_msg = f"Trajectory duration exceeds max allowable set at {self.traj_max_exec_time}s"

                # send the result
                return response


        self.get_logger().info("Trajectory complete")

        # time the trajectory
        response.traj_points = fullPath
        response.error_code = PlanPath.Response.NO_ERROR
        response.error_msg = ""

        # send the result
        return response
            

# spline path but given the number of samples to make along the curve
def spline_path_sampled(points: list, numSamples: int) -> list:
    # accumulate radial linear dist of path
    distances = []
    for i in range(1, len(points)):
        distances.append(radial_dist(points[i-1], points[i]))
    totalDistance = np.sum(distances)

    # run the path generator
    return spline_path_gen(points, numSamples, distances, totalDistance)

# spline path but given the max sampling ratio in samples / m travelled
# samples may be closer together than max sampling ratio
def spline_path_spaced(points: list, samples_per_m: int) -> list:
    # accumulate radial linear dist of path
    distances = []
    for i in range(1, len(points)):
        distances.append(radial_dist(points[i-1], points[i]))
    totalDistance = np.sum(distances)

    # figure out the number of samples needed for this spacing
    numSamples = int(totalDistance * samples_per_m)

    # run the path gen
    return spline_path_gen(points, numSamples, distances, totalDistance)
    

def spline_path_gen(points: list, numSamples: int, distances: list, totalDistance: float) -> tuple:
    # dont have to check for colinearity at all
    # the spline will lerp on its own if needed
    tangents = []

    # assign the pt 0 tangent to be the vector between pt 0 and pt 1
    tangentVect = np.array([
        points[1][0] - points[0][0],
        points[1][1] - points[0][1],
        points[1][2] - points[0][2]
    ])
    tangentVect = tangentVect / np.linalg.norm(tangentVect)
    tangents.append(np.array([tangentVect[0], tangentVect[1], tangentVect[2]]))

    # assign the pt n tangent to be the vector between pt n-1 and pt n
    tangentVect = np.array([
        points[len(points)-1][0] - points[len(points)-2][0],
        points[len(points)-1][1] - points[len(points)-2][1],
        points[len(points)-1][2] - points[len(points)-2][2]
    ])
    tangentVect = tangentVect / np.linalg.norm(tangentVect)
    tangents.append(np.array([tangentVect[0], tangentVect[1], tangentVect[2]]))

    
    # iterate from 1 to n-1
    for i in range(1, len(points)-1):
        # calculate vector P from pt k-1 to pt k
        p = np.array([
            points[i][0] - points[i-1][0],
            points[i][1] - points[i-1][1],
            points[i][2] - points[i-1][2]
        ])
        p = p / np.linalg.norm(p)

        # calculate vector A from pt k to pt k+1
        a = np.array([
            points[i+1][0] - points[i][0],
            points[i+1][1] - points[i][1],
            points[i+1][2] - points[i][2]
        ])
        a = a / np.linalg.norm(a)

        # calculate raidal distance dk from pt k-1 to pt k
        dk = distances[i-1]
        # calculate radial distance dk+1 from pt k to pt k+1
        dk1 = distances[i]

        # assign tangent at pk according to A * dk / dk+1 + p * dk+1 / dk 
        tangentK = a * dk / dk1 + p * dk1 / dk
        tangentK = tangentK / np.linalg.norm(tangentK)

        # save the tangent to the list of tangents
        tangents.insert(-1, np.array([tangentK[0], tangentK[1], tangentK[2]]))

    sampledPts = []
    samplesPerSeg = len(points)*[None] # need to pre-init this
        
    # re-iterate to generate curve from 1 to n-1
    for i in range(1, len(points)):
        # calculate num samples for this portion based on pct of total distance between pts
        samplesPerSeg[i] = round(distances[i-1] / totalDistance * numSamples)

        # generate the sample indicies
        locSamplePts = np.linspace(0.0, 1.0, samplesPerSeg[i])

        # sample the spline and save
        for samplePt in locSamplePts:
            sampledPts.append(spline_interp(points[i-1], tangents[i-1], points[i], tangents[i], samplePt))
    
    return (sampledPts, samplesPerSeg)


# helper function for getting radial distance (l2 norm) between 2 vectors
def radial_dist(point1: np.ndarray, point2: np.ndarray) -> float:
    return np.linalg.norm(point1 - point2)

def point_dist(point1: TrajPoint, point2: TrajPoint) -> float:
    return radial_dist(
        np.array([point1.pose.position.x, point1.pose.position.y, point1.pose.position.z]),
        np.array([point2.pose.position.x, point2.pose.position.y, point2.pose.position.z])
    )


# cubic hermite interpolation function. Can extend to any dimensional order
# tested with 2d and 3d cases first but should extend to higher dimensionality
def spline_interp(position1: np.ndarray, tangent1: np.ndarray,
                  position2: np.ndarray, tangent2: np.ndarray, t: float) -> np.ndarray:

    # timing params
    t2 = t * t
    t3 = t2* t

    P0 = 2.0 * t3 - 3.0 * t2 + 1.0  # partial of position wrt position1
    T0 = t3 - 2.0 * t2 + t          # partial of position wrt tangent1
    P1 = -2.0 * t3 + 3.0 * t2       # partial of position wrt position2
    T1 = t3 - t2                    # partial of position wrt tangent2

    # this computes position
    result = P0 * position1 + T0 * tangent1 + P1 * position2 + T1 * tangent2

    return result

if __name__ == '__main__':
    main()
