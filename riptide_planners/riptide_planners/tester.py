#!/bin/python3

import matplotlib.pyplot as plt
import numpy as np
import transforms3d.euler

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from riptide_msgs2.srv import PlanPath
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header


def main():
    rclpy.init()

    node = PlannerNode()

    rclpy.spin_once(node, timeout_sec=0.5)

    node.call_service()

    rclpy.shutdown()



class PlannerNode(Node):
    def __init__(self):
        super().__init__('riptide_planner_tester')

        # create the service client
        self.service = self.create_client(PlanPath, "plan_path")

    def call_service(self):
        # check readiness
        if self.service.service_is_ready():
            request = PlanPath.Request()
            request.path_points = [
                    PoseStamped(
                        header = Header(frame_id=""),
                        pose=Pose(
                            position=Point(x = 1.0),
                            orientation=Quaternion(z=0.8509035, w=0.525322)
                        )
                    ),
                    PoseStamped(
                        header = Header(frame_id=""),
                        pose=Pose(position=Point(x = 0.0))
                    ),
                    PoseStamped(
                        header = Header(frame_id=""),
                        pose=Pose(
                            position=Point(y = 2.0),
                            orientation=Quaternion()
                        )
                    )
            ]

            # send request
            future = self.service.call_async(request)

            self.get_logger().info("Sent request")

            # wait for it!
            recived = False
            while not recived:
                rclpy.spin_once(self, timeout_sec=0.5)
                # print("checking")
                recived = future.done()

                if not rclpy.ok():
                    return

            self.get_logger().info("Response recived")

            # print the results
            result = future.result()
            if result.error_code != 0:
                self.get_logger().error(f"Recived error {result.error_code}: {result.error_msg}")
            
            else:
                print("path planned, visualizing...")

                pathPts = [[point.pose.position.x, point.pose.position.y, point.pose.position.z] for point in result.traj_points]
                splineArr = np.array(pathPts)

                print(splineArr.shape, len(result.traj_points))

                # Creating an empty figure and setup the 3d plot axes
                fig = plt.figure()
                ax = fig.add_subplot(1, 3, 1, projection="3d")

                # plot the resulting curve in either a line or a scatter 
                # (depends on what info you want)
                ax.scatter(splineArr[1:-1, 0], splineArr[1:-1, 1], splineArr[1:-1, 2], "b")
                ax.scatter(splineArr[0, 0], splineArr[0, 1], splineArr[0, 2], "g")
                ax.scatter(splineArr[-1, 0], splineArr[-1, 1], splineArr[-1, 2], "r")

                ax.set_xlabel("x (m)")
                ax.set_ylabel("y (m)")
                ax.set_zlabel("z (m)")

                # set the aspect ratio -- has to be after data plotting to work properly
                ax.set_aspect('equal')

                ax.set_title("Planned Path")

                print(result.traj_points[0].header, result.traj_points[-1].header)

                # get the timing info for the rest
                start_stamp = Time.from_msg(result.traj_points[0].header.stamp)
                times = [(Time.from_msg(point.header.stamp) - start_stamp).nanoseconds * 1.0e-9 for point in result.traj_points]

                # now do the orientation
                orientRaw = [transforms3d.euler.quat2euler((
                        point.pose.orientation.w, point.pose.orientation.x, 
                        point.pose.orientation.y, point.pose.orientation.z, 
                    )) for point in result.traj_points ]

                orients = np.array(orientRaw)

                # plot the orientation
                ax = fig.add_subplot(1, 3, 2)
                ax.grid()
                ax.scatter(times, orients[:, 0])
                ax.scatter(times, orients[:, 1])
                ax.scatter(times, orients[:, 2])
                ax.legend(["roll", "pitch", "yaw"])
                ax.set_xlabel("time (s)")
                ax.set_ylabel("rotation (radians)")

                velRaw = [[
                        point.lin_veloc.linear.x, point.lin_veloc.linear.y, point.lin_veloc.linear.z, 
                    ] for point in result.traj_points ]
                
                vels = np.array(velRaw)

                # plot the velocity
                ax = fig.add_subplot(1, 3, 3)
                ax.grid()
                ax.scatter(times, vels[:, 0])
                ax.scatter(times, vels[:, 1])
                ax.scatter(times, vels[:, 2])
                ax.legend(["vx", "vy", "vz"])
                ax.set_xlabel("time (s)")
                ax.set_ylabel("velocity (m/s)")
            
                # Showing the above plot
                plt.show()


        else:
            self.get_logger().fatal("Service not ready")

        

if __name__ == '__main__':
    main()
