#!/bin/python3

import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from riptide_msgs2.srv import PlanPath
from geometry_msgs.msg import PoseStamped, Pose, Point
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
                        pose=Pose(position=Point(x = 10.0))
                    ),
                    PoseStamped(
                        header = Header(frame_id=""),
                        pose=Pose(position=Point(x = 0.0))
                    )
            ]

            # send request
            future = self.service.call_async(request)

            self.get_logger().info("Sent request")

            # wait for it!
            recived = False
            while not recived:
                rclpy.spin_once(self, timeout_sec=0.5)
                print("checking")
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

                # Creating an empty figure and setup the 3d plot axes
                fig = plt.figure()
                ax = plt.axes(projection="3d")

                # plot the resulting curve in either a line or a scatter 
                # (depends on what info you want)
                ax.scatter(splineArr[:, 0], splineArr[:, 1], splineArr[:, 2])
                # ax.plot3D(splineArr[:, 0], splineArr[:, 1], splineArr[:, 2], 'red')
                ax.set_xlabel("x")
                ax.set_ylabel("y")
                ax.set_zlabel("z")
            
                # Showing the above plot
                plt.show()


        else:
            self.get_logger().fatal("Service not ready")

        

if __name__ == '__main__':
    main()
