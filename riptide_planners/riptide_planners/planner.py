#!/bin/python3

from geometry_msgs.msg import Pose, Quaternion
from transforms3d.euler import euler2quat
import numpy as np
import matplotlib.pyplot as plt

def main():
    pose1 = Pose()
    pose1.position.x = 0.0
    pose1.position.y = 0.0
    pose1.position.z = 0.0
    orient1 = euler2quat(0, 0, 0)
    pose1.orientation.w = orient1[0]
    pose1.orientation.x = orient1[1]
    pose1.orientation.y = orient1[2]
    pose1.orientation.z = orient1[3]
    

    pose2 = Pose()
    pose2.position.x = 1.0
    pose2.position.y = 0.0
    pose2.position.z = 1.0
    orient2 = euler2quat(3.14, 0, 0)
    pose2.orientation.w = orient2[0]
    pose2.orientation.x = orient2[1]
    pose2.orientation.y = orient2[2]
    pose2.orientation.z = orient2[3]

    

    splinePts = []

    for t in np.linspace(0.0, 1.0, 50):
        splPose = spline_interp(pose1, pose2, t)
        splinePts.append([
            splPose.position.x, splPose.position.y, splPose.position.z, 
            splPose.orientation.x, splPose.orientation.y, 
            splPose.orientation.z, splPose.orientation.w
        ])

    splineArr = np.array(splinePts)

    print(pose1)

    print(splinePts[24])

    print(pose2)

    # Creating an empty figure
    # or plot
    fig = plt.figure()
    ax = plt.axes(projection="3d")


    ax.plot3D(splineArr[:, 0], splineArr[:, 1], splineArr[:, 2], 'red')
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
 
    # Showing the above plot
    plt.show()
    


def spline_interp(pose1: Pose, pose2: Pose, t: float) -> Pose:
    result = Pose()

    # timing params
    t2 = t * t
    t3 = t2* t

    P0 = 2.0 * t3 - 3.0 * t2 + 1.0  # partial of position wrt position1
    T0 = t3 - 2.0 * t2 + t          # partial of position wrt tangent1
    P1 = -2.0 * t3 + 3.0 * t2       # partial of position wrt position2
    T1 = t3 - t2                    # partial of position wrt tangent2

    # this computes position
    result.position.x = P0 * pose1.position.x + T0 * pose1.orientation.x + P1 * pose2.position.x + T1 * pose2.orientation.x
    result.position.y = P0 * pose1.position.y + T0 * pose1.orientation.y + P1 * pose2.position.y + T1 * pose2.orientation.y
    result.position.z = P0 * pose1.position.z + T0 * pose1.orientation.z + P1 * pose2.position.z + T1 * pose2.orientation.z

    # this computes orientation in axis angle
    result.orientation.x = T0 * pose1.orientation.x + T1 * pose2.orientation.x
    result.orientation.y = T0 * pose1.orientation.y + T1 * pose2.orientation.y
    result.orientation.z = T0 * pose1.orientation.z + T1 * pose2.orientation.z
    result.orientation.w = T0 * pose1.orientation.w + T1 * pose2.orientation.w

    return result

if __name__ == '__main__':
    main()
