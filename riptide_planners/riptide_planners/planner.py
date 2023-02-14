#!/bin/python3

from geometry_msgs.msg import Vector3
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt

def main():

    waypts = [
        Vector3(x=0.0, y=0.0, z=0.0),
        Vector3(x=1.0, y=0.0, z=1.0),
        Vector3(x=1.0, y=1.0, z=2.0),
        Vector3(x=1.0, y=1.0, z=3.0),
        Vector3(x=1.0, y=4.0, z=3.0)
    ]

    spline = spline_path(waypts, 500)

    splinePts = []
    for point in spline:
        splinePts.append([point.x, point.y, point.z])

    splineArr = np.array(splinePts)

    print(np.min(splineArr[:, 0]))
    print(np.argmin(splineArr[:, 0]))

    print(splinePts[0])
    print(splinePts[24])
    print(splinePts[-1])

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
    

def spline_path(points: list, samples: int) -> Vector3:
    samplePts = []

    # dont have to check for colinearity at all
    # the spline will lerp on its own if needed

    # accumulate radial linear dist of path
    distances = []
    for i in range(len(points)):
        distances.append(radial_dist(points[i-1], points[i]))
    totalDistance = np.sum(distances)

    tangents = []

    # assign the pt 0 tangent to be the vector between pt 0 and pt 1
    tangentVect = np.array([
        points[1].x - points[0].x,
        points[1].y - points[0].y,
        points[1].z - points[0].z
    ])
    tangentVect = tangentVect / np.linalg.norm(tangentVect)
    tangents.append(Vector3(x=tangentVect[0], y=tangentVect[1], z=tangentVect[2]))

    # assign the pt n tangent to be the vector between pt n-1 and pt n
    tangentVect = np.array([
        points[len(points)-1].x - points[len(points)-2].x,
        points[len(points)-1].y - points[len(points)-2].y,
        points[len(points)-1].z - points[len(points)-2].z
    ])
    tangentVect = tangentVect / np.linalg.norm(tangentVect)
    tangents.append(Vector3(x=tangentVect[0], y=tangentVect[1], z=tangentVect[2]))

    
    # iterate from 1 to n-1
    for i in range(1, len(points)-1):
        # calculate vector P from pt k-1 to pt k
        p = np.array([
            points[i].x - points[i-1].x,
            points[i].y - points[i-1].y,
            points[i].z - points[i-1].z
        ])
        p = p / np.linalg.norm(p)

        # calculate vector A from pt k to pt k+1
        a = np.array([
            points[i+1].x - points[i].x,
            points[i+1].y - points[i].y,
            points[i+1].z - points[i].z
        ])
        a = a / np.linalg.norm(a)

        # calculate raidal distance dk from pt k-1 to pt k
        dk = distances[i]
        # calculate radial distance dk+1 from pt k to pt k+1
        dk1 = distances[i+1]

        # assign tangent at pk according to A * dk / dk+1 + p * dk+1 / dk 
        tangentK = a * dk / dk1 + p * dk1 / dk
        tangentK = tangentK / np.linalg.norm(tangentK)

        # save the tangent to the list of tangents
        tangents.insert(-1, Vector3(x=tangentK[0], y=tangentK[1], z=tangentK[2]))
    
        
    # re-iterate to generate curve from 1 to n-1
    for i in range(1, len(points)):
        # calculate num samples for this portion based on pct of total distance between pts
        numSamples = int(distances[i] / totalDistance * samples)

        # generate the sample indicies
        locSamples = np.linspace(0.0, 1.0, numSamples)

        # sample the spline and save
        for sample in locSamples:
            samplePts.append(spline_interp(points[i-1], tangents[i-1], points[i], tangents[i], sample))
    
    return samplePts



def radial_dist(point1: Vector3, point2: Vector3) -> float:
    return sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)


def spline_interp(position1: Vector3, tangent1: Vector3, position2: Vector3, tangent2: Vector3, t: float) -> Vector3:
    result = Vector3()

    # timing params
    t2 = t * t
    t3 = t2* t

    P0 = 2.0 * t3 - 3.0 * t2 + 1.0  # partial of position wrt position1
    T0 = t3 - 2.0 * t2 + t          # partial of position wrt tangent1
    P1 = -2.0 * t3 + 3.0 * t2       # partial of position wrt position2
    T1 = t3 - t2                    # partial of position wrt tangent2

    # this computes position
    result.x = P0 * position1.x + T0 * tangent1.x + P1 * position2.x + T1 * tangent2.x
    result.y = P0 * position1.y + T0 * tangent1.y + P1 * position2.y + T1 * tangent2.y
    result.z = P0 * position1.z + T0 * tangent1.z + P1 * position2.z + T1 * tangent2.z

    return result

if __name__ == '__main__':
    main()
