#!/bin/python3

from math import sqrt
import numpy as np
import matplotlib.pyplot as plt

def main():

    # list of waypoints for the generator to make
    waypts = [
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 1.0]),
        np.array([1.0, 1.0, 2.0]),
        np.array([1.0, 1.0, 3.0]),
        np.array([1.0, 4.0, 3.0]),
        np.array([-1.0, 2.0, 1.0])
    ]

    # spline = spline_path_sampled(waypts, 50)
    spline = spline_path_spaced(waypts, 5)
    splineArr = np.array(spline)

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
    

def spline_path_gen(points: list, numSamples: int, distances: list, totalDistance: float) -> list:
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
        
    # re-iterate to generate curve from 1 to n-1
    for i in range(1, len(points)):
        # calculate num samples for this portion based on pct of total distance between pts
        locSamples = round(distances[i-1] / totalDistance * numSamples)

        # generate the sample indicies
        locSamplePts = np.linspace(0.0, 1.0, locSamples)

        # sample the spline and save
        for samplePt in locSamplePts:
            sampledPts.append(spline_interp(points[i-1], tangents[i-1], points[i], tangents[i], samplePt))
    
    return sampledPts


# helper function for getting radial distance (l2 norm) between 2 vectors
def radial_dist(point1: np.ndarray, point2: np.ndarray) -> float:
    return np.linalg.norm(point1 - point2)

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
