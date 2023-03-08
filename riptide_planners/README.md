# Riptide Planners
The riptide planner system is a 6DOF trajectory builder. It uses a cubic hermite spline generator for position and a SLERP approach for orientation.

## Pathing Phase
The planner is given a set of poses the robot should go through during the trajectory following. This list can contain points in any TF refrence frame, so long as it is in the same TF tree as the frame identified by the `common_frame` parameter. 
1. The poses are transformed back to the TF frame `common_frame` in which the controller should be controlling position of the vehicle.
2. The transformed positions are fed into a cubic hermite planner which plans positions and the tangent of the position at each point. The trajectory density is controlled by the `interp_density` parameter.
3. The transformed orientations are fed into a Spherical Linear intERPolation (SLERP) system to generate a constant rotation "path" between waypoints.
4. At this point, the path generation is complete, and the trajectory system is prepared for execution. The trajectory system uses a trapezoidal motion planning technique





## new idea
1. plan linear path with rough distance between points
2. run a simple trapezoidal timing on distance to get total duration (approx) of path
3. use duration to compute time scaling
4. apply time scaling to path sampling in position and orientation
5. apply time scaling to get velocity
6. apply time scaling to get accels
