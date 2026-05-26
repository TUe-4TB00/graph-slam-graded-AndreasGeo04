import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    pose_4 = result.atPose2(X(4))
    landmark_2 = result.atPoint2(L(2))

    bearing = pose_4.bearing(landmark_2)
    distance = pose_4.range(landmark_2)

    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), bearing, distance, MEASUREMENT_NOISE))
    return graph
