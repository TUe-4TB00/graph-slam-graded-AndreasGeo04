import numpy as np
from helperfunctions import add_pose_from_global, add_landmark_measurement_from_global
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate, pose_5):
    # Adding the initial estimate for the 5th pose using our helper function `add_pose_from_global` which also adds the odometry factor between X(4) and X(5).
    pose_4 = initial_estimate.atPose2(X(4))
    graph, initial_estimate = add_pose_from_global(
        graph=graph,
        initial_estimate=initial_estimate,
        prev_key=X(4),
        new_key=X(5),
        prev_pose=pose_4,
        new_pose_global=pose_5,
        odom_noise=ODOMETRY_NOISE
    )
    return graph, initial_estimate

def add_landmark_measurement(graph, result, pose_5, landmark):
    # Adding the measurement from X(5) to the chosen landmark using our helper function `add_landmark_measurement_from_global` which calculates the correct bearing and range from the global poses.``
    landmark_point = result.atPoint2(L(landmark))
    graph = add_landmark_measurement_from_global(
        graph=graph,
        pose_key=X(5),
        pose=pose_5,
        landmark_key=L(landmark),
        landmark_point=landmark_point,
        measurement_noise=MEASUREMENT_NOISE
    )
    return graph

def optimize(graph, initial_estimate):
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate)
    result = optimizer.optimize()
    return result

def minimize_marginals(graph, initial_estimate, pose_options):
    best_pose = None
    best_landmark = None
    lowest_marginal_score = float("inf")
    best_marginal_sum = None

    for pose_name, pose_5 in pose_options.items():
        for landmark in (1, 2):
            candidate_graph = gtsam.NonlinearFactorGraph(graph)
            candidate_estimate = gtsam.Values(initial_estimate)

            candidate_graph, candidate_estimate = add_pose(
                candidate_graph,
                candidate_estimate,
                pose_5
            )
            result = optimize(candidate_graph, candidate_estimate)

            candidate_graph = add_landmark_measurement(
                candidate_graph,
                result,
                pose_5,
                landmark
            )
            result = optimize(candidate_graph, candidate_estimate)

            marginals = gtsam.Marginals(candidate_graph, result)
            landmark_1_covariance = marginals.marginalCovariance(L(1))
            landmark_2_covariance = marginals.marginalCovariance(L(2))
            marginal_score = landmark_1_covariance.trace() + landmark_2_covariance.trace()
            sum_of_marginals = landmark_1_covariance.sum() + landmark_2_covariance.sum()

            if marginal_score < lowest_marginal_score:
                best_pose = pose_name
                best_landmark = landmark
                lowest_marginal_score = marginal_score
                best_marginal_sum = sum_of_marginals

    return best_pose, best_landmark, best_marginal_sum

def minimize_errors(graph, initial_estimate, pose_options):
    best_pose = None
    best_landmark = None
    lowest_error = float("inf")
    lowest_stability_error = float("inf")
    true_poses = {
        1: gtsam.Pose2(0.0, 0.0, 0.0),
        2: gtsam.Pose2(2.0, 0.0, 0.0),
        3: gtsam.Pose2(4.0, 0.0, 0.0),
    }

    for pose_name, pose_5 in pose_options.items():
        for landmark in (1, 2):
            candidate_graph = gtsam.NonlinearFactorGraph(graph)
            candidate_estimate = gtsam.Values(initial_estimate)

            candidate_graph, candidate_estimate = add_pose(
                candidate_graph,
                candidate_estimate,
                pose_5
            )
            pose_only_result = optimize(candidate_graph, candidate_estimate)

            candidate_graph = add_landmark_measurement(
                candidate_graph,
                pose_only_result,
                pose_5,
                landmark
            )
            result = optimize(candidate_graph, candidate_estimate)

            list_of_errors = []
            stability_errors = []
            for pose_index in (1, 2, 3):
                pose_error = true_poses[pose_index].localCoordinates(
                    result.atPose2(X(pose_index))
                )
                stability_error = pose_only_result.atPose2(X(pose_index)).localCoordinates(
                    result.atPose2(X(pose_index))
                )
                list_of_errors.append(np.abs(pose_error).sum())
                stability_errors.append(np.abs(stability_error).sum())

            sum_of_errors = sum(list_of_errors)
            sum_of_stability_errors = sum(stability_errors)

            if (
                sum_of_errors < lowest_error - 1e-12
                or (
                    abs(sum_of_errors - lowest_error) <= 1e-12
                    and sum_of_stability_errors < lowest_stability_error
                )
            ):
                best_pose = pose_name
                best_landmark = landmark
                lowest_error = sum_of_errors
                lowest_stability_error = sum_of_stability_errors

    return best_pose, best_landmark, lowest_error
