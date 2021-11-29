/** @file WalkerController.cpp
 *  @author Robert Vandemark
 *  @brief A simple walker algorithm controller.
 *  @version 0.0.0
 *  @date 2021-11-28
 *
 *  @copyright Copyright 2021 Robert Vandemark
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted.
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 *  SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR
 *  IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <cmath>
#include <algorithm>

#include "turtlebot3_walker/WalkerController.hpp"

WalkerController::WalkerController(const double walking_speed,
                                   const double evaluation_speed,
                                   const double stop_angle_range,
                                   const double stop_threshold) :
        evaluating(true) {
    setWalkingSpeed(walking_speed);
    setEvaluationSpeed(evaluation_speed);
    setStopAngleRange(stop_angle_range);
    setStopThreshold(stop_threshold);
}

void WalkerController::setWalkingSpeed(const double new_walking_speed) {
    walking_speed = new_walking_speed;
}

void WalkerController::setEvaluationSpeed(const double new_evaluation_speed) {
    evaluation_speed = new_evaluation_speed;
}

void WalkerController::setStopAngleRange(const double new_stop_angle_range) {
    stop_angle_range = new_stop_angle_range;
}

void WalkerController::setStopThreshold(const double new_stop_threshold) {
    stop_threshold = new_stop_threshold;
}

void WalkerController::handleScanInput(const float dist_min,
                                       const float dist_max,
                                       const float angle_increment,
                                       const std::vector<float>& dists,
                                       double* lin_x_vel,
                                       double* ang_z_vel) {
    // Get only the readings in the angle range on either side
    const int num_samples = std::floor(stop_angle_range / angle_increment);
    std::vector<float> trimmed(dists.begin(), dists.begin() + num_samples);
    trimmed.insert(trimmed.end(), dists.end() - num_samples, dists.end());

    // Sort the distances and drop any outside of the allowable range
    std::sort(trimmed.begin(), trimmed.end());
    trimmed = std::vector<float>(
        std::lower_bound(trimmed.begin(),
                         trimmed.end(),
                         dist_min),
        std::upper_bound(trimmed.begin(),
                         trimmed.end(),
                         dist_max));

    // Confirm whether or not there are any obstacles that are already too close
    const bool sees_obstacle = (trimmed.end() != std::find_if(
        trimmed.begin(),
        trimmed.end(),
        [&](float f) { return f < stop_threshold; }));

    // If we are evaluating a new direction to move, modify ang_z_vel depending
    // on the value of sees_obstacle; if we are walking, do so for lin_x_vel
    if (sees_obstacle) {
        // If evaluating, continue to spin
        // If walking, stop moving forward and re-evaluate
        *lin_x_vel = 0.0;
        *ang_z_vel = evaluation_speed;
    } else {
        // If evaluating, found a safe direction to move!
        // If moving, continue to move forward
        *lin_x_vel = walking_speed;
        *ang_z_vel = 0.0;
    }

    // We are evaluating whenever we see an obstacle
    evaluating = sees_obstacle;
}
