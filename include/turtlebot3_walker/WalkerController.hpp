/** @file WalkerController.hpp
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

#pragma once

#include <vector>

/** A simple walker algorithm controller.
 */
class WalkerController {
 private:
    /** Whether or not the walking actor is (re)evaluating the direction in
     *  which it should travel.
     */
    bool evaluating;

    /** The speed at which the walking actor will move directly forward. Units
     *  are in meters/second.
     */
    double walking_speed;

    /** The speed at which the walking actor will turn when (re)evaluating a
     *  direction in which to move. Units are in radians/second.
     */
    double evaluation_speed;

    /** The angle "in front" of the walking actor to consider when scanning for
     *  obstacles, to the left and the right of the scanner. More precisely,
     *  obstacles will be considered if they are within the semi-circular
     *  region formed by rotating about the walking actor's positive z-axis off
     *  of its positive x-axis, and rotating the negative of this amount, with
     *  a distance of @a stop_threshold. Units are in radians.
     */
    double stop_angle_range;

    /** The distance between an obstacle and the walking actor that will make
     *  the actor stop and find a new direction. Units are in meters.
     */
    double stop_threshold;

 public:
    /** Constructor with initial values for all members.
     *  @param stop_angle_range The initial value for @a stop_angle_range.
     *  @param stop_threshold The initial value for @a stop_threshold.
     */
    WalkerController(const double walking_speed,
                     const double evaluation_speed,
                     const double stop_angle_range,
                     const double stop_threshold);

    /** Setter for the speed at which the walking actor will move directly
     *  forward.
     *  @param new_walking_speed The new walking speed, in the same units as
     *  @a walking_speed.
     */
    void setWalkingSpeed(const double new_walking_speed);

    /** Setter for the speed at which the walking actor will turn when
     *  (re)evaluating a direction in which to move.
     *  @param new_evaluation_speed The new evaluation speed, in the same units
     *  as @a evaluation_speed.
     */
    void setEvaluationSpeed(const double new_evaluation_speed);

    /** Setter for the stopping angle range to check.
     *  @param new_stop_angle_range The new stopping angle range to check, in
     *  the same units as @a stop_angle_range.
     */
    void setStopAngleRange(const double new_stop_angle_range);

    /** Setter for the stopping threshold.
     *  @param new_stop_threshold The new threshold to stop at, in the same
     *  units as @a stop_threshold.
     */
    void setStopThreshold(const double new_stop_threshold);

    /** Handle updated scanner input. If a new path is being evaluated, 
     *  @param dist_min The minimum allowable distance (any value in @a dists
     *  lower than this should be discarded), in meters.
     *  @param dist_max The maximum allowable distance (any value in @a dists
     *  higher than this should be discarded), in meters.
     *  @param angle_increment The angle increment that each sample is
     *  separated by, in radians.
     *  @param dists The list of depth samples surrounding the walking actor,
     *  each in meters. The first sample starts at an angle of 0.0, the one
     *  after is at @a angle_increment, all the way up to 2*pi radians.
     *  @param lin_x_vel Return value, the linear velocity in the +x direction
     *  that the robot should have as a result of this function call.
     *  @param ang_z_vel Return value, the angular velocity about the z-axis
     *  that the robot should have as a result of this function call.
     */
    void handleScanInput(const float dist_min,
                         const float dist_max,
                         const float angle_increment,
                         const std::vector<float>& dists,
                         double* lin_x_vel,
                         double* ang_z_vel);
};
