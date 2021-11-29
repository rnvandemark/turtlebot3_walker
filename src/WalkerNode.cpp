/** @file WalkerNode.hpp
 *  @author Robert Vandemark
 *  @brief A node which uses a walker controller with a turtlebot.
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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/bind.hpp>

#include "turtlebot3_walker/WalkerController.hpp"

/** Handle laser scan updates, which will publish an update for the turtlebot's
 *  commanded velocity.
 *  @param controller The walking actor controller used to control this bot
 *  @param msg The laser scan update
 *  @param cmd_vel_pub The publisher used to send an updated turtlebot velocity
 */
void scanCallback(WalkerController* controller,
                  const sensor_msgs::LaserScanConstPtr& msg,
                  ros::Publisher* cmd_vel_pub) {
    // Pass the data onto the controller and get and updated velocity
    double lin_x_vel, ang_z_vel;
    controller->handleScanInput(msg->range_min,
                                msg->range_max,
                                msg->angle_increment,
                                msg->ranges,
                                &lin_x_vel,
                                &ang_z_vel);

    // Publish this updated velocity as a twist
    geometry_msgs::Twist tw;
    tw.linear.x = lin_x_vel;
    tw.angular.z = ang_z_vel;
    cmd_vel_pub->publish(tw);
}

/** Main entry point for our ROS node.
 *  @param argc The number of program arguments.
 *  @param argv The list of program input string arguments.
 *  @return The return code of the program.
 */
int main(int argc, char** argv) {
    // Init the node and name it
    ros::init(argc, argv, "turtlebot3_walker");
    ros::NodeHandle nh;

    double ws;
    if (nh.getParam("initial_walking_speed", ws) && (0 < ws)) {
        ROS_DEBUG_STREAM("Received initial walking speed: " << ws);
    } else {
        ROS_FATAL_STREAM("ROS parameter 'initial_walking_speed' either unset"
                         " or has an improper value, must be a nonzero,"
                         " positive double.");
        return -1;
    }

    double es;
    if (nh.getParam("initial_evaluation_speed", es) && (0 < es)) {
        ROS_DEBUG_STREAM("Received initial evaluation speed: " << es);
    } else {
        ROS_FATAL_STREAM("ROS parameter 'initial_evaluation_speed' either"
                         " unset or has an improper value, must be a nonzero,"
                         " positive double.");
        return -1;
    }

    double sar;
    if (nh.getParam("initial_stop_angle_range", sar) && (0 < sar)) {
        ROS_DEBUG_STREAM("Received initial stop angle range: " << sar);
    } else {
        ROS_FATAL_STREAM("ROS parameter 'initial_stop_angle_range' either"
                         " unset or has an improper value, must be a nonzero,"
                         " positive double.");
        return -1;
    }

    double st;
    if (nh.getParam("initial_stop_threshold", st) && (0 < st)) {
        ROS_DEBUG_STREAM("Received initial stop threshold: " << st);
    } else {
        ROS_FATAL_STREAM("ROS parameter 'initial_stop_threshold' either"
                         " unset or has an improper value, must be a nonzero,"
                         " positive double.");
        return -1;
    }

    // Create our controller given configuration parameters on the param server
    WalkerController controller(ws, es, sar, st);

    // Publish the command velocity topic for the turtlebot
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(
        "/cmd_vel",
        1);

    // Subscribe to updates on the laser scan topic, where the callback is a
    // function of this update and the walker controller, and each updated
    // laser reading updates the velocity of the robot
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>(
        "/scan",
        1,
        boost::bind(scanCallback, &controller, _1, &cmd_vel_pub));

    // Spin until we're done
    ros::spin();

    return 0;
}
