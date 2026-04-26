#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan

TARGET_ANGLE = -30       # your real front angle
ANGLE_WINDOW = 5         # check -35 to -25 degrees
OBSTACLE_DISTANCE = 0.5  # meter

def callback(msg):
    front_ranges = []

    for i, r in enumerate(msg.ranges):
        if math.isinf(r) or math.isnan(r):
            continue

        if r <= msg.range_min or r >= msg.range_max:
            continue

        angle_rad = msg.angle_min + i * msg.angle_increment
        angle_deg = math.degrees(angle_rad)

        if TARGET_ANGLE - ANGLE_WINDOW <= angle_deg <= TARGET_ANGLE + ANGLE_WINDOW:
            front_ranges.append(r)

    if not front_ranges:
        rospy.logwarn("No valid front reading")
        return

    front_distance = min(front_ranges)

    rospy.loginfo("Front distance: %.2f m", front_distance)

    if front_distance < OBSTACLE_DISTANCE:
        rospy.logwarn("Obstacle detected in front!")
    else:
        rospy.loginfo("Front is clear.")

def main():
    rospy.init_node("front_obstacle_detector")
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == "__main__":
    main()

