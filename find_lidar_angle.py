#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan

def callback(msg):
    rospy.loginfo("Scan received: %d range values", len(msg.ranges))

    valid_ranges = []

    for i, r in enumerate(msg.ranges):
        # reject invalid readings
        if math.isinf(r) or math.isnan(r):
            continue

        # reject 0.00 and values outside LiDAR range
        if r <= msg.range_min or r >= msg.range_max:
            continue

        angle_rad = msg.angle_min + i * msg.angle_increment
        angle_deg = math.degrees(angle_rad)

        valid_ranges.append((r, angle_deg))

    if not valid_ranges:
        rospy.logwarn("Scan received, but no valid LiDAR readings")
        return

    min_distance, min_angle = min(valid_ranges, key=lambda x: x[0])

    rospy.loginfo("Closest valid object: %.2f m at %.1f degrees", min_distance, min_angle)

def main():
    rospy.init_node("front_obstacle_detector")
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == "__main__":
    main()

