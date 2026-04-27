#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarObstacleAvoidanceP:
    def __init__(self):
        rospy.init_node("lidar_obstacle_avoidance_p")

        self.safe_distance = 0.35
        self.front_angle_limit = 30.0

        self.kp = 4.0
        self.forward_speed = 0.10
        self.reverse_speed = -0.08
        self.min_turn_speed = 0.70
        self.max_turn_speed = 1.50

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        rospy.loginfo("LiDAR obstacle avoidance P controller started.")

    def scan_callback(self, msg):
        valid_front_ranges = []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            if r <= msg.range_min or r >= msg.range_max:
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            if -self.front_angle_limit <= angle_deg <= self.front_angle_limit:
                valid_front_ranges.append((r, angle_deg))

        if not valid_front_ranges:
            rospy.logwarn("No valid front LiDAR readings")
            return

        min_distance, min_angle = min(valid_front_ranges, key=lambda x: x[0])

        rospy.loginfo(
            "Front closest object: %.3f m at %.1f degrees",
            min_distance,
            min_angle
        )

        cmd = Twist()

        if min_distance <= self.safe_distance:
            rospy.logwarn("Obstacle detected. Reverse and turn.")

            error = self.safe_distance - min_distance
            turn_speed = self.kp * error

            turn_speed = max(turn_speed, self.min_turn_speed)
            turn_speed = min(turn_speed, self.max_turn_speed)

            cmd.linear.x = self.reverse_speed
            cmd.angular.z = turn_speed

        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    try:
        LidarObstacleAvoidanceP()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
