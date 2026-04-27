#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidanceP:
    def __init__(self):
        rospy.init_node("obstacle_avoidance_p")

        # Desired safe distance from obstacle (m)
        self.safe_distance = 0.35

        # Front detection sector: -30° to +30°
        self.front_angle_limit = 30.0

        # Proportional gain
        self.kp = 25.0

        # Forward speed when path is clear
        self.forward_speed = 0.12

        # ROS communication
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        rospy.loginfo("P Controller Obstacle Avoidance Started")

    def scan_callback(self, msg):
        front_points = []

        # Read LiDAR data
        for i, r in enumerate(msg.ranges):

            # Ignore invalid values
            if math.isinf(r) or math.isnan(r):
                continue

            if r <= msg.range_min or r >= msg.range_max:
                continue

            # Convert index to angle
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            # Keep only front sector
            if -self.front_angle_limit <= angle_deg <= self.front_angle_limit:
                front_points.append((r, angle_deg))

        if not front_points:
            rospy.logwarn("No valid front obstacle data")
            return

        # Closest obstacle in front
        min_distance, min_angle = min(front_points, key=lambda x: x[0])

        cmd = Twist()

        # Path clear
        if min_distance > self.safe_distance:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

            rospy.loginfo(
                "Clear path | Distance: %.3f m",
                min_distance
            )

        # Obstacle detected
        else:
            # Recalculate error every scan
            error = self.safe_distance - min_distance

            # Pure P controller
            turn_speed = self.kp * error

            cmd.linear.x = 0.0

            # Turn away from obstacle
            if min_angle >= 0:
                cmd.angular.z = -turn_speed   # turn right
            else:
                cmd.angular.z = turn_speed    # turn left

            rospy.loginfo(
                "Obstacle: %.3f m | Error: %.3f | Turn: %.3f",
                min_distance,
                error,
                cmd.angular.z
            )

        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    try:
        ObstacleAvoidanceP()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
