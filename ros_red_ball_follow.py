#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class RedBallFollower:
    def __init__(self):
        rospy.init_node("red_ball_follower_ros", anonymous=True)

        self.bridge = CvBridge()

        # Subscribe camera image from usb_cam node
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.image_callback
        )

        # Publish velocity command to robot/arduino_node
        self.cmd_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=10
        )

        # Publish processed output image
        self.output_image_pub = rospy.Publisher(
            "/red_ball/output_image",
            Image,
            queue_size=10
        )

        # Robot movement parameters
        self.forward_speed = 0.12
        self.turn_speed = 0.35

        # Image processing parameters
        self.center_tolerance = 50
        self.min_area = 500

        rospy.loginfo("Red Ball Follower ROS node started.")
        rospy.loginfo("Subscribing image from /usb_cam/image_raw")
        rospy.loginfo("Publishing velocity to /cmd_vel")
        rospy.loginfo("Publishing output image to /red_ball/output_image")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )
        except Exception as e:
            rospy.logerr("Failed to convert ROS image to OpenCV image: %s", str(e))
            return

        height, width, _ = frame.shape
        frame_center = width // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red colour range in HSV
        # Red needs two ranges because red appears at both ends of HSV hue scale
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        red_mask = mask1 + mask2

        # Remove small noise
        red_mask = cv2.erode(red_mask, None, iterations=2)
        red_mask = cv2.dilate(red_mask, None, iterations=2)

        contours, _ = cv2.findContours(
            red_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        cmd = Twist()
        action = "STOP - NO RED BALL"

        # Draw centre reference line
        cv2.line(
            frame,
            (frame_center, 0),
            (frame_center, height),
            (255, 255, 0),
            2
        )

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(largest_contour)

                cx = int(x + w / 2)
                cy = int(y + h / 2)

                error = cx - frame_center

                # Draw detected ball area
                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 0),
                    2
                )

                cv2.circle(
                    frame,
                    (cx, cy),
                    5,
                    (255, 0, 0),
                    -1
                )

                cv2.putText(
                    frame,
                    "cx: {} error: {}".format(cx, error),
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2
                )

                # Robot movement decision
                if abs(error) <= self.center_tolerance:
                    cmd.linear.x = self.forward_speed
                    cmd.angular.z = 0.0
                    action = "MOVE FORWARD"

                elif error < 0:
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.turn_speed
                    action = "TURN LEFT"

                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = -self.turn_speed
                    action = "TURN RIGHT"

                rospy.loginfo(
                    "Red ball detected | cx: %d | error: %d | area: %.2f | action: %s",
                    cx,
                    error,
                    area,
                    action
                )

            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                action = "STOP - OBJECT TOO SMALL"

                rospy.loginfo(
                    "Red object detected but too small | area: %.2f",
                    area
                )

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            action = "STOP - RED BALL NOT DETECTED"

        # Publish velocity command
        self.cmd_pub.publish(cmd)

        # Display action text on image
        cv2.putText(
            frame,
            action,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2
        )

        # Publish processed image as ROS topic
        try:
            output_msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding="bgr8"
            )
            self.output_image_pub.publish(output_msg)

        except Exception as e:
            rospy.logerr("Failed to publish output image: %s", str(e))

        # Optional local OpenCV display
        cv2.imshow("Red Ball Follower Output", frame)
        cv2.imshow("Red Ball Mask", red_mask)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        RedBallFollower()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        cv2.destroyAllWindows()
