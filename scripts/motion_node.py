#!/usr/bin/env python3

import rospy
import math
from visual_nav_system.msg import RoiFeatures, MotionData

pub = None

prev_x = None
prev_y = None

motion_threshold = rospy.get_param("~motion_threshold", 1.0) # smoothing factor


def callback(msg):
    global prev_x, prev_y, pub

    cx = msg.centroid_x
    cy = msg.centroid_y

    if prev_x is None:
        prev_x = cx
        prev_y = cy
        return

    # ---- SMOOTHING (VERY IMPORTANT FIX) ----
    dx_raw = cx - prev_x
    dy_raw = cy - prev_y

    dx = motion_threshold * dx_raw
    dy = motion_threshold * dy_raw

    magnitude = math.sqrt(dx**2 + dy**2)

    # direction logic
    if abs(dx) > abs(dy):
        direction = "RIGHT" if dx > 0 else "LEFT"
    else:
        direction = "DOWN" if dy > 0 else "UP"

    reliable = magnitude > 1.0

    msg_out = MotionData()
    msg_out.dx = dx
    msg_out.dy = dy
    msg_out.magnitude = magnitude
    msg_out.direction = direction
    msg_out.reliable = reliable

    pub.publish(msg_out)

    rospy.loginfo(f"[MOTION] dx={dx:.2f}, dy={dy:.2f}, dir={direction}, rel={reliable}")

    prev_x = cx
    prev_y = cy


def main():
    global pub

    rospy.init_node("motion_node")

    pub = rospy.Publisher("/motion_data", MotionData, queue_size=10)
    rospy.Subscriber("/roi_features", RoiFeatures, callback)

    rospy.loginfo("Motion Node Started")

    rospy.spin()


if __name__ == "__main__":
    main()