#!/usr/bin/env python3

import rospy
from navigation_pkg.msg import NavigationCommand
from navigation_pkg.msg import CameraMotion
from navigation_pkg.msg import ObjectData
from navigation_pkg.msg import DepthData


class NavigationNode:

    def __init__(self):

        rospy.init_node("navigation_node")

        # ---------------- PARAMETERS ----------------
        self.safety_distance = rospy.get_param("~safety_distance", 0.5)

        # ---------------- STATE STORAGE ----------------
        self.latest_camera = None
        self.latest_object = None
        self.latest_depth = None

        # ---------------- SUBSCRIBERS ----------------
        rospy.Subscriber("/camera_motion", CameraMotion, self.camera_cb)
        rospy.Subscriber("/object_data", ObjectData, self.object_cb)
        rospy.Subscriber("/depth_data", DepthData, self.depth_cb)

        # ---------------- PUBLISHER ----------------
        self.pub = rospy.Publisher(
            "/navigation_command",
            NavigationCommand,
            queue_size=10
        )

        # ---------------- MAIN LOOP ----------------
        rospy.Timer(rospy.Duration(0.2), self.decision_loop)  # 5 FPS requirement

        rospy.loginfo("Navigation Decision Node Started")

    # =================================================
    # CALLBACKS
    # =================================================

    def camera_cb(self, msg):
        self.latest_camera = msg

    def object_cb(self, msg):
        self.latest_object = msg

    def depth_cb(self, msg):
        self.latest_depth = msg

    # =================================================
    # DECISION ENGINE
    # =================================================

    def decision_loop(self, event):

        cmd = NavigationCommand()
        cmd.command = "STOP"
        cmd.duration = 0.0

        # ---------------- SAFETY CHECK ----------------
        if self.latest_depth is not None:

            if (self.latest_depth.obstacle_detected or
                self.latest_depth.average_depth < self.safety_distance):

                cmd.command = "STOP"
                cmd.duration = 0.0
                self.pub.publish(cmd)
                return

        # ---------------- CAMERA MOTION PRIORITY ----------------
        if self.latest_camera is not None:

            if self.latest_camera.valid and self.latest_camera.confidence > 0.6:

                if self.latest_camera.movement == "left":
                    cmd.command = "MOVE_LEFT"
                    cmd.duration = 1.0

                elif self.latest_camera.movement == "right":
                    cmd.command = "MOVE_RIGHT"
                    cmd.duration = 1.0

                else:
                    cmd.command = "STOP"
                    cmd.duration = 0.0

                self.pub.publish(cmd)
                return

        # ---------------- OBJECT-BASED DECISION ----------------
        if self.latest_object is not None:

            # assume image center = 320 (can be parameterized later)
            image_center = 320

            if self.latest_object.x < image_center:
                cmd.command = "MOVE_RIGHT"
                cmd.duration = 0.8
            else:
                cmd.command = "MOVE_LEFT"
                cmd.duration = 0.8

            self.pub.publish(cmd)
            return

        # ---------------- DEFAULT SAFE STATE ----------------
        cmd.command = "STOP"
        cmd.duration = 0.0
        self.pub.publish(cmd)


if __name__ == "__main__":
    NavigationNode()
    rospy.spin()
