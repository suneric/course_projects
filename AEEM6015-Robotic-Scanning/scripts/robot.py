#!/usr/bin/env python
import rospy
import numpy as np
from iiwa_msgs.msg import CartesianPose
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import MoveToCartesianPoseActionResult
from iiwa_msgs.msg import MoveToJointPositionActionResult
from geometry_msgs.msg import PoseStamped
import util

class iiwa_kuka:
    # create a image view with a frame size for the ROI
    def __init__(self):
        print("create robot instance...")
        self.cp_sub = rospy.Subscriber("/iiwa/state/CartesianPose", CartesianPose, self._cp_callback)
        self.jp_sub = rospy.Subscriber("/iiwa/state/JointPosition", JointPosition, self._jp_callback)
        self.cp = None
        self.jp = None
        self.cp_pub = rospy.Publisher('/iiwa/command/CartesianPose',PoseStamped, queue_size=1)
        self.jp_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=1)

    def ready(self):
        if self.jp == None or self.cp == None:
            return False
        else:
            return True

    # check if cartisian pose is in workspace
    def in_workspace(self, cp):
        x = cp.pose.position.x
        y = cp.pose.position.y
        z = cp.pose.position.z
        if x < 0.5 or x > 0.85:
            print("x out of workspace")
            return False
        if y < -0.25 or y > 0.25:
            print("y out of workspace")
            return False
        if z < 0.1 or z > 1.2:
            print("z out of workspace")
            return False
        return True

    def cartesian_pose(self):
        return self.cp

    def joint_position(self):
        return self.jp

    def initial_jp(self):
        jp = JointPosition()
        jp.position.a1 = 0.0
        jp.position.a2 = 0.0
        jp.position.a3 = 0.0
        jp.position.a4 = -0.4*np.pi
        jp.position.a5 = 0.0
        jp.position.a6 = 0.5*np.pi
        jp.position.a7 = 0.0
        return jp

    def move_to_cp(self, cp_cmd, callback):
        #print("cp", self.cartesian_pose(),"command cp", cp_cmd)
        if self._same_cartesian_pos(cp_cmd,self.cartesian_pose())==False:
            self.cp_pub.publish(cp_cmd)
            callback('moving')
        else:
            callback('done')

    def move_to_jp(self, jp_cmd, callback):
        if self._same_joint_position(jp_cmd,self.joint_position())==False:
            self.jp_pub.publish(jp_cmd)
            callback('moving')
        else:
            callback('done')

    def _cp_callback(self,data):
        #print("_cp_callback")
        self.cp = data

    def _jp_callback(self,data):
        #print("_jp_callback")
        self.jp = data

    def _same_joint_position(self, jp0, jp1):
        tolerance = 0.001
        if abs(jp0.position.a1-jp1.position.a1) > tolerance:
            return False
        elif abs(jp0.position.a2-jp1.position.a2) > tolerance:
            return False
        elif abs(jp0.position.a3-jp1.position.a3) > tolerance:
            return False
        elif abs(jp0.position.a4-jp1.position.a4) > tolerance:
            return False
        elif abs(jp0.position.a5-jp1.position.a5) > tolerance:
           return False
        elif abs(jp0.position.a6-jp1.position.a6) > tolerance:
           return False
        elif abs(jp0.position.a7-jp1.position.a7) > tolerance:
           return False
        else:
           print("reached.")
           return True

    def _same_cartesian_pos(self, cp_cmd, cp_rbt):
        tolerance = 0.001
        if abs(cp_cmd.pose.position.x-cp_rbt.poseStamped.pose.position.x) > tolerance:
            return False
        elif abs(cp_cmd.pose.position.y-cp_rbt.poseStamped.pose.position.y) > tolerance:
            return False
        elif abs(cp_cmd.pose.position.z-cp_rbt.poseStamped.pose.position.z) > tolerance:
            return False
        elif abs(cp_cmd.pose.orientation.x-cp_rbt.poseStamped.pose.orientation.x) > tolerance:
            return False
        elif abs(cp_cmd.pose.orientation.y-cp_rbt.poseStamped.pose.orientation.y) > tolerance:
            return False
        elif abs(cp_cmd.pose.orientation.z-cp_rbt.poseStamped.pose.orientation.z) > tolerance:
            return False
        elif abs(cp_cmd.pose.orientation.w-cp_rbt.poseStamped.pose.orientation.w) > tolerance:
            return False
        else:
            print("reached.")
            return True
