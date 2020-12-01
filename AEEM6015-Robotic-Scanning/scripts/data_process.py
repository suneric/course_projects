#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from camera import rs_image
from robot import iiwa_kuka
import util
import pcl

import os
import glob
import struct
import ctypes

#######################
# data processor
class pc_processor:
    def __init__(self,camera,robot,dist,frame):
        self.camera = camera
        self.robot = robot
        self.bbox = self._bbox(dist,frame)
        self.temp_folder = "/home/yufeng/Temp/Scanning/"
        self._clean_temp_folder(self.temp_folder)

    ##################################################
    # scan and point could process
    # input point could, matrix of camera to global
    def scan_and_save(self,index):
        pc = self.camera.point_cloud()
        mat = util.cartesian_to_matrix(self.robot.cartesian_pose())
        cloud = self._cloud_process(pc,mat)
        self._save_cloud(cloud,index)

    def _in_box(self,x,y,z):
        if x < self.bbox[0]:
            return False
        elif x > self.bbox[1]:
            return False
        elif y < self.bbox[2]:
            return False
        elif y > self.bbox[3]:
            return False
        elif z < self.bbox[4]:
            return False
        elif z > self.bbox[5]:
            return False
        else:
            return True

    def _cloud_process(self,cloud,mat):
        # filter out the point outside of the frame and depth
        point_list = []
        for data in cloud:
            x,y,z,rgb=data[:4]
            if self._in_box(x,y,z):
                tp = np.dot(mat,np.array([x,y,z,1])) # transform point
                point_list.append([tp[0],tp[1],tp[2],rgb])
        pcl_cloud = pcl.PointCloud_PointXYZRGB()
        pcl_cloud.from_list(point_list)
        return pcl_cloud

    def _save_cloud(self,cloud,index):
        file = self.temp_folder+"point_"+str(index)+".ply"
        print("save point cloud to ",file)
        pcl.save(cloud,file,"ply")

    def _bbox(self,dist,frame):
        ps = self.camera.pixel_size()
        f = self.camera.camera_focal()
        x = dist*frame*ps/f[0]
        y = dist*frame*ps/f[1]
        return np.array([-x,x,-y,y,0.8*dist,1.2*dist])

    def _clean_temp_folder(self,temp_folder):
        files = glob.glob(temp_folder+"*")
        for f in files:
            print("remove ", f)
            os.remove(f)
