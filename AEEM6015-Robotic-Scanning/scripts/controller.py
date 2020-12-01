#!/usr/bin/env python
import rospy
import numpy as np
import pcl
import os
import glob
from iiwa_msgs.msg import CartesianPose
from iiwa_msgs.msg import JointPosition
from geometry_msgs.msg import PoseStamped
from camera import rs_image
from robot import iiwa_kuka
import util
from viewpoint import vp_generator, vp_graph, vp_holder
from data_process import pc_processor
from sys import maxsize

class auto_scan:
    def __init__(self, camera, robot, dist, frame, mode='ETS'):
        print("create auto scan controller...")
        self.camera = camera
        self.robot = robot
        self.dist = dist  # distance to object
        self.frame = frame # camere frame (height = 2*frame, width = 2*frame)
        self.pcp = pc_processor(camera,robot,dist,frame) # process point cloud

        self.busy = False # robot status
        self.status = 'ready' # 'exploration','scanning','completed'

        # viewpoints
        self.vp_creator = vp_generator(camera,robot,dist)
        self.viewpoints = vp_graph()
        self.vp_index = -1

        # performance
        self.scanning_path = np.int_([])
        self.exploring_path = np.int_([])

        self.time_out = 1 # add a time out for stablize the robot

        self.mode = mode # 'EAS' explore and scan, 'ETS' explore then scan

    ######################################################################
    #### initialize
    def ready_to_initial(self):
        if self.status == 'ready':
            return True
        else:
            return False

    def initialize(self):
        if self.status == 'ready' and self.busy == False:
            jp = self.robot.initial_jp()
            if jp != None:
                self.robot.move_to_jp(jp, self._initial_cb)

    def _initial_cb(self, status):
        self.busy = True
        rospy.sleep(self.time_out)
        if status == 'done':
            print("initialing...")
            vp = self.vp_creator.compute_vp(0,0)
            if vp != None and self.robot.in_workspace(vp):
                self.vp_index = self.viewpoints.add_vp(vp)
                self.status = 'exploration'
                print("exploring...")
            else:
                print("invalid initial position")

        self.busy = False

    ######################################################################
    #### scan
    def ready_to_scan(self):
        if self.status == 'scanning':
            return True
        else:
            return False
    # autonomous scanning
    def scan(self):
        if self.status == 'scanning' and self.busy == False:
            vph = self.viewpoints.get_vph(self.scanning_path[self.vp_index])
            if vph != None:
                self.robot.move_to_cp(vph.vp(), self._scan_cb)

    def _scan_cb(self, status):
        self.busy == True
        rospy.sleep(2*self.time_out)
        if status == 'done':
            vph = self.viewpoints.get_vph(self.vp_index)
            self.pcp.scan_and_save(self.vp_index)
            index = self.vp_index+1 # index of item in scanning path
            if index >= self.scanning_path.size:
                self.status = 'completed' # complete
                self.vp_index = None
                print("scanning complete")
                print("exploring path",self.exploring_path,"dist",self.viewpoints.path_distance(self.exploring_path))
                print("scanning path",self.scanning_path,"dist",self.viewpoints.path_distance(self.scanning_path))
            else:
                vph_next = self.viewpoints.get_vph(index)
                self.vp_index = index

        self.busy == False

    #####################################################################
    # explore the object and create viewpoint for scanning
    def ready_to_explore(self):
        if self.status == 'exploration':
            return True
        else:
            return False

    def explore(self):
        if self.status == 'exploration' and self.busy == False:
            vph = self.viewpoints.get_vph(self.vp_index)
            if vph != None:
                self.robot.move_to_cp(vph.vp(), self._explore_cb)

    def _explore_cb(self, status):
        self.busy == True
        rospy.sleep(self.time_out)
        if status == 'done':
            vph = self.viewpoints.get_vph(self.vp_index)
            if self.mode == 'EAS':
                self.pcp.scan_and_save(self.vp_index)
            next_vp = self._explore_neighbor(vph)
            if next_vp == None: # terminate explore
                print("generate ", self.viewpoints.vp_size(), " viewpoints")
                if self.mode == 'EAS':
                    self.status = 'completed'
                    self.vp_index = None
                    print("scanning complete")
                    print("scanning path",self.exploring_path,"dist",self.viewpoints.path_distance(self.exploring_path))
                    self.viewpoints.print_graph()
                else:
                    self.status = 'scanning'
                    self.viewpoints.print_graph()
                    print("computing shortest path for scanning")
                    self.scanning_path = self.viewpoints.shortest_path()
                    print("shortest path is", self.scanning_path)
                    if self.scanning_path.size > 0:
                        self.vp_index = 0
                        print("scanning...")
                    else:
                        self.vp_index = None
            else:
                self.vp_index = next_vp

        self.busy == False

    # explore neighbor and create viewpoints
    def _explore_neighbor(self, vph):
        self.exploring_path = np.append(self.exploring_path,vph.vp_index())
        if vph.neighbor_index('left') == None:
            self._create_left(vph)
        if vph.neighbor_index('right') == None:
            self._create_right(vph)
        if vph.neighbor_index('up') == None:
            self._create_up(vph)
        if vph.neighbor_index('down') == None:
            self._create_down(vph)
        self.viewpoints.update_neighbors()

        print("current",vph.vp_index(),
        "left", vph.neighbor_index('left'),\
        "right",vph.neighbor_index('right'),\
        "up",vph.neighbor_index('up'),\
        "down",vph.neighbor_index('down'))

        return self.viewpoints.explore_next(self.exploring_path)

    # create viewpoint
    def _create_left(self, vph):
        vpi = self.viewpoints.neighbor_search(vph,'left')
        if vpi < 0:
            vp = self.vp_creator.compute_vp(-self.frame, 0)
            if vp != None and self.robot.in_workspace(vp):
                vpi = self.viewpoints.add_vp(vp)

        self.viewpoints.add_neighbor(vph.vp_index(),vpi,'left')
        if vpi >= 0:
            self.viewpoints.add_neighbor(vpi,vph.vp_index(),'right')

    def _create_right(self, vph):
        vpi = self.viewpoints.neighbor_search(vph,'right')
        if vpi < 0:
            vp = self.vp_creator.compute_vp(self.frame, 0)
            if vp != None and self.robot.in_workspace(vp):
                vpi = self.viewpoints.add_vp(vp)

        self.viewpoints.add_neighbor(vph.vp_index(),vpi,'right')
        if vpi >= 0:
            self.viewpoints.add_neighbor(vpi,vph.vp_index(),'left')

    def _create_up(self,vph):
        vpi = self.viewpoints.neighbor_search(vph,'up')
        if vpi < 0:
            vp = self.vp_creator.compute_vp(0,-self.frame)
            if vp != None and self.robot.in_workspace(vp):
                vpi = self.viewpoints.add_vp(vp)

        self.viewpoints.add_neighbor(vph.vp_index(),vpi,'up')
        if vpi >= 0:
            self.viewpoints.add_neighbor(vpi,vph.vp_index(),'down')

    def _create_down(self,vph):
        vpi = self.viewpoints.neighbor_search(vph,'down')
        if vpi < 0:
            vp = self.vp_creator.compute_vp(0,self.frame)
            if vp != None and self.robot.in_workspace(vp):
                vpi = self.viewpoints.add_vp(vp)

        self.viewpoints.add_neighbor(vph.vp_index(),vpi,'down')
        if vpi >= 0:
            self.viewpoints.add_neighbor(vpi,vph.vp_index(),'up')
