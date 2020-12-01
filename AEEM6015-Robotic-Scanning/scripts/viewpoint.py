#!/usr/bin/env python
import rospy
import numpy as np
from math import *#sin, cos, acos, asin, radians
from geometry_msgs.msg import PoseStamped
import util
from path_opt import TSPOpt


####################################################
#### viewpoint graph
####################################################
class vp_graph:
    def __init__(self):
        self.vp_list = np.array([])

    def vp_size(self):
        return self.vp_list.size

    # get viewpoint holder
    def get_vph(self, index):
        if index >= 0 and index < self.vp_list.size:
            return self.vp_list[index]
        else:
            return None

    def update_neighbors(self):
        for vph in self.vp_list:
            if vph.neighbor_index('left') == None:
                vpi = self.neighbor_search(vph,'left')
                if vpi >= 0:
                    self.add_neighbor(vph.vp_index(), vpi, 'left')
                    self.add_neighbor(vpi,vph.vp_index(),'right')
            if vph.neighbor_index('right') == None:
                vpi = self.neighbor_search(vph,'right')
                if vpi >= 0:
                    self.add_neighbor(vph.vp_index(), vpi, 'right')
                    self.add_neighbor(vpi,vph.vp_index(),'left')
            if vph.neighbor_index('up') == None:
                vpi = self.neighbor_search(vph,'up')
                if vpi >= 0:
                    self.add_neighbor(vph.vp_index(), vpi, 'up')
                    self.add_neighbor(vpi,vph.vp_index(),'down')
            if vph.neighbor_index('down') == None:
                vpi = self.neighbor_search(vph,'down')
                if vpi >= 0:
                    self.add_neighbor(vph.vp_index(), vpi, 'down')
                    self.add_neighbor(vpi,vph.vp_index(),'up')

    def neighbor_search(self, vph, type):
        vpi = -1
        if type == 'left':
            vph1 = self.get_vph_n3(vph,'up','left','down')
            vph2 = self.get_vph_n3(vph,'down','left','up')
            if vph1 != None:
                vpi = vph1.vp_index()
                #print(vph.vp_index(),"up-left-down",vpi)
            elif vph2 != None:
                vpi = vph2.vp_index()
                #print(vph.vp_index(),"down-left-up",vpi)
            else:
                vph1 = self.get_vph_n2(vph,'up','left')
                vph12 = self.get_vph_n3(vph1,'left','down','right')
                vph2 = self.get_vph_n2(vph,'down','left')
                vph22 = self.get_vph_n3(vph2,'left','up','right')
                if vph12 != None:
                    vpi = vph12.vp_index()
                    #print(vph.vp_index(),"up-left,left-down-right",vpi)
                elif vph22 != None:
                    vpi = vph22.vp_index()
                    #print(vph.vp_index(),"down-left,left-up-right",vpi)
        elif type == 'right':
            vph1 = self.get_vph_n3(vph,'up','right','down')
            vph2 = self.get_vph_n3(vph,'down','right','up')
            if vph1 != None:
                vpi = vph1.vp_index()
                #print(vph.vp_index(),"up-right-down",vpi)
            elif vph2 != None:
                vpi = vph2.vp_index()
                #print(vph.vp_index(),"down-right-up",vpi)
            else:
                vph1 = self.get_vph_n2(vph,'up','right')
                vph12 = self.get_vph_n3(vph1,'right','down','left')
                vph2 = self.get_vph_n2(vph,'down','right')
                vph22 = self.get_vph_n3(vph2,'right','up','left')
                if vph12 != None:
                    vpi = vph12.vp_index()
                    #print(vph.vp_index(),"up-right, right-down-left",vpi)
                elif vph22 != None:
                    vpi = vph22.vp_index()
                    #print(vph.vp_index(),"down-right, right-up-left",vpi)
        elif type == 'up':
            vph1 = self.get_vph_n3(vph,'left','up','right')
            vph2 = self.get_vph_n3(vph,'right','up','left')
            if vph1 != None:
                vpi = vph1.vp_index()
                #print(vph.vp_index(),"left-up-right",vpi)
            elif vph2 != None:
                vpi = vph2.vp_index()
                #print(vph.vp_index(),"right-up-left",vpi)
            else:
                vph1 = self.get_vph_n2(vph,'left','up')
                vph12 = self.get_vph_n3(vph1,'up','right','down')
                vph2 = self.get_vph_n2(vph,'right','up')
                vph22 = self.get_vph_n3(vph2,'up','left','down')
                if vph12 != None:
                    vpi = vph12.vp_index()
                    #print(vph.vp_index(),"left-up, up-right-down",vpi)
                elif vph22 != None:
                    vpi = vph22.vp_index()
                    #print(vph.vp_index(),"right-up, up-left-down",vpi)
        elif type == 'down':
            vph1 = self.get_vph_n3(vph,'left','down','right')
            vph2 = self.get_vph_n3(vph,'right','down','left')
            if vph1 != None:
                vpi = vph1.vp_index()
                #print(vph.vp_index(),"left-down-right",vpi)
            elif vph2 != None:
                vpi = vph2.vp_index()
                #print(vph.vp_index(),"right-down-left",vpi)
            else:
                vph1 = self.get_vph_n2(vph,'left','down')
                vph12 = self.get_vph_n3(vph1,'down','right','up')
                vph2 = self.get_vph_n2(vph,'right','down')
                vph22 = self.get_vph_n3(vph2,'down','left','up')
                if vph12 != None:
                    vpi = vph12.vp_index()
                    #print(vph.vp_index(),"left-down, down-right-up",vpi)
                elif vph22 != None:
                    vpi = vph22.vp_index()
                    #print(vph.vp_index(),"right-down, down-left-up",vpi)
        return vpi

    # get neighbor vph in 3 step
    def get_vph_n3(self,vph,type1,type2,type3):
        if vph == None:
            return None
        vph1 = self.get_vph(vph.neighbor_index(type1))
        if vph1 == None:
            return None
        vph2 = self.get_vph(vph1.neighbor_index(type2))
        if vph2 == None:
            return None
        vph3 = self.get_vph(vph2.neighbor_index(type3))
        return vph3
    # get neighbor vph in 2 step
    def get_vph_n2(self,vph,type1,type2):
        vph1 = self.get_vph(vph.neighbor_index(type1))
        if vph1 == None:
            return None
        vph2 = self.get_vph(vph1.neighbor_index(type2))
        return vph2

    def add_vp(self, vp):
        if (vp == None):
            return -1
        else:
            index = self.vp_list.size
            vph = vp_holder(index, vp)
            self.vp_list = np.append(self.vp_list, vph)
            return index
            # unlike the list append, numpy append does not append in-place,
            # it returns a new array with extra elements appeded

    # vpi: index of current viewpoint
    # neighbor: index of neighbor viewpoints
    def add_neighbor(self, vpi, neighbor, type):
        if vpi == None:
            print("invalid viewpoint")
            return
        nbi = self.get_neighbor(vpi,type)
        # update neighbor index, -1 for invalid
        if nbi == None:
            self.vp_list[vpi].add_neighbor(neighbor,type)

    def get_neighbor(self, vpi, type):
        return self.vp_list[vpi].neighbor_index(type)

    def path_distance(self,vp_path):
        count = vp_path.size
        total = 0
        for i in range(count-1):
            vph1 = self.vp_list[int(i)]
            vph2 = self.vp_list[int(i+1)]
            dist = self.vp_distance(vph1,vph2)
            total = total + dist
        return total

    def vp_distance(self,vph1,vph2):
        if vph1 == None or vph2 == None:
            return 0.0
        vp1 = vph1.vp()
        vp2 = vph2.vp()
        x1,y1,z1 = vp1.pose.position.x,vp1.pose.position.y,vp1.pose.position.z
        x2,y2,z2 = vp2.pose.position.x,vp2.pose.position.y,vp2.pose.position.z
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))

    # find next viewpoint by explored vp indices
    # return index of the next vp
    def explore_next(self,vp_explored):
        if vp_explored is None or vp_explored.size == 0:
            return None
        vph_s = self.vp_list[vp_explored[vp_explored.size-1]] # start vp
        vp_unexplored = np.array([vph_s])
        vpi_unexplored = np.array([],dtype=int)
        for vph in self.vp_list:
            if vph.full_surrounded() == False or self.explored_vp(vph, vp_explored) == False:
                vp_unexplored = np.append(vp_unexplored, vph)
                vpi_unexplored = np.append(vpi_unexplored,vph.vp_index())

        if vp_unexplored.size <=1:
            return None
        else:
            vpi = self.shortest_path(vp_unexplored)[1]
            next_vp = vp_unexplored[vpi].vp_index()
            print("waiting to explore", vpi_unexplored, "next", next_vp)
            return next_vp

    def explored_vp(self,vph,vp_explored):
        for vpi in vp_explored:
            if vph.vp_index() == vpi:
                return True
        else:
            return False

    # compute a shortest path for scanning
    def shortest_path(self, vp_list=None):
        if vp_list is None:
            vp_list = self.vp_list
        # weight graph
        num_vp = vp_list.size
        graph = np.zeros((num_vp,num_vp))
        for i in range(num_vp):
            vph_i = vp_list[i]
            for j in range(i,num_vp):
                vph_j = vp_list[j]
                if i == j:
                    graph[i][j]=0.0
                else:
                    dist = self.vp_distance(vph_i, vph_j)
                    graph[i][j] = dist
                    graph[j][i] = dist
        #print("distance graph",graph)
        # travelling sales man problem solver
        tspOpt = TSPOpt(graph,num_vp)
        dist, path = tspOpt.optimize()
        #print("shortest path",dist,path)
        return np.array(path)

    def print_graph(self):
        vph_tl = None # top left viewpoint
        for vph in self.vp_list:
            if vph.neighbor_index('left') < 0 and vph.neighbor_index('up') < 0:
                vph_tl = vph
                break
        row_n = 1
        vph_down = self.get_vph(vph_tl.neighbor_index('down'))
        while vph_down != None:
            row_n = row_n+1
            vph_down = self.get_vph(vph_down.neighbor_index('down'))
        col_n = 1
        vph_right = self.get_vph(vph_tl.neighbor_index('right'))
        while vph_right != None:
            col_n = col_n+1
            vph_right = self.get_vph(vph_right.neighbor_index('right'))

        #print("column, row",col_n, row_n)
        # build graph
        graph = np.zeros((row_n,col_n),dtype=int)
        vph_row = vph_tl
        i = 0
        while vph_row != None:
            vph_col = vph_row
            j = 0
            while vph_col != None:
                #print(i,j)
                graph[i][j] = vph_col.vp_index()
                vph_col = self.get_vph(vph_col.neighbor_index('right'))
                j = j+1
            vph_row = self.get_vph(vph_row.neighbor_index('down'))
            i = i+1
        print("complete graph",graph)


####################################################
#### viewpoint holder
####################################################
class vp_holder:
    def __init__(self, index, vp):
        self.index = index
        self.viewpoint = vp
        self.left = None
        self.right = None
        self.up = None
        self.down = None

    def vp_index(self):
        return self.index

    def vp(self):
        return self.viewpoint

    def full_surrounded(self):
        if self.left == None or self.right == None or self.up == None or self.down == None:
            return False
        else:
            #print(self.index,"is full surrounded")
            return True

    # neighbor for 0:left, 1:right, 2:up, and 3:down
    def add_neighbor(self, neighbor, type):
        if type == 'left':
            self.left = neighbor
        elif type == 'right':
            self.right = neighbor
        elif type == 'up':
            self.up = neighbor
        elif type == 'down':
            self.down = neighbor
        else:
            print("try to add invalid neighbor")

    def neighbor_index(self, type):
        if type == 'left':
            return self.left
        elif type == 'right':
            return self.right
        elif type == 'up':
            return self.up
        elif type == 'down':
            return self.down
        else:
            print("try to query invalid neighbor")

########################################################
### viewpoint generator
########################################################
class vp_generator:
    # initial vp_generator with camera information
    def __init__(self,camera,robot,dist):
        self.camera = camera
        self.robot = robot
        self.dist = dist

    # compute a viewpoint
    def compute_vp(self,dx,dy):
        center = self.camera.image_center()
        u = center[0]+dx
        v = center[1]+dy
        cp = self.robot.cartesian_pose()
        mat = util.cartesian_to_matrix(cp)
        #print("current cartisan pose ", cp)
        vp = self.viewpoint(u,v,self.dist,mat)
        return vp

    # create a viewpoint at pixel (u,v) and distance to surface is d
    # with tranform of end-effector and camere to end-effector(4*4 homougenous transform)
    def viewpoint(self,u,v,d,mat_c):
        # target point in camera frame at pixel u, v
        # print("current", mat)
        pc = self.position3d(u,v)
        if pc[2] < 0: # depth
            print("invalid depth image")
            return None

        # target point normal in camera frame
        nc = self.normal3d(u,v)
        pv = pc-d*nc
        # rotate angles in robot frame
        a = asin(-nc[1]) # about x
        b = asin(nc[0]/cos(a)) # about y
        c = 0 # about z
        mat_v = util.transform(np.array([a,b,c]), pv)
        #print("in camera, normal, position ", nc, pv, [a,b,c], mat_v)

        # mat_c : camera pose in robot frame
        # mat_v : next viewpoint in camera system
        # mat_e: next camera pose in robot frame
        mat_e = np.dot(mat_c,mat_v)
        vp = self.transform_to_cartesian(mat_e)
        return vp

    # evaluate pointion in 3d with the pixel u and v, in meter
    def position3d(self,u,v):
        f = self.camera.camera_focal()
        pp = self.camera.camera_principal()
        ps = self.camera.pixel_size()
        dist = self.distance(u,v)
        #print("u,v",u,v,dist)
        x = dist*(u-pp[0])*ps/f[0]
        y = dist*(v-pp[1])*ps/f[1]
        z = dist
        return 0.001*np.array([x,y,z])

    # calculate normal at pixel u,v
    def normal3d(self,u,v):
        # TODO: validate u and v (u for width, v for height)
        # z[v,u] for depth
        f = self.camera.camera_focal()
        pp = self.camera.camera_principal()
        ps = self.camera.pixel_size()
        dzdu = (self.distance(u+10,v)-self.distance(u-10,v))/20.0 # approximation with 20 pixels
        dzdv = (self.distance(u,v+10)-self.distance(u,v-10))/20.0 # approximation with 20 pixels
        # solution 1
        dxdu = self.distance(u,v)/f[0] + (u-pp[0])*ps*dzdu/f[0]
        dydu = (v-pp[1])*ps*dzdu/f[1]
        dxdv = (u-pp[0])*ps*dzdv/f[0]
        dydv = self.distance(u,v)/f[1] + (v-pp[1])*ps*dzdv/f[1]
        vec_u = [dxdu,dydu,dzdu]
        vec_v = [dxdv,dydv,dzdv]
        n = np.cross(vec_u,vec_v) # v cross u for the normal pointing to the camera
        unit_n = n/np.linalg.norm(n)
        return unit_n

    # calculate normal at pixel u,v
    # def normal3d_1(self,u,v):
    #     dzdu = (self.distance(u+10,v)-self.distance(u-10,v))/20.0 # approximation with 10 pixels
    #     dzdv = (self.distance(u,v+10)-self.distance(u,v-10))/20.0 # approximation with 10 pixels
    #     n = np.array([-dzdu, -dzdv, 1])
    #     unit_n = n/np.linalg.norm(n)
    #     return unit_n

    # calculate mean distance in a small pixel frame around u,v
    # a non-zero mean value for the pixel with its neighboring pixels
    # in mm
    def distance(self,u,v,size=3):
        dist_list=[]
        for i in range(-size,size):
            for j in range(-size,size):
                value = self.camera.depth_image()[v+j,u+i]
                if value != 0:
                    dist_list.append(value)
        #print(dist_list)
        if not dist_list:
            return -1
        else:
            return np.mean(dist_list)

    def transform_to_cartesian(self, mat):
        x,y,z,px,py,pz,pw = util.matrix_to_cartesian(mat)
        cp = PoseStamped()
        cp.header.frame_id="iiwa_link_0"
        cp.pose.position.x = x
        cp.pose.position.y = y
        cp.pose.position.z = z
        cp.pose.orientation.w = pw
        cp.pose.orientation.x = px
        cp.pose.orientation.y = py
        cp.pose.orientation.z = pz
        return cp

    def eular_to_cartesian(self,p,a,b,c):
        print(p,a,b,c)
        cp = PoseStamped()
        cp.header.frame_id="iiwa_link_0"
        cp.pose.position.x = p[0]
        cp.pose.position.y = p[1]
        cp.pose.position.z = p[2]
        w,x,y,z = util.eularangle_to_quaternion(c,b,a)
        cp.pose.orientation.w = w
        cp.pose.orientation.x = x
        cp.pose.orientation.y = y
        cp.pose.orientation.z = z
        return cp
