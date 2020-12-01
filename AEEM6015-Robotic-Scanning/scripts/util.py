#!/usr/bin/env python
import numpy as np
from iiwa_msgs.msg import CartesianPose
from iiwa_msgs.msg import JointPosition
from math import *

def trig(angle):
    return cos(angle),sin(angle)

# convert eular angle to quaternion
# yaw (Z)-pitch (Y)-roll (X)
# return q1,q1,q3,q4 which is w,x,y,z
def eularangle_to_quaternion(yaw, pitch, roll):
    cy, sy = trig(0.5*yaw)
    cp, sp = trig(0.5*pitch)
    cr, sr = trig(0.5*roll)
    w = sy*sp*sr+cy*cp*cr
    x = -sy*sp*cr+cy*cp*sr
    y = sy*cp*sr+cy*sp*cr
    z = sy*cp*cr-cy*sp*sr
    return w,x,y,z

# def eularangle_to_quaternion2(yaw,pitch,roll):
#     cy, sy = trig(yaw)
#     cp, sp = trig(pitch)
#     cr, sr = trig(roll)
#     w = 0.5*sqrt(1+cy*cp+cy*cr-sy*sp*sr+cp*cr)
#     x = (cp*sr+cy*sr+sy*sp*cr)/(4*w)
#     y = (sy*cp+sy*cr+cy*sp*sr)/(4*w)
#     z = (-sy*sr+cy*sp*cr+sp)/(4*w)
#     return w,x,y,z

def quaternion_to_eularangle(w,x,y,z):
    sr_cp = 2*(w*x+y*z)
    cr_cp = 1-2*(x*x+y*y)
    roll = atan2(sr_cp, cr_cp)
    sp = 2*(w*y-z*x)
    pitch = asin(sp)
    if abs(sp) >= 1:
        pitch = copysign(0.5*np.pi, sp)
    sy_cp = 2*(w*z+x*y)
    cy_cp = 1-2*(y*y+z*z)
    yaw = atan2(sy_cp, cy_cp)
    return yaw,pitch,roll

def transform(rotation, translation):
    Cx,Sx = trig(rotation[0])
    Cy,Sy = trig(rotation[1])
    Cz,Sz = trig(rotation[2])
    dX = translation[0]
    dY = translation[1]
    dZ = translation[2]
    mat_trans = np.array([[1,0,0,dX],
                          [0,1,0,dY],
                          [0,0,1,dZ],
                          [0,0,0,1]])
    mat_rotX = np.array([[1,0,0,0],
                         [0,Cx,-Sx,0],
                         [0,Sx,Cx,0],
                         [0,0,0,1]])
    mat_rotY = np.array([[Cy,0,Sy,0],
                         [0,1,0,0],
                         [-Sy,0,Cy,0],
                         [0,0,0,1]])
    mat_rotZ = np.array([[Cz,-Sz,0,0],
                         [Sz,Cz,0,0],
                         [0,0,1,0],
                         [0,0,0,1]])
    return np.dot(mat_rotZ,np.dot(mat_rotY,np.dot(mat_rotX,mat_trans)))

def cartesian_to_matrix(cp):
    position = cp.poseStamped.pose.position
    orientation = cp.poseStamped.pose.orientation
    matrix = np.eye(4)
    # translation
    matrix[0,3] = position.x# in meter
    matrix[1,3] = position.y
    matrix[2,3] = position.z
    # quaternion to matrix
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    Nq = w*w + x*x + y*y + z*z
    if Nq < 0.001:
        return matrix

    s = 2.0/Nq
    X = x*s
    Y = y*s
    Z = z*s
    wX = w*X; wY = w*Y; wZ = w*Z
    xX = x*X; xY = x*Y; xZ = x*Z
    yY = y*Y; yZ = y*Z; zZ = z*Z
    matrix=np.array([[1.0-(yY+zZ), xY-wZ, xZ+wY, position.x],
            [xY+wZ, 1.0-(xX+zZ), yZ-wX, position.y],
            [xZ-wY, yZ+wX, 1.0-(xX+yY), position.z],
            [0, 0, 0, 1]])
    return matrix

# homougenous matrix to quaternion
def matrix_to_cartesian(mat):
    rot = np.array([mat[0,0:3],mat[1,0:3],mat[2,0:3]])
    trans = np.array([mat[0,3],mat[1,3],mat[2,3]])
    x = trans[0]
    y = trans[1]
    z = trans[2]
    qw = 0.5*sqrt(1+rot[0,0]+rot[1,1]+rot[2,2])
    qx = (rot[2,1]-rot[1,2])/(4*qw)
    qy = (rot[0,2]-rot[2,0])/(4*qw)
    qz = (rot[1,0]-rot[0,1])/(4*qw)
    return x,y,z,qx,qy,qz,qw
