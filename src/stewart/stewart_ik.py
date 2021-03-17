#! /usr/bin/python
import rospy, rospkg, math
import numpy as np
import IPython
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class stewart_ik():

    def __init__(self):
        self.MUL = 1
        self.height = 2.0 *self.MUL
        self.b =      [[-0.101,    0.8, 0.25, 1],
                      [0.101,    0.8, 0.25, 1],
                      [0.743, -0.313, 0.25, 1],
                      [0.642, -0.487, 0.25, 1],
                      [-0.643, -0.486, 0.25, 1],
                      [-0.744, -0.311, 0.25, 1]]

        self.p =      [[-0.642,  0.487, -0.05, 1],
                      [0.642,  0.487, -0.05, 1],
                      [0.743,  0.313, -0.05, 1],
                      [0.101,   -0.8, -0.05, 1],
                      [-0.101,   -0.8, -0.05, 1],
                      [-0.743,  0.313, -0.05, 1]]

        self.b = np.asarray(self.b)*self.MUL
        self.p = np.asarray(self.p)*self.MUL

        # self.sub = rospy.Subscriber('stewart/platform_twist', Twist, self.stewart_calback)
        self.keybaord_sub = rospy.Subscriber('/keyboard_input_py/raw_input', Int32, self.servoing_callback)
        self.pub = rospy.Publisher("/stewart/position_cmd", Float32MultiArray, queue_size=100)

        self._reset_pos()

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self._ik_and_publish(self.x,self.y,self.z,self.roll,self.pitch,self.yaw)
            rate.sleep()


    def _ik_and_publish(self,x,y,z,roll,pitch,yaw):
        T = transformation_matrix(x, y, z + self.height, roll, pitch, yaw)

        msg_out = Float32MultiArray()
        for i in range(6):
            length = np.matmul(T,np.transpose(self.p[i,:]))-np.transpose(self.b[i,:])
            d = np.linalg.norm(length) - self.height
            msg_out.data.append(d)
        self.pub.publish(msg_out)

    def _reset_pos(self):
        self.x,self.y,self.z,self.roll,self.pitch,self.yaw = 0,0,0,0,0,0

    #decode the keybaord input
    def servoing_callback(self, req , step = 0.1):
        #a=97, s =115, d =100, w =119, x=120
        #q = 113, e =101

        if req.data == 119:
            self.pitch += step
        elif req.data == 120:
            self.pitch += -step
        elif req.data == 97:
            self.roll += step
        elif req.data == 100:
            self.roll += -step
        elif req.data == 113:
            self.yaw += step/2.
        elif req.data == 101:
            self.yaw += -step/2.

        elif req.data == 115: #reset the platform
            self._reset_pos()


def transformation_matrix(x,y,z,r,p,yaw):
    T = [[np.cos(yaw)*np.cos(p), -np.sin(yaw)*np.cos(r) + np.cos(yaw)*np.sin(p)*np.sin(r),  np.sin(yaw)*np.sin(r)+np.cos(yaw)*np.sin(p)*np.cos(r), x],
         [np.sin(yaw)*np.cos(p),  np.cos(yaw)*np.cos(r) + np.sin(yaw)*np.sin(p)*np.sin(r), -np.cos(yaw)*np.sin(r)+np.sin(yaw)*np.sin(p)*np.cos(r), y],
         [-np.sin(p),                             np.cos(p)*np.sin(r),                         np.cos(p)*np.cos(yaw), z],
         [0.,                                         0.,                                       0., 1.]]
    return T


if __name__ == '__main__':

    rospy.init_node("stewart_ik")
    stewart_ik()
