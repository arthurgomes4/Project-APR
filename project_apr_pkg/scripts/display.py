#! /usr/bin/python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from copy import copy
from nav_msgs.msg import Odometry
from project_apr_pkg.msg import nav_path,lidar_points

class display:

    def __init__(self, name, pos_topic, lidar_points_topic, nav_path_topic):

        # frame size
        self.maxP = 600

        #maze parameters
        wall_width = 0.2
        cell_number = 9
        corridor_width = 0.8

        # make divisor
        self.divisor = (wall_width + corridor_width)/2

        # transfrom from corner frame {c} to centre frame {o}
        disp = (self.divisor + cell_number*(corridor_width + wall_width))/2
        self.T_oc = np.array([[ 1,  0, -disp],
                              [ 0, -1, disp],
                              [ 0,  0,    1]])
        
        self.T_co = np.array([[ 1,  0, disp],
                              [ 0, -1, disp],
                              [ 0,  0,    1]])

        self.position = ()
        self.lp_points = []
        self.way = []
        self.lidar_frame = np.zeros((self.maxP, self.maxP,3), dtype=np.uint8)
        
        rospy.init_node(name)
        rospy.Subscriber(pos_topic, Odometry, self.pos_callback)
        rospy.Subscriber(lidar_points_topic, lidar_points, self.lp_callback)
        rospy.Subscriber(nav_path_topic, nav_path, self.nav_path_callback)

        start_pos = self.xy_to_pixel(-3,-2)
        end_pos = self.xy_to_pixel(3,3)

        while len(self.position) == 0:
            pass

        while not rospy.is_shutdown():
            frame = np.zeros((self.maxP, self.maxP,3), dtype=np.uint8)
            frame = cv2.circle(frame, start_pos, 8, (255,0,0), 2 )
            frame = cv2.circle(frame, end_pos, 8, (255,0,0), 2 )

            
            if len(self.lp_points) > 0:
                for pt in self.lp_points:
                    self.lidar_frame[pt[1]:pt[1]+2,pt[0]:pt[0]+2] = (0,255,0)
            
            if len(self.way) > 0:
                temp = copy(self.way)
                for i in range(len(temp)-1):
                    x1,y1 = self.mat2point(temp[i][0],temp[i][1])
                    x2,y2 = self.mat2point(temp[i+1][0],temp[i+1][1])

                    x1,y1 = self.xy_to_pixel(x1,y1)
                    x2,y2 = self.xy_to_pixel(x2,y2)

                    frame = cv2.line(frame, (x1,y1), (x2,y2), (255,255,0), 2)

            frame = cv2.circle(frame, self.position, 7, (0,0,255), -1)
            frame = frame + self.lidar_frame
            cv2.imshow('viz',frame)
            cv2.waitKey(1)


    def pos_callback(self, odom_msg):
        self.position = self.xy_to_pixel(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)

    def xy_to_pixel(self,x,y):

        maxX, minX = 5, -5
        maxY, minY = -5, 5

        return (int(self.maxP*(x - minX)//(maxX - minX)), int(self.maxP*(y - minY)//(maxY - minY)))

    def lp_callback(self, msg):
        points = np.array([msg.x, msg.y, [1]*len(msg.x)])
        points = np.matmul(self.T_oc, points)
        temp = []
        for i in range(points.shape[1]):
            temp.append(self.xy_to_pixel(points[0,i],points[1,i]))

        self.lp_points = copy(temp)

    def nav_path_callback(self, msg):
        self.way = []
        for i in range(len(msg.rows)):
            self.way.append((msg.rows[i], msg.columns[i]))

        print(self.way)

    def mat2point(self,i,j):
        c_point = np.array([[j*self.divisor + self.divisor/2],[i*self.divisor + self.divisor/2],[1]]) 
        o_point = np.matmul(self.T_oc, c_point)
        return float(o_point[0]), float(o_point[1])


if __name__ == '__main__':

    o = display('display', '/odom','/lidar_points', '/nav_path')