#! /usr/bin/python3

import rospy
import numpy as np
import math as m
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from project_apr_pkg.srv import move, moveResponse, moveTraj, moveTrajResponse
from project_apr_pkg.msg import lidar_points
from tf.transformations import euler_from_quaternion

class walker():
    '''
    This class offers movement services.
    '''
    def __init__(self, name, vel_topic, pos_topic):

        #===========================================================
        # set motion parameters for bot
        #===========================================================
        # max velocity
        self.bot_velocity = 0.5

        # position error tolerance for the P controller 
        self.position_tolerance = 0.05

        # P gain
        self.linear_gain = 3.0

        # P gain for angular corrective controller
        self.angular_gain = 1

        # angle tolerance
        self.angular_tolerance = 0.005
        #============================================================

        # postion of the bot currently
        self.position = []
        # yaw of the bot
        self.yaw = None

        # maze parameters
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

        # init node
        rospy.init_node(name)

        # subscribe to odometry topic
        rospy.Subscriber(pos_topic, Odometry, self.odom_callback)

        # create publisher object to publish twist messages
        self.vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=1)

        # set rate of controller
        self.controller_rate = rospy.Rate(30)

        # offer movement services 
        self.service_container = rospy.Service('move_to_goal', move, self.handle_move_to_goal)
        self.traj_sc = rospy.Service('move_traj', moveTraj, self.handle_move_traj)

        # wait for odom to become available
        while len(self.position) == 0:
            pass   
        
        rospy.loginfo('correcting bot orientation')
        curr = self.point2mat(self.position[0], self.position[1])
        self.move_to_goal(curr[0],curr[1], onp=True)

        rospy.loginfo('waiting for requests')
        rospy.spin()
        rospy.loginfo('node shutting down')

    # convert x,y location to grid location
    def point2mat(self,x,y):

        o_point = [[x], [y],[1]]
        c_point = np.matmul(self.T_co, o_point)
        element = c_point//self.divisor
        return (int(element[1]), int(element[0]))

    # handle function for move_traj service
    def handle_move_traj(self, req):
        path = []
        for a in range(len(req.i)):
            path.append((req.i[a], req.j[a]))

        waypoints = []
        waypoints.append(path[0])
        dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])

        for i in range(1,len(path)- 1):
            new_dir = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            if new_dir != dir:
                waypoints.append((path[i][0] - dir[0], path[i][1] - dir[1]))
                waypoints.append((path[i][0] + new_dir[0], path[i][1] + new_dir[1]))
                dir = new_dir[:]
        waypoints.append(path[-1])

        path = waypoints[:]

        waypoints = []

        prev = (None, None)
        for node in path:
            if node != prev:
                waypoints.append(node)
                prev = node

        for i in range(len(waypoints)):
            # find the corresponding global goal
            x,y = self.mat2point(waypoints[i][0],waypoints[i][1])

            rospy.loginfo('moving to ' + str(x) + ',' + str(y))
            Ex,Ey = self.linear_error(x,y)
            vel_msg = Twist()

            while abs(Ex) > self.position_tolerance or abs(Ey) > self.position_tolerance:

                # calculate and publish velocity
                Vx = self.linear_gain*Ex
                Vy = self.linear_gain*Ey

                vel_msg.linear.x, vel_msg.linear.y = self.cap_velocity(Vx,Vy)
                vel_msg.angular.z = self.correct_orientation()
                self.vel_pub.publish(vel_msg)

                Ex,Ey = self.linear_error(x,y)

                self.controller_rate.sleep()

            stop_msg = Twist()
            self.vel_pub.publish(stop_msg)
            rospy.loginfo('reached!')
        
        resp = moveTrajResponse()
        resp.ack = True
        return resp

    # handle function for move_to_goal service
    def handle_move_to_goal(self, req):
        ret = self.move_to_goal(req.i, req.j)
        resp = moveResponse()
        resp.ack = ret
        return resp
    
    # convert a grid location to a (x,y) location
    def mat2point(self,i,j):
        c_point = np.array([[j*self.divisor + self.divisor/2],[i*self.divisor + self.divisor/2],[1]]) 
        o_point = np.matmul(self.T_oc, c_point)
        return float(o_point[0]), float(o_point[1])

    # moves bot to given goal using P controller
    def move_to_goal(self, i,j, onp=False):

        # find the corresponding global goal
        x,y = self.mat2point(i,j)

        rospy.loginfo('moving to ' + str(x) + ',' + str(y))
        Ex,Ey = self.linear_error(x,y)
        vel_msg = Twist()
        # print(Ex,Ey)

        while abs(Ex) > self.position_tolerance or abs(Ey) > self.position_tolerance or onp*(abs(self.yaw) > self.angular_tolerance) :

            # calculate and publish velocity
            Vx = self.linear_gain*Ex
            Vy = self.linear_gain*Ey

            vel_msg.linear.x, vel_msg.linear.y = self.cap_velocity(Vx,Vy)
            vel_msg.angular.z = self.correct_orientation()
            self.vel_pub.publish(vel_msg)

            Ex,Ey = self.linear_error(x,y)

            self.controller_rate.sleep()

        stop_msg = Twist()
        for i in range(5):
            self.vel_pub.publish(stop_msg)
        
        rospy.loginfo('reached!')
        return True
   
    # calculate a corrective rotation to keep yaw at 0 rad
    def correct_orientation(self):
        return -self.angular_gain*self.yaw

    # limit velocity to set limit while maintaining bot direction
    def cap_velocity(self,Vx,Vy):
        v_mag = m.sqrt(Vx**2 + Vy**2)
        if v_mag > self.bot_velocity:
            Vx = self.bot_velocity*Vx/v_mag
            Vy = self.bot_velocity*Vy/v_mag
        return Vx,Vy

    # calculate linear position error along both axes
    def linear_error(self, x,y):
        return x - self.position[0], y - self.position[1]

    # callback function for odometry
    def odom_callback(self, msg):
        self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        (roll, pitch, self.yaw) = euler_from_quaternion([q.x,q.y,q.z,q.w])

if __name__ == '__main__':
    walker('walker', '/cmd_vel', '/odom')
