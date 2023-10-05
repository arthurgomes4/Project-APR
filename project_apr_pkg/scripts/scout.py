#! /usr/bin/python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
import heapq as hp
import math as m
from project_apr_pkg.srv import move, moveRequest, moveTrajRequest, moveTraj
from project_apr_pkg.msg import nav_path, lidar_points
import time

class scout:
    '''
    This class deals with maze mapping, solving and directs bot movements accordingly.
    '''
    def __init__(self, laser_topic, pos_topic):

        # odometry topic for bot localisation
        self.pos_topic = pos_topic

        # maze paraeters
        wall_width = 0.2
        self.cell_number = 9
        corridor_width = 0.8

        # SET THE GOAL
        self.goal_pos = (3,15)

        # divisor for cell identification
        self.divisor = (wall_width + corridor_width)/2

        # transfrom from corner frame {c} to centre frame {o}
        disp = (self.divisor + self.cell_number*(corridor_width + wall_width))/2
        self.T_co = np.array([[ 1,  0, disp],
                              [ 0, -1, disp],
                              [ 0,  0,    1]])

        # create 2D occupancy matrix where 0 = wall and 1 = clear. Borders are by default filled with 0's
        self.occ_matrix = np.ones((self.cell_number*2+1,self.cell_number*2+1), dtype=np.int_)
        self.occ_matrix[0:self.cell_number*2+1,                      0] = 0
        self.occ_matrix[                     0, 0:self.cell_number*2+1] = 0
        self.occ_matrix[0:self.cell_number*2+1,     self.cell_number*2] = 0
        self.occ_matrix[    self.cell_number*2, 0:self.cell_number*2+1] = 0

        # maze reset flag. Incase wrong values in occupancy matrix leads to a reset this flag is set high.
        self.MAZE_RESET = False

        # occupancy matrix update count.
        self.occ_matrix_update = 0

        # number of occ matrix updates needed before path plotting.
        self.occ_matrix_update_threshold = 3

        # flag is set high when matrix is being updated from lidar feed.
        self.MAZE_UPDATE_STATUS = False
        
        # flag is set high when bot is moving to lock the occ matrix from being updated
        self.LASER_CALLBACK_LOCK = False

        # list of all traversed nodes
        self.traversed_nodes = []

        # list of display topics 
        location_topic = '/location'
        nav_path_topic = '/nav_path'
        lidar_points_topic = '/lidar_points'

        # initialize node
        rospy.init_node('scout')

        # subscribe to the lidar topic
        rospy.Subscriber(laser_topic,LaserScan,self.laser_callback2)
        self.nav_path_pub = rospy.Publisher(nav_path_topic, nav_path, queue_size=1)
        self.lidar_points_pub = rospy.Publisher(lidar_points_topic, lidar_points, queue_size=1)

        # create tf listener object 
        self.tf_lis = tf.TransformListener()

        # service objects for requesting bot motion.
        self.service_container = rospy.ServiceProxy('move_to_goal', move)
        self.service_container_2 = rospy.ServiceProxy('move_traj', moveTraj)

        rospy.loginfo('node setup complete')

        # amount of time to delay before starting run (seconds)
        time.sleep(30)
        
        # start timer
        # start = time.time()

        # begin searching maze. This includes run/s to and from centre.
        self.search_maze()

        # perform search again if wrong values in occ matrix prevent complete search.
        while self.MAZE_RESET:
            self.MAZE_RESET = False
            self.search_maze()
        
        # find path to centre for blind run. In this mode the bot moves on a fixed path to the centre. 
        maze = np.copy(self.occ_matrix)
        path = self.find_path(maze, self.bot_pos, self.goal_pos)

        # cross check with traversed nodes and try again
        if len(path) == 0:
            self.correct_occ_matrix()
            maze = np.copy(self.occ_matrix)
            path = self.find_path(maze, self.bot_pos, self.goal_pos)
            if len(path) == 0:
                rospy.logwarn('MAZE correction unsuccessful')

        # in case a path to centre is not available, reset the occ matrix and perform search again.
        while len(path) == 0:
            self.reset_occ_matrix()
            while self.MAZE_RESET:
                self.MAZE_RESET = False
                self.search_maze()
            maze = np.copy(self.occ_matrix)
            path = self.find_path(maze, self.bot_pos, self.goal_pos)

        # stop timer for search time. start timer for run time.
        # search_time = time.time() - start
        # start = time.time()

        # rospy.loginfo('final run')

        # # request bot motion along fixed path to centre
        # self.request_traj(path)

        # # stop run time timer.
        # run_time = time.time() - start

        # # output final times.
        # rospy.loginfo('runtime = '+str(run_time)+' search time = '+str(search_time)+' total = '+str(run_time + search_time/30))

    def publish_lidar_points(self, points):
        lp_msg = lidar_points()
        lp_msg.x = list(points[0,:])
        lp_msg.y = list(points[1,:])
        self.lidar_points_pub.publish(lp_msg)


    def publish_nav_path(self,path):
        nav_path_msg = nav_path()
        rows = []
        columns = []
        for node in path:
            rows.append(node[0])
            columns.append(node[1])
        nav_path_msg.rows = rows
        nav_path_msg.columns = columns
        self.nav_path_pub.publish(nav_path_msg)

    # cross check the occ matrix for incorrect entries.
    def correct_occ_matrix(self):
        for k in range(len(self.traversed_nodes)-1):
            start = self.traversed_nodes[k]
            end = self.traversed_nodes[k+1]
            vec = (end[0] - start[0], end[1] - start[1])
            mag = abs(vec[0] + vec[1])
            unit = (vec[0]//mag, vec[1]//mag)

            for i in range(mag):
                new = (start[0] + i*unit[0],start[1] + i*unit[1])
                self.occ_matrix[new[0],new[1]] = 1

            self.occ_matrix[end[0],end[1]] = 1

    # perform a maze search.
    def search_maze(self):
        #================================ RUNNING TO THE CENTRE ====================================
        rospy.loginfo('moving to goal')

        # save the bot starting postion
        self.bot_pos = self.locate_bot(self.pos_topic)

        # save to traversed nodes
        self.traversed_nodes.append(self.bot_pos)

        # wait for occ matrix to finish updating if not complete already
        self.wait_for_occ_mat_update()

        # calculate path to centre
        maze = np.copy(self.occ_matrix)
        path = self.find_path(maze, self.bot_pos, self.goal_pos)

        # cross check with traversed nodes and try again
        if len(path) == 0:
            self.correct_occ_matrix()
            maze = np.copy(self.occ_matrix)
            path = self.find_path(maze, self.bot_pos, self.goal_pos)
            if len(path) == 0:
                rospy.logwarn('MAZE correction unsuccessful')

        # if path cannot be found reset occ matrix and try again.
        while len(path) == 0:
            self.reset_occ_matrix()

            self.occ_matrix_update = 0
            self.wait_for_occ_mat_update()

            maze = np.copy(self.occ_matrix)
            path = self.find_path(maze, self.bot_pos, self.goal_pos)
            
        # find the nearest direction change in the path.
        node,path = self.find_dir_change(path)

        # save to traversed nodes
        self.traversed_nodes.append(node)

        # request movement till that direction change.
        self.request_move(node[0],node[1])

        while not rospy.is_shutdown():
            
            # wait for occ matrix to finish updating if not complete already
            self.wait_for_occ_mat_update()

            # check if existing path to centre is blocked by newly detected walls. 
            # If yes then calculate a new path.
            maze = np.copy(self.occ_matrix)
            if self.check_path(maze, path):
                path = self.find_path(maze, node, self.goal_pos)
            
            # if a path to the centre cannot be found
            if len(path) == 0:

                # locate the bots grid position
                temp_bot_pos = self.locate_bot(self.pos_topic)

                # if not in the maze centre then reset occ matrix and try again.
                if temp_bot_pos != self.goal_pos:

                    # cross check with traversed nodes and try again
                    self.correct_occ_matrix()
                    maze = np.copy(self.occ_matrix)
                    path = self.find_path(maze, temp_bot_pos, self.goal_pos)
                    if len(path) == 0:
                        rospy.logwarn('MAZE correction unsuccessful')
                        
                    while len(path) == 0:
                        self.reset_occ_matrix()

                        self.occ_matrix_update = 0
                        self.wait_for_occ_mat_update()

                        maze = np.copy(self.occ_matrix)
                        path = self.find_path(maze, temp_bot_pos, self.goal_pos)
                else:
                    break
            
            # find the nearest direction change in the path.
            node,path = self.find_dir_change(path)

            # save to traversed nodes
            self.traversed_nodes.append(node)

            # request movement till that direction change.
            self.request_move(node[0],node[1])

        #================================ RUNNING BACK TO START ===================================
        rospy.loginfo('moving back home')

        # wait for occ matrix to finish updating if not complete already
        self.wait_for_occ_mat_update()

        # calculate path to start
        maze = np.copy(self.occ_matrix)
        path = self.find_path(maze, self.goal_pos, self.bot_pos)

        # cross check with traversed nodes and try again
        if len(path) == 0:
            self.correct_occ_matrix()
            maze = np.copy(self.occ_matrix)
            path = self.find_path(maze, self.goal_pos, self.bot_pos)

            if len(path) == 0:
                rospy.logwarn('MAZE correction unsuccessful')

        # if path cannot be found reset occ matrix and try again.
        while len(path)==0:
            self.reset_occ_matrix()

            self.occ_matrix_update = 0
            self.wait_for_occ_mat_update()

            maze = np.copy(self.occ_matrix)
            path = self.find_path(maze, self.goal_pos, self.bot_pos)
            
        # find the nearest direction change in the path.
        node,path = self.find_dir_change(path)

        # save to traversed nodes
        self.traversed_nodes.append(node)

        # request movement till that direction change.
        self.request_move(node[0],node[1])

        while not rospy.is_shutdown():
            
            # wait for occ matrix to finish updating if not complete already
            self.wait_for_occ_mat_update()

            # check if existing path to centre is blocked by newly detected walls. 
            # If yes then calculate a new path.
            maze = np.copy(self.occ_matrix)
            if self.check_path(maze, path):
                path = self.find_path(maze, node, self.bot_pos)
            
            # if a path to the centre cannot be found
            if len(path)==0:

                # locate the bots grid position
                temp_bot_pos = self.locate_bot(self.pos_topic)

                # if not in the maze centre then reset occ matrix and try again.
                if temp_bot_pos != self.bot_pos:

                    # cross check with traversed nodes and try again
                    self.correct_occ_matrix()
                    maze = np.copy(self.occ_matrix)
                    path = self.find_path(maze, temp_bot_pos, self.bot_pos)
                    if len(path) == 0:
                        rospy.logwarn('MAZE correction unsuccessful')

                    while len(path)==0:
                        self.reset_occ_matrix()

                        self.occ_matrix_update = 0
                        self.wait_for_occ_mat_update()

                        maze = np.copy(self.occ_matrix)
                        path = self.find_path(maze, temp_bot_pos, self.bot_pos)
                else:
                    break
            
            # find the nearest direction change in the path.
            node,path = self.find_dir_change(path)

            # save to traversed nodes
            self.traversed_nodes.append(node)

            # request movement till that direction change.
            self.request_move(node[0],node[1])

    # reset the occ matrix
    def reset_occ_matrix(self):

        self.occ_matrix = np.ones((self.cell_number*2+1,self.cell_number*2+1), dtype=np.int_)
        self.occ_matrix[0:self.cell_number*2+1,                 0] = 0
        self.occ_matrix[                0, 0:self.cell_number*2+1] = 0
        self.occ_matrix[0:self.cell_number*2+1,     self.cell_number*2] = 0
        self.occ_matrix[    self.cell_number*2, 0:self.cell_number*2+1] = 0

        rospy.logwarn('MAZE RESET')

        #set warning flag
        self.MAZE_RESET = True

    #waits for number of occ matrix updates to reach threshold
    def wait_for_occ_mat_update(self):
        
        while self.occ_matrix_update < self.occ_matrix_update_threshold:
            pass
        return

    # locate the bot from odometry messages
    def locate_bot(self, pos_topic):

        odom_msg = rospy.wait_for_message(pos_topic, Odometry)
        o_point = [[odom_msg.pose.pose.position.x], [odom_msg.pose.pose.position.y],[1]]
        c_point = np.matmul(self.T_co, o_point)
        element = c_point//self.divisor
        return (int(element[1]), int(element[0]))

    # request bot movement to a given grid location
    def request_move(self,i,j):
        rospy.wait_for_service('move_to_goal')
        try:
            self.LASER_CALLBACK_LOCK = True
            self.occ_matrix_update = 0
            while self.MAZE_UPDATE_STATUS:
                pass
            req = moveRequest()
            req.i = i
            req.j = j
            resp = self.service_container(req)
            self.LASER_CALLBACK_LOCK = False
            
            return resp.ack
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # request bot movement along a trajectory of grid locations
    def request_traj(self,path):
        rospy.wait_for_service('move_traj')
        try:
            req = moveTrajRequest()
            for node in path:
                req.i.append(node[0])
                req.j.append(node[1])
            resp = self.service_container_2(req)
            return resp.ack
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # find the immediate direction change in a path
    def find_dir_change(self,path):
        dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])

        for i in range(1,len(path)-1):
            if (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]) != dir:
                return path[i], path[i:]

        return path[-1], list()

    # check path for obstacles
    def check_path(self, maze, path):
        # path is a list of tuples
        for node in path:
            if not maze[node[0],node[1]]:
                return 1
        return 0

    # find the path between two given nodes using A*
    def find_path(self, maze, start, goal):
        maze_dims = (maze.shape[0], maze.shape[1])

        open_nodes = []
        closed_nodes = np.zeros(maze_dims, dtype=np.bool)
        past_cost_matrix = np.full(maze_dims, np.Inf)
        parent_matrix = np.full((maze_dims[0],maze_dims[1],2), None)

        # initializing OPEN with start
        # heuristic added cost, node_pos
        hp.heappush(open_nodes, (0,start))
        past_cost_matrix[start[0], start[1]] = 0

        while len(open_nodes) > 0:

            current = hp.heappop(open_nodes)[1]
            closed_nodes[current[0], current[1]] = True

            if current == goal:
                path = []
                while current != (None,None):
                    path.append(current)
                    current = (parent_matrix[current[0], current[1], 0], parent_matrix[current[0], current[1], 1])

                # to display path ============
                # for node in path:
                #     maze[node[0],node[1]] = 2
                # for row in maze:
                #     print(row)
                # ============================
                self.publish_nav_path(path[::-1])
                return path[::-1]

            # find neighbours of CURRENT
            # create a neighbours list of each row having current_ctg, node_pos

            directions = ((-1,  0), # north
                        ( 0, -1), # west
                        ( 1,  0), # south
                        ( 0,  1)) # east

            for dir in directions:
                neighbour = (current[0] + dir[0], current[1] + dir[1])
                if closed_nodes[neighbour[0], neighbour[1]] or not maze[neighbour[0], neighbour[1]]:
                    continue
                else:
                    # temp = past cost of CURRENT + Heuristic of neighbour + 1 (step size)
                    temp = past_cost_matrix[current[0], current[1]] + 1
                    
                    if temp < past_cost_matrix[neighbour[0], neighbour[1]]:

                        past_cost_matrix[neighbour[0], neighbour[1]] = temp
                        parent_matrix[neighbour[0], neighbour[1]] = current

                        # we have to change the past cost in OPEN as well
                        k=0
                        for i in range(len(open_nodes)):
                            if open_nodes[i][1] == neighbour:
                                try:
                                    open_nodes[i][1] = temp + m.sqrt((goal[0]-neighbour[0])**2 + (goal[1]-neighbour[1])**2)
                                except TypeError:
                                    return list()
                                hp.heapify(open_nodes)
                                k = 1
                                break
                        if not k:        
                            hp.heappush(open_nodes, (temp + m.sqrt((goal[0]-neighbour[0])**2 + (goal[1]-neighbour[1])**2), neighbour))
        return list()

    # convert lidar values to points in lidars 2D plane.
    def lasermsg_2_xy(self, msg):
        distances = np.array(msg.ranges)
        theta = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment*0.5, msg.angle_increment)
        X = distances*np.cos(theta)
        Y = distances*np.sin(theta)
        l_points = np.stack((X,Y,np.ones((len(X)))))
        return l_points

    # callback for lidar messages. Uses latest available transform
    def laser_callback1(self, laser_msg):

        if self.LASER_CALLBACK_LOCK:
            return
        
        self.MAZE_UPDATE_STATUS = True
        # calculate points in lidar frame
        l_points = self.lasermsg_2_xy(laser_msg)

        # calculate points in corner frame
        try:
            t = self.tf_lis.lookupTransform('/odom', '/chassis', rospy.Time(0))
            (x,y) = t[0][0:2]
            (roll, pitch, yaw) = euler_from_quaternion(t[1])
            yaw *= -1
            T_ol = np.array([[np.cos(yaw), np.sin(yaw), x],[-np.sin(yaw), np.cos(yaw), y],[0,0,1]])
            T_cl = np.matmul(self.T_co, T_ol)

            c_points = np.matmul(T_cl, l_points)
            c_points = c_points//self.divisor
            elements = c_points[0:2, :]

            # update the occ matrix 
            for point in np.transpose(elements):
                try:
                    self.occ_matrix[int(point[1]),int(point[0])] = 0
                except IndexError:
                    pass

            self.occ_matrix_update += 1
            self.MAZE_UPDATE_STATUS = False

        except tf.LookupException:
            pass

    # callback for lidar messages. Uses transform from the same time stamp as the lidar message. takes more time but gives better results
    def laser_callback2(self, laser_msg):

        
        # calculate points in lidar frame
        l_points = self.lasermsg_2_xy(laser_msg)

        while True and not rospy.is_shutdown():
            try:
                t = self.tf_lis.lookupTransform('/odom', '/chassis', laser_msg.header.stamp)
                break
            except (tf.LookupException, tf.ExtrapolationException) as e:
                if 'past' in str(e):
                    return
                
        (x,y) = t[0][0:2]
        (roll, pitch, yaw) = euler_from_quaternion(t[1])
        yaw *= -1
        T_ol = np.array([[np.cos(yaw), np.sin(yaw), x],[-np.sin(yaw), np.cos(yaw), y],[0,0,1]])
        T_cl = np.matmul(self.T_co, T_ol)

        c_points = np.matmul(T_cl, l_points)
        self.publish_lidar_points(c_points)
        c_points = c_points//self.divisor
        elements = c_points[0:2, :]

        if self.LASER_CALLBACK_LOCK:
            return
        
        self.MAZE_UPDATE_STATUS = True

        for point in np.transpose(elements):
            try:
                self.occ_matrix[int(point[1]),int(point[0])] = 0
            except IndexError:
                pass
   
        self.occ_matrix_update += 1
        self.MAZE_UPDATE_STATUS = False

if __name__ == '__main__':
    scout('laser_scan','odom')
