#! /usr/bin/python3

from math import cos, sin
import rospy
from geometry_msgs.msg import Twist

class vex:
    '''
    This class is a velocity decoder for APR
    '''

    # callback
    def callback(self,msg):
        scaler = 1
        ang_offset = -0.785
        x_msg = Twist()
        x_msg.linear.x = scaler*(msg.linear.x*cos(ang_offset) - msg.linear.y*sin(ang_offset))
        x_msg.angular.z = scaler*msg.angular.z
        y_msg = Twist()
        y_msg.linear.x = scaler*(msg.linear.x*sin(ang_offset) + msg.linear.y*cos(ang_offset))
        y_msg.angular.z = scaler*msg.angular.z
        self.x_vel_pub.publish(x_msg)
        self.y_vel_pub.publish(y_msg)

    def __init__(self, x_vel_topic,y_vel_topic,combined_topic):

        # init node
        rospy.init_node('vex')

        # sub to combined velocity topic
        vel_sub = rospy.Subscriber(combined_topic, Twist, self.callback)

        # pub to separate X direction
        self.x_vel_pub = rospy.Publisher(x_vel_topic,Twist,queue_size=1)

        # pub to separate Y direction
        self.y_vel_pub = rospy.Publisher(y_vel_topic,Twist,queue_size=1)

        rospy.loginfo('waiting for '+ combined_topic)
        rospy.spin()

if __name__ == '__main__':
    vex("/cmd_vel_x","/cmd_vel_y","/cmd_vel")