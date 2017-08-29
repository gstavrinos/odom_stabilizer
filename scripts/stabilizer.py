#!/usr/bin/env python
import tf
import rospy
import traceback
from tf import TransformListener
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

twist_publisher = None

def init():
    global twist_publisher
    rospy.init_node('odom_stabilizer')
    twist_topic = rospy.get_param("~twist_topic", "bebop/cmd_vel")
    rospy.Subscriber("bebop/odom", Odometry, odom_callback)
    twist_publisher = rospy.Publisher(twist_topic, Twist, queue_size=1);
    while not rospy.is_shutdown():
        rospy.spin()

def odom_callback(msg):
    global twist_publisher
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    twist = Twist()
    if position.z > 0.5:
        twist.linear.x = -position.x * 0.8
        print 'position x = ' + str(position.x)
        print 'vel x = ' + str(twist.linear.x)
        twist.linear.y = -position.y * 0.8
        print 'position y = ' + str(position.y)
        print 'vel y = ' + str(twist.linear.y)
        twist.linear.z = 0.6 - position.z * 0.8
        print 'position z = ' + str(position.z)
        print 'vel z = ' + str(twist.linear.z)
    twist_publisher.publish(twist)

if __name__ == '__main__':
    init() 