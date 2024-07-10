#!/usr/bin/python3
import rospy
from hwzero.msg import order

def engine_node_callback2(data):
    rospy.loginfo("engine2 :%s",data)

def start3():
    rospy.init_node('engine2_node', anonymous=False)  # Initialize the node once
    rate = rospy.Rate(.75)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.Subscriber('engine2_order', order, engine_node_callback2)

if __name__ == '__main__':
    start3()
