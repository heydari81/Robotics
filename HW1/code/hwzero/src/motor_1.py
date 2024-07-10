#!/usr/bin/python3
import rospy
from hwzero.msg import order

def engine_node_callback1(data):
    rospy.loginfo("engine1 :%s",data)

def start3():
    rospy.init_node('engine1_node', anonymous=False) 
    rate = rospy.Rate(.75)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.Subscriber('engine1_order', order, engine_node_callback1)

if __name__ == '__main__':
    start3()
