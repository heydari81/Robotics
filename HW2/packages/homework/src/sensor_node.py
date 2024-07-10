#!/usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from homework.msg import proximity 

class SensorNode:
    def __init__(self):
        rospy.init_node('sensor_node')
        self.pub_custom = rospy.Publisher('/custom_scan', proximity, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def laser_callback(self, msg):
        min_front_distance = (msg.ranges[0:30])
        min_front_distance2 = (msg.ranges[330:359])
        min_front_distance = min(min_front_distance)
        min_front_distance2 = min(min_front_distance2)
        min_front = min(min_front_distance,min_front_distance2)
        custom_msg = proximity()
        custom_msg.distance = min_front 
        self.pub_custom.publish(custom_msg)

    def shutdown(self):
        rospy.loginfo("Shutting down sensor_node.")

if __name__ == "__main__":
    SensorNode()
