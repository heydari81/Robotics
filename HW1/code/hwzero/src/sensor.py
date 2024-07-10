#!/usr/bin/python3
import rospy
from hwzero.msg import proximity
import json
import os
iteration = 1
def read_distance(iteration):
    script_dir = os.path.dirname(__file__)
    file_path = os.path.join(script_dir, 'distances.json') 
    with open(file_path, 'r') as f:
        distance_data = json.load(f)
    return [int(x) for x in distance_data[str(iteration)].split()]

def distance_sensor_node_define():
    rospy.init_node("sensor",anonymous=False)
    publisher = rospy.Publisher("distance",proximity,queue_size=10)
    rate = rospy.Rate(0.75)
    return publisher,rate
def start():
 publisher,rate = distance_sensor_node_define()
 global iteration
 while not rospy.is_shutdown() and iteration<=10:
    rate.sleep()
    rospy.loginfo(iteration)
    left, up, right, down = read_distance(iteration)
    distance_msg = proximity()
    distance_msg.left = left
    distance_msg.up = up
    distance_msg.right = right
    distance_msg.down = down
    publisher.publish(distance_msg)
    iteration = iteration+1
if __name__ == '__main__':
    start()

