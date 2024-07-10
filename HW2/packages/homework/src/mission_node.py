#!/usr/bin/python3

import rospy
from homework.srv import GetNextDestination,GetNextDestinationResponse
import re
import os


def parse_coordinates(line):
    match = re.match(r'x:\s*([-+]?\d*\.\d+|\d+),\s*y:\s*([-+]?\d*\.\d+|\d+)', line)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return x, y
    else:
        rospy.logerr("Invalid line format: %s", line)
        return None, None


line_number = 1
def read_destination_from_line():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    file_path = os.path.join(script_dir, "obstacles.txt")
    try:
        with open(file_path, "r") as file:
            for _ in range(line_number - 1):
                file.readline()
            line = file.readline().strip()
            next_x, next_y = parse_coordinates(line)
            return next_x, next_y
    except IOError as e:
        rospy.logerr("Failed to read file: %s", str(e))
        return None, None


def handle_get_next_destination(req):
    global line_number
    x, y = read_destination_from_line()
    line_number = line_number+1
    if line_number == 5 :
        line_number = 1
    return GetNextDestinationResponse(next_x = x,next_y = y)

def mission_node():
    rospy.init_node('mission_node')
    service = rospy.Service('get_next_destination', GetNextDestination, handle_get_next_destination)
    rospy.spin()

if __name__ == "__main__":
    mission_node()
