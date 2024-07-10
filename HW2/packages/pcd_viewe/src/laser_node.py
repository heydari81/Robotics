#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2,LaserScan
from laser_assembler.srv import AssembleScans2

class laser_node:
    def __init__(self):
        rospy.init_node('laser_node')
        
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pc_pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)
        
        rospy.wait_for_service('assemble_scans2')
        self.assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        
    def scan_callback(self, scan): 
        try:
            response = self.assemble_scans(rospy.Time(0,0), rospy.get_rostime())
            self.pc_pub.publish(response.cloud)
        except rospy.ServiceException as e:
            rospy.logerr("failed: %s", e)

if __name__ == '__main__':
    try:
        node = laser_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
