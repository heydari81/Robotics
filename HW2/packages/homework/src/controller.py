#!/usr/bin/python3

import rospy
import tf
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from homework.srv import GetNextDestination
from homework.msg import proximity

from math import radians

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        rospy.Subscriber('/custom_scan', proximity, self.callback_custom)
        self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        self.linear_speed = rospy.get_param("/controller_node/linear_speed") # m/s
        self.angular_speed = -.8
        self.goal_angle = radians(0) # rad
        self.stop_distance = 2 # m
        self.distance = 0
        self.GO, self.ROTATE = 0, 1
        self.state = self.ROTATE
         
        
    def laser_callback(self, msg: LaserScan):
        if msg.ranges[0] <= self.stop_distance:
            self.state = self.ROTATE
    
    def get_heading_and_pose(self):
        
        msg = rospy.wait_for_message("/odom" , Odometry)
        orientation = msg.pose.pose.orientation
        position_y = msg.pose.pose.position.y
        position_x = msg.pose.pose.position.x
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return yaw ,position_x,position_y
    
    def get_next_destination(self):
     rospy.wait_for_service('get_next_destination')
     try:
        output = rospy.ServiceProxy('get_next_destination', GetNextDestination)
        response = output()
        return response.next_x, response.next_y
        pass
     except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None, None

    def callback_custom(self,msg):
     self.distance = msg.distance 

    def calculate_rotation(self,x, y, theta, next_x, next_y):
        delta_x = next_x - x
        delta_y = next_y - y
        target_angle = math.atan2(delta_y, delta_x)
        
        angle_diff = target_angle - theta
        
        
        rotate_degree = math.degrees(angle_diff)
        
        if rotate_degree > 180:
            rotate_degree -= 360
        elif rotate_degree < -180:
            rotate_degree += 360
        
        return rotate_degree


    def run(self):
        self.reamining = 0
        while not rospy.is_shutdown():
         next_x , next_y = self.get_next_destination();
         rospy.loginfo('next dest : x: %f , y :%f' , next_x,next_y)
         while (1):
            if self.linear_speed == .8 :
                if next_x == -4.6 and next_y == -4.3:
                    alpha  = 4
                elif next_x == -4.3 and next_y == 4.5:
                    alpha = -3
                elif next_x == 4.6 and next_y == -4.5 :
                    alpha = -12
                else :
                    alpha = -10
            elif self.linear_speed == 0.4 :
                if next_x == -4.6 and next_y == -4.3:
                    alpha  = 2
                elif next_x == -4.3 and next_y == 4.5:
                    alpha = -.5
                elif next_x == 4.6 and next_y == -4.5 :
                    alpha = -14
                else :
                    alpha = -2
            elif self.linear_speed == 0.2 :
                if next_x == -4.6 and next_y == -4.3:
                    alpha  = 1
                elif next_x == -4.3 and next_y == 4.5:
                    alpha = -.25
                elif next_x == 4.6 and next_y == -4.5 :
                    alpha = .5
                else :
                    alpha = 0
            prev_angle,position_x,position_y = self.get_heading_and_pose()
            rate = rospy.Rate(1)
            rate.sleep()
            if self.state == self.GO:
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_publisher.publish(twist)
                rospy.loginfo('self.distance : %f',self.distance)
                rospy.loginfo('self.distancestop : %f',self.stop_distance)
                rospy.loginfo('out if')
                if (self.distance <= 3):
                  rospy.loginfo('in if')
                  prev_angle,position_x,position_y = self.get_heading_and_pose()
                  rospy.loginfo('now we are : x: %f , y :%f' , position_x,position_y)
                  error = (position_x-next_x)**2 + (position_y-next_y)**2
                  error = error**(0.5)
                  rospy.loginfo('error : %f',error)
                  next_x , next_y = self.get_next_destination();
                  rospy.loginfo('next dest : x: %f , y :%f' , next_x,next_y)
                  self.state = self.ROTATE
                continue
            
            self.cmd_publisher.publish(Twist())            
            remaining = self.calculate_rotation(position_x,position_y,prev_angle,next_x,next_y)-alpha
            if remaining<0:
                twist = Twist()
                twist.angular.z = self.angular_speed
                self.cmd_publisher.publish(twist)
            else:
               twist = Twist()
               twist.angular.z = self.angular_speed*(-1)
               self.cmd_publisher.publish(twist)
            
            while abs(remaining) >= 1:
                prev_angle,position_x,position_y = self.get_heading_and_pose()
                remaining = self.calculate_rotation(position_x,position_y,prev_angle,next_x,next_y)-alpha
            
            self.cmd_publisher.publish(Twist())

            rospy.sleep(1)
            
            self.state = self.GO


if __name__ == "__main__":
    controller = Controller()
    controller.run()