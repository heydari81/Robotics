#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import tf
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi

class RobotFollower():
    def __init__(self):
        rospy.init_node('robot_follower', anonymous=False)
        rospy.on_shutdown(self.shutdown_procedure)
        rospy.Subscriber("/tb3_0/odom", Odometry, self.odom_data_callback)
        
        self.target_x = 0
        self.target_y = 0
        self.current_yaw = 0
        self.error_threshold = 0.005

        self.k_i = 0
        self.k_p = 0.2
        self.k_d = 0

        self.k_i_a = 0
        self.k_p_a = 2
        self.k_d_a = 0

        self.dt = 0.005
        self.v = 0
        rate = 1 / self.dt

        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=5)
        self.errs = []
        self.angle_errors = []

    def odom_data_callback(self, msg):
        self.target_x = msg.pose.pose.position.x
        self.target_y = msg.pose.pose.position.y
        rospy.loginfo('Target coordinates: x = %f, y = %f', self.target_x, self.target_y)

    def get_current_pose(self):
        msg = rospy.wait_for_message("/tb3_1/odom", Odometry)
        orientation = msg.pose.pose.orientation
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x, orientation.y, orientation.z, orientation.w
        )) 
        rospy.loginfo('x,y == %f , %f',position_x,position_y)
        return yaw, position_x, position_y


    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def go(self):
        sum_i_dist = 0
        prev_dist_error = 0

        sum_i_angle = 0
        prev_angle_error = 0

        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v
        error = 100

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            rospy.loginfo(move_cmd)
            current_yaw, current_x, current_y = self.get_current_pose()

            error = sqrt((self.target_x - current_x) ** 2 + (self.target_y - current_y) ** 2)
            rospy.loginfo(f"Target X: {self.target_x}")
            rospy.loginfo(f"Target Y: {self.target_y}")
            if error < 0.5:
                move_cmd.angular.z = 0
                move_cmd.linear.x = 0
                self.cmd_vel.publish(move_cmd)
            else:
                target_angle = atan2((self.target_y - current_y), (self.target_x - current_x))
                angle_error = self.normalize_angle(target_angle - current_yaw)

                self.errs.append(error)
                self.angle_errors.append(angle_error)

                sum_i_dist += error * self.dt
                sum_i_angle += angle_error * self.dt
                P = self.k_p * error
                I = self.k_i * sum_i_dist
                D = self.k_d * (error - prev_dist_error)
                Pa = self.k_p_a * angle_error
                Ia = self.k_i_a * sum_i_angle
                Da = self.k_d_a * (angle_error - prev_angle_error)

                prev_dist_error = error
                prev_angle_error = angle_error

                move_cmd.linear.x = self.v + min(P + I + D, 0.7)
                move_cmd.angular.z = min(Pa + Ia + Da, 1.5)

                self.r.sleep()

    def shutdown_procedure(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        plt.plot(list(range(len(self.errs))), self.errs, label='Distance Errors')
        plt.axhline(y=0.5, color='R')
        plt.draw()
        plt.show()
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        controller = RobotFollower()
        controller.go()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
