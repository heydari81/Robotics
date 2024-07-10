#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
import tf 
from nav_msgs.msg import Odometry
import numpy as np
from math import sqrt, atan2
from math import sqrt
from math import atan2
import  math
import matplotlib.pyplot as plt


class PathFollower():
    def __init__(self,path_number):
        rospy.init_node('path_fallower', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.goal_x, self.goal_y = self.generate_path(path_number)
        # PID parameters
        self.k_i,self.k_p,self.k_d,self.k_i_a,self.k_p_a,self.k_d_a = self.generate_PID_param(path_number)

        self.dt = 0.005
        self.v = 0
        self.D = 2
        rate = 1 / self.dt
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.errs = []
        self.errs1 = []
    def generate_PID_param(self,path_number):
        if path_number == 0:
           k_i = 0
           k_p = 2.5
           k_d = 0
           k_i_a = 0
           k_p_a = 2.5
           k_d_a = 30
        elif path_number == 1:
           k_i = 0.07
           k_p = 0.7
           k_d = 3
           k_i_a = 0.8
           k_p_a = 5
           k_d_a = 80 
        elif path_number == 2:
           k_i = 0.01
           k_p = 0.1
           k_d = 1
           k_i_a = 0.4
           k_p_a = 4
           k_d_a = 40
        elif path_number == 3:
           k_i = 0.05
           k_p = 0.5
           k_d = 5
           k_i_a = 0.7
           k_p_a = 7
           k_d_a = 70
        return k_i,k_p,k_d,k_i_a,k_p_a,k_d_a
    def generate_path(self,path_number):
        rospy.loginfo('path:: %d',path_number)
        if path_number == 0:
            X = np.linspace(0, 4 , 100)
            Y = X
        elif path_number == 1:
            X1 = np.linspace(-1, 1 , 100)
            Y1 = np.array([3]*100)

            X2 = np.linspace(1, 1 + 2**(1/2) , 100)
            Y2 = - (2**(1/2)) * (X2 - 1) + 3

            Y3 = np.linspace(1, -1 , 100)
            X3 = np.array([1 + 2**(1/2)]*100)

            X4 = np.linspace(1 + 2**(1/2), 1, 100)
            Y4 = (2**(1/2)) * (X4 - 1 - 2**(1/2)) -1 

            X5 = np.linspace(1, -1 , 100)
            Y5 = np.array([-3]*100)

            X6 = np.linspace(-1, -1 - 2**(1/2) , 100)
            Y6 = - (2**(1/2)) * (X6 + 1) - 3 

            Y7 = np.linspace(-1, 1 , 100)
            X7 = np.array([- 1 - 2**(1/2)]*100)

            X8 = np.linspace(-1 - 2**(1/2), -1, 100)
            Y8 = (2**(1/2)) * (X8 + 1 + 2**(1/2)) + 1

            X = np.concatenate([X1, X2, X3, X4, X5, X6, X7, X8])
            Y = np.concatenate([Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8])
        elif path_number == 2:
            X1 = np.linspace(-6., -2 , 50)
            Y1 = np.zeros((50,))

            x_dim, y_dim = 2,2
            t = np.linspace(np.pi, 0, 100)
            X2 = x_dim * np.cos(t) 
            Y2 = y_dim * np.sin(t)

            X3 = np.linspace(2, 6 , 50)
            Y3 = np.zeros((50,))

            x_dim, y_dim = 6,6
            t = np.linspace(np.pi*2, np.pi, 200)
            X4 = x_dim * np.cos(t) 
            Y4 = y_dim * np.sin(t)
            X = np.concatenate([X1, X2, X3, X4])
            Y = np.concatenate([Y1, Y2, Y3, Y4])
        elif path_number == 3:
            # logarithmic spiral 
            a = 0.17
            k = math.tan(a)
            X , Y = [] , []

            for i in range(150):
                t = i / 20 * math.pi
                dx = a * math.exp(k * t) * math.cos(t)
                dy = a * math.exp(k * t) * math.sin(t)
                X.append(dx)
                Y.append(dy) 
            X = np.concatenate([X])
            Y = np.concatenate([Y])
        return X.tolist(), Y.tolist()

    def get_current_position(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        orientation = msg.pose.pose.orientation
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x, orientation.y, orientation.z, orientation.w
        )) 
        rospy.loginfo('x,y == %f , %f',position_x,position_y)
        return yaw, position_x, position_y

    def go(self):
        sum_i_dist = 0
        prev_dist_error = 0

        sum_i_theta = 0
        prev_theta_error = 0

        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v

        while not rospy.is_shutdown():
            for i in range(len(self.goal_x)):
                while not rospy.is_shutdown() and i<len(self.goal_x):
                    self.cmd_vel.publish(move_cmd)
                    theta_perv, x_perv, y_perv = self.get_current_position()

                    err = sqrt((self.goal_x[i] - x_perv)**2 + (self.goal_y[i] - y_perv)**2)
                    if err < 0.1 :
                        move_cmd.angular.z = 0
                        move_cmd.linear.x = 0
                        self.cmd_vel.publish(move_cmd)
                        break

                    self.theta = atan2((self.goal_y[i] - y_perv), (self.goal_x[i] - x_perv))
                    err_th = (self.theta - theta_perv) 
                    while err_th > math.pi:
                        err_th -= 2 * math.pi
                    while err_th < -math.pi:
                        err_th += 2 * math.pi
                    self.errs.append(err)

                    sum_i_dist += err * self.dt
                    sum_i_theta += err_th * self.dt

                    P = self.k_p * err
                    I = self.k_i * sum_i_dist
                    D = self.k_d * (err - prev_dist_error)
                    
                    Pa = self.k_p_a * err_th
                    Ia = self.k_i_a * sum_i_theta
                    Da = self.k_d_a * (err_th - prev_theta_error)

                    prev_dist_error = err
                    prev_theta_error = err_th
                    plant_input = P+I+D
                    rospy.loginfo('plant input : %f',plant_input)
                    plant_a_input = Pa+Ia+Da
                    move_cmd.linear.x = min(plant_input,0.5)
                    move_cmd.angular.z = min(plant_a_input,1.57)

                    self.r.sleep()


    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        plt.plot(list(range(len(self.errs))),
                    self.errs, label='errs')
        plt.axhline(y=0,color='R')
        plt.draw()
        plt.show()

        rospy.sleep(1)
if __name__ == '__main__':
    try:
        path_number = 0
        path_follower = PathFollower(path_number)
        path_follower.go()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
