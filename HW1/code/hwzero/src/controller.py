#!/usr/bin/python3
import rospy
from hwzero.msg import proximity
from hwzero.msg import order
import random
import time
Data = None
Data_Old = None
def callback(data):
    global Data
    Data = data

def controller_node():
    rospy.init_node('controller', anonymous=False)
    engine1_pub = rospy.Publisher('engine1_order', order, queue_size=10)
    engine2_pub = rospy.Publisher('engine2_order', order, queue_size=10)
    rospy.Subscriber('distance', proximity, callback)
    return engine1_pub, engine2_pub

def start2():
    engine1_pub, engine2_pub = controller_node()
    while not rospy.is_shutdown():
        if Data:
            start_time = time.time()
            engine_msg1,engine_msg2 = create_order_msg(Data)
            engine_msg1.velocity = 0
            engine_msg2.velocity = 0
            engine1_pub.publish(engine_msg1)
            engine2_pub.publish(engine_msg2)
            rospy.loginfo("first data recevied")
            Data_Old = Data
            Data_List = [Data.left,Data.up,Data.down,Data.right]
            Data_old_list = [Data_Old.left,Data_Old.up,Data_Old.down,Data_Old.right]
            min_number = min(Data_old_list)
            min_index = Data_old_list.index(min_number)
            min_element = Data_List[min_index]
            while not rospy.is_shutdown():
                Data_List = [Data.left,Data.up,Data.down,Data.right]
                Data_old_list = [Data_Old.left,Data_Old.up,Data_Old.down,Data_Old.right]
                min_number = min(Data_old_list)
                min_index = Data_old_list.index(min_number)
                min_element = Data_List[min_index]                
                if max(Data_List) == 140 and max(Data_old_list)==140:
                    rospy.signal_shutdown("Shutting down")
                    exit()
                rospy.loginfo('min of old distance : %s' , min_number )
                rospy.loginfo('now we are in  : %s' , min_element )
                if min_element - min_number > 10 or min_element - min_number < -10 :
                    engine_msg1,engine_msg2 = create_order_msg(Data)
                    elapsed_time = time.time() - start_time
                    engine_msg1.velocity = abs(int((min_element - min_number)/elapsed_time))
                    engine_msg2.velocity = engine_msg1.velocity
                    start_time = time.time()
                    rospy.loginfo("we should rotate")
                    engine1_pub.publish(engine_msg1)
                    engine2_pub.publish(engine_msg2)
                    Data_Old = Data
                else:
                    engine_msg1,engine_msg2 = create_zero_messaage()
                    rospy.loginfo("do not need rotate")
                    elapsed_time = time.time() - start_time
                    engine_msg1.velocity = abs(int((min_element - min_number)/elapsed_time))
                    engine_msg2.velocity = engine_msg1.velocity
                    start_time = time.time()
                    engine1_pub.publish(engine_msg1)
                    engine2_pub.publish(engine_msg2)
                rate = rospy.Rate(.75)
                rate.sleep()

def create_order_msg(Data):
    Data2 = [Data.left,Data.up,Data.down,Data.right]
    def find_way(Data):
        if max(Data2) == Data.left:
            maxIndex = "left"
        if max(Data2) == Data.right:
            maxIndex = "right"
        if max(Data2) == Data.up:
            maxIndex = "up"
        if max(Data2) == Data.down:
            maxIndex = "down"


        if min(Data2) == Data.left:
            minIndex = "left"
        if min(Data2) == Data.right:
            minIndex = "right"
        if min(Data2) == Data.up:
            minIndex = "up"
        if min(Data2) == Data.down:
            minIndex = "down"


        if maxIndex == "left":
            if minIndex == "right":
                way = 2
                sgn = 1
            if minIndex == "down":
                way = 1
                sgn = 1
            if minIndex == "up":
                way = 1
                sgn = -1
        if maxIndex == "right":
            if minIndex == "left":
                way = 2
                sgn = 1
            if minIndex == "down":
                way = 1
                sgn = -1
            if minIndex == "up":
                way = 1
                sgn = 1
        if maxIndex == "down":
            if minIndex == "right":
                way = 1
                sgn = 1
            if minIndex == "left":
                way = 1
                sgn = -1
            if minIndex == "up":
                way = 2
                sgn = 1
        if maxIndex == "up":
            if minIndex == "right":
                way = 1
                sgn = -1
            if minIndex == "down":
                way = 2
                sgn = 1
            if minIndex == "left":
                way = 1
                sgn = 1
        return way,sgn
    way , sgn = find_way(Data)
    def UpLeft_LeftDown_DownRight_RightUp(sgn):
        engine_msg1 = order()
        engine_msg2 = order()
        if sgn == 1:
            rospy.loginfo("robot should rotate 90 degree ccw")
            engine_msg1.rotate_robot_degree = 90
            engine_msg1.rotate_wheel = "negative"
            engine_msg1.velocity = None
            engine_msg1.rotate_robot_direction = "ccw"
            engine_msg2.rotate_robot_degree = 90
            engine_msg2.rotate_wheel = "positive"
            engine_msg2.velocity = None
            engine_msg2.rotate_robot_direction = "ccw"
        if sgn == -1:
            rospy.loginfo("robot should rotate 90 degree cw")
            engine_msg1.rotate_robot_degree = 90
            engine_msg1.rotate_wheel = "positive"
            engine_msg1.velocity = None
            engine_msg1.rotate_robot_direction = "cw"
            engine_msg2.rotate_robot_degree = 90
            engine_msg2.rotate_wheel = "negative"
            engine_msg2.velocity = None
            engine_msg2.rotate_robot_direction = "cw"
        return engine_msg1,engine_msg2
    def UpDown_LeftRight(sgn):
        sgn = random.choice([1, -1])
        engine_msg1 = order()
        engine_msg2 = order()
        rospy.loginfo("robot should rotate 180 degree cw or ccw")
        if sgn == 1:
            engine_msg1.rotate_robot_degree = 180
            engine_msg1.rotate_wheel = "positive"
            engine_msg1.velocity = None
            engine_msg1.rotate_robot_direction = "cw"
            engine_msg2.rotate_robot_degree = 180
            engine_msg2.rotate_wheel = "negative"
            engine_msg2.velocity = None
            engine_msg2.rotate_robot_direction = "cw"
        if sgn == -1:
            engine_msg1.rotate_robot_degree = 180
            engine_msg1.rotate_wheel = "negative"
            engine_msg1.velocity = None
            engine_msg1.rotate_robot_direction = "ccw"
            engine_msg2.rotate_robot_degree = 180
            engine_msg2.rotate_wheel = "positive"
            engine_msg2.velocity = None
            engine_msg2.rotate_robot_direction = "ccw"

        return engine_msg1,engine_msg2
    if way == 1:
        engine_msg1,engine_msg2 = UpLeft_LeftDown_DownRight_RightUp(sgn)
    if way == 2:
        engine_msg1,engine_msg2 = UpDown_LeftRight(sgn)
    return engine_msg1,engine_msg2
def create_zero_messaage():
    engine_msg1 = order()
    engine_msg2 = order()
    engine_msg1.rotate_robot_degree = 0
    engine_msg1.rotate_wheel = "positive"
    engine_msg1.velocity = None
    engine_msg1.rotate_robot_direction = "None"
    engine_msg2.rotate_robot_degree = 0
    engine_msg2.rotate_wheel = "positive"
    engine_msg2.velocity = None
    engine_msg2.rotate_robot_direction = "None"
    return engine_msg1,engine_msg2
   

if __name__ == '__main__':
    start2()
