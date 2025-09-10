#!/usr/bin/env python
# coding = utf-8
import rospy
import threading
import numpy as np
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from def_msgs.msg import pose
from def_msgs.msg import follower_yaw
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


x_target = 30.0
y_target = 0.0

class odometry_data:
    def __init__(self):
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.v = 0.0
        self.omega = 0.0
        self.updated = False

class odometry_listener():
    def __init__(self, robot_name, topic):
        self.robot_name = robot_name
        self.topic = topic
        self.data = odometry_data()
        self.subscriber = None
    
    def start(self):
        def call_back(msg):
            with self.data.condition:
                self.data.v = msg.twist.twist.linear.x
                self.data.omega = msg.twist.twist.angular.z
                self.data.updated = True
                self.data.condition.notify_all()

        self.subscriber = rospy.Subscriber(self.topic, Odometry, call_back)
        rospy.logwarn("L:{}".format(self.topic))

    def get_v(self):
        with self.data.condition:
            while not self.data.updated:
                self.data.condition.wait()
        v = self.data.v

        return v

    def get_omega(self):
        with self.data.condition:
            while not self.data.updated:
                self.data.condition.wait()
        omega = self.data.omega
        
        return omega

class scan_data:
    def __init__(self):
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.front_scan = []
        self.updated = False

class scan_listener():
    def __init__(self, robot_name, topic):
        self.robot_name = robot_name
        self.topic = topic
        self.data = scan_data()
        self.subscriber = None
    
    def start(self):
        def call_back(msg):
            with self.data.condition:
                angle_min = msg.angle_min
                angle_inc = msg.angle_increment
                ranges = msg.ranges
                range_max = msg.range_max

                front_scan = []
                for i in range(len(ranges)):
                    current_angle = angle_min + i * angle_inc
                    current_angle = (current_angle + math.pi) % (2 * math.pi) - math.pi

                    if -math.pi / 3 <= current_angle <= math.pi / 3:
                        distance = ranges[i]
        
                        if math.isinf(distance):
                            distance = range_max
                        elif distance < 0:
                            continue
                
                        front_scan.append(distance)
                
                self.data.front_scan = front_scan
                self.data.updated = True
                self.data.condition.notify_all()

        self.subscriber = rospy.Subscriber(self.topic, LaserScan, call_back)
        rospy.logwarn("L:{}".format(self.topic))

    def get_front_scan(self):
        with self.data.condition:
            while not self.data.updated:
                self.data.condition.wait()
        front_scan = self.data.front_scan
        return front_scan

class pose_data:
    def __init__(self):
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.time = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.updated = False

class pose_listener():
    def __init__(self, robot_name, topic):
        self.robot_name = robot_name
        self.topic = topic
        self.data = pose_data()
        self.subscriber = None
    
    def start(self):
        def call_back(msg):
            with self.data.condition:
                if msg.robot_name is not None:
                    if msg.robot_name == self.robot_name:
                        self.data.time = msg.time
                        self.data.x = msg.x
                        self.data.y = msg.y
                        self.data.z = msg.z
                        self.data.yaw = msg.yaw
                        self.data.updated = True
                        self.data.condition.notify_all()

        self.subscriber = rospy.Subscriber(self.topic, pose, call_back)
        rospy.logwarn("L:{}".format(self.topic))

    def get_data(self):
        with self.data.condition:
            while not self.data.updated:
                self.data.condition.wait()
        time = self.data.time
        x = self.data.x
        y = self.data.y
        z = self.data.z
        yaw = self.data.yaw

        return time, x, y, z, yaw

class ed_data:
    def __init__(self):
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.ed = 0.0
        self.updated = False

class ed_listener():
    def __init__(self, robot_name, topic):
        self.robot_name = robot_name
        self.topic = topic
        self.data = ed_data()
        self.subscriber = None
    
    def start(self):
        def call_back(msg):
            with self.data.condition:
                self.data.ed = msg.data
                self.data.updated = True
                self.data.condition.notify_all()

        self.subscriber = rospy.Subscriber(self.topic, Float32, call_back)
        rospy.logwarn("L:{}".format(self.topic))

    def get_data(self):
        with self.data.condition:
            while not self.data.updated:
                self.data.condition.wait()
        ed = self.data.ed

        return ed



robot1_odom_listener = odometry_listener('robot1', '/robot1/odom')
robot2_odom_listener = odometry_listener('robot2', '/robot2/odom')
robot3_odom_listener = odometry_listener('robot3', '/robot3/odom')

robot1_scan_listener = scan_listener('robot1', '/robot1/scan')
robot2_scan_listener = scan_listener('robot2', '/robot2/scan')
robot3_scan_listener = scan_listener('robot3', '/robot3/scan')

robot1_pose_listener = pose_listener('robot1', '/robot_poses')

robot2_ed_listener = ed_listener('robot2', '/robot2/e_d')
robot3_ed_listener = ed_listener('robot3', '/robot3/e_d')
