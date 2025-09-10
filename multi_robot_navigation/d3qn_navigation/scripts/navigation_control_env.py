#!/usr/bin/env python
# coding = utf-8
import rospy
import math
from listeners import robot1_scan_listener, robot1_pose_listener, x_target, y_target, robot1_scan_listener, robot1_odom_listener, robot2_scan_listener, robot3_scan_listener

class NavigationControlEnv:
    def __init__(self, lamdac = 2):
        self.lamdac = lamdac
        self.omega_deadzone = 0.1
    
    def step(self, n_action):
        next_state = self.get_state()
        reward = self.get_reward()

        return reward, next_state
        
    def get_reward(self):
        _, x, y, _, yaw = robot1_pose_listener.get_data()
        omega = robot1_odom_listener.get_omega()
        scan_data_robot1 = robot1_scan_listener.get_front_scan()

        scan_data_robot2 = robot2_scan_listener.get_front_scan()
        scan_data_robot3 = robot3_scan_listener.get_front_scan()

        distance = math.hypot(x_target - x, y_target - y)
        rd_t = 10 * (28 - distance)

        d_min_robot1 = min(scan_data_robot1)
        rc_t1 = -100 if d_min_robot1 <= 0.13 else -(self.lamdac ** 2) / d_min_robot1

        d_min_robot2 = min(scan_data_robot2)
        rc_t2 = -100 if d_min_robot2 <= 0.13 else -(self.lamdac ** 2) / d_min_robot2

        d_min_robot3 = min(scan_data_robot3)
        rc_t3 = -100 if d_min_robot3 <= 0.13 else -(self.lamdac ** 2) / d_min_robot3

        rc_t = 0.5 * (rc_t1 + rc_t2 + rc_t3)

        rw_t = -0.4 * max(abs(omega) - self.omega_deadzone, 0)

        target_angle = math.atan2(y_target - y, x_target - x)
        theta_error = math.atan2(math.sin(target_angle - yaw), math.cos(target_angle - yaw))
        abs_error = abs(theta_error)
        rt_t = -0.6 * (abs_error / math.pi)

        if abs_error > math.pi / 2:
            rt_t += -1.5 * (abs_error - math.pi / 2)

        return rd_t + rc_t + rw_t + rt_t


    def get_state(self):
        state = robot1_scan_listener.get_front_scan()
        robot2_state = robot2_scan_listener.get_front_scan()
        robot3_state = robot3_scan_listener.get_front_scan()
        state = state + robot2_state + robot3_state
        return state
