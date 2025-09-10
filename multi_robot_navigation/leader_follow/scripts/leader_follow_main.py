#!/usr/bin/env python
# coding=utf-8
import rospy
import math
import time
from def_msgs.msg import pose
from def_msgs.msg import follower_yaw
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


robot2_l_wish = 0.5
robot3_l_wish = 1.0
robot2_ang_wish = math.radians(-180)
robot3_ang_wish = math.radians(180)
d = 0.08

robot2_vel_pub = None
robot3_vel_pub = None
robot2_e_d_pub = None
robot2_e_theta_pub = None
robot3_e_d_pub = None
robot3_e_theta_pub = None

prev_positions = {}

class RobotPosition(object):
    def __init__(self):
        self.time = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.omega = 0.0
    
class RobotsPositions(object):
    def __init__(self):
        self.robot1 = RobotPosition()
        self.robot2 = RobotPosition()
        self.robot3 = RobotPosition()
    
robots_positions = RobotsPositions()

def angle_diff(a, b):
    diff = (a - b + math.pi) % (2 * math.pi) - math.pi
    return diff if diff != -math.pi else math.pi

def update_velocity(robot, msg):
    global prev_positions
    current_time = rospy.Time.now().to_sec()
    
    if msg.robot_name not in prev_positions:
        prev_positions[msg.robot_name] = {'time': current_time, 'x': msg.x, 'y': msg.y, 'yaw': msg.yaw}
    
    dt = current_time - prev_positions[msg.robot_name]['time']

    alpha_velocity = 1.0
    alpha_omega = 1.0
    if dt > 0.001:
        dx = msg.x - prev_positions[msg.robot_name]['x']
        dy = msg.y - prev_positions[msg.robot_name]['y']
        robot_v = math.hypot(dx, dy) / dt
        robot.v = alpha_velocity * robot_v + (1 - alpha_velocity) * robot.v
    
        dyaw = angle_diff(msg.yaw, prev_positions[msg.robot_name]['yaw'])
        robot_omega = dyaw / dt
        robot.omega = alpha_omega * robot_omega + (1 - alpha_omega) * robot.omega
    
    prev_positions[msg.robot_name] = {'time': current_time, 'x': msg.x, 'y': msg.y, 'yaw': msg.yaw}

def controller(x_wish, y_wish, x_actual, y_actual, follower, leader, desired_angle):
    e_x = x_wish - x_actual
    e_y = y_wish - y_actual
    e_d = math.hypot(e_x, e_y)
    
    relative_yaw = angle_diff(follower.yaw, leader.yaw)
    if e_d <= 0.05:
        e_theta = relative_yaw  
    else:
        e_theta = angle_diff(desired_angle, relative_yaw)
    
    current_time = rospy.Time.now().to_sec()
    
    K_v = 0.4
    K_omega = 0.8
    k = 0.4
    
    v_vF = K_v * e_d
    v_F = leader.v * math.cos(e_theta) + v_vF
    
    denominator = math.sqrt(1 + e_x**2 + e_y**2)
    gamma_term = K_omega * e_theta
    omega_F = leader.omega + (k * leader.v * e_y) / denominator + gamma_term
    
    v_F = max(min(v_F, 2.0), -2.0)
    omega_F = max(min(omega_F, 2.0), -2.0)

    return v_F, omega_F, e_d, e_theta

def calculate_desired_position(robot_length_wish, robot_angle_wish, follow_robot, leader_robot):
    global d
    x_wish = robot_length_wish * math.cos(robot_angle_wish)
    y_wish = robot_length_wish * math.sin(robot_angle_wish)

    follow_x = follow_robot.x - d * math.cos(follow_robot.yaw)
    follow_y = follow_robot.y - d * math.sin(follow_robot.yaw)
    dx = follow_x - leader_robot.x
    dy = follow_y - leader_robot.y
    theta = leader_robot.yaw

    x_actual = dx * math.cos(theta) + dy * math.sin(theta)
    y_actual = -dx * math.sin(theta) + dy * math.cos(theta)

    ex = x_wish - x_actual
    ey = y_wish - y_actual
    desired_angle = math.atan2(ey, ex)


    return x_wish, y_wish, x_actual, y_actual, desired_angle

def leader_follow():
    global robot2_vel_pub, robot3_vel_pub
    global robot2_e_d_pub, robot2_e_theta_pub, robot3_e_d_pub, robot3_e_theta_pub
    global robot2_ang_wish
    global robot3_ang_wish

    robot2_x_wish, robot2_y_wish, robot2_x_actually, robot2_y_actually, robot2_desired_angle = calculate_desired_position(robot2_l_wish, robot2_ang_wish,robots_positions.robot2,robots_positions.robot1)
    v_F, omega_F, robot2_e_d, robot2_e_theta = controller(robot2_x_wish, robot2_y_wish, robot2_x_actually, robot2_y_actually, robots_positions.robot2, robots_positions.robot1, robot2_desired_angle)
    robot2_vel_cmd = Twist()
    robot2_vel_cmd.linear.x = v_F
    robot2_vel_cmd.angular.z = omega_F

    robot2_e_d_pub.publish(robot2_e_d)
    robot2_e_theta_pub.publish(robot2_e_theta)

    robot3_x_wish, robot3_y_wish, robot3_x_actually, robot3_y_actually, robot3_desired_angle = calculate_desired_position(robot3_l_wish,robot3_ang_wish,robots_positions.robot3,robots_positions.robot1)
    v_F, omega_F, robot3_e_d, robot3_e_theta = controller(robot3_x_wish, robot3_y_wish, robot3_x_actually, robot3_y_actually, robots_positions.robot3, robots_positions.robot1, robot3_desired_angle)
    robot3_vel_cmd = Twist()
    robot3_vel_cmd.linear.x = v_F
    robot3_vel_cmd.angular.z = omega_F

    robot3_e_d_pub.publish(robot3_e_d)
    robot3_e_theta_pub.publish(robot3_e_theta)

    robot2_vel_pub.publish(robot2_vel_cmd)
    robot3_vel_pub.publish(robot3_vel_cmd)

    # warning_message = "Desired position for robot 3: ({}, {}), Actual position: ({}, {})".format(robot3_x_wish, robot3_y_wish, robot3_x_actually, robot3_y_actually)
    # rospy.logwarn(warning_message)

def select_robot_msg(msg):
    robot = getattr(robots_positions, msg.robot_name, None)
    if robot is not None:
        robot.time = msg.time
        robot.x = msg.x
        robot.y = msg.y
        robot.z = msg.z
        robot.yaw = msg.yaw

        update_velocity(robot, msg)
    else:
        rospy.logwarn("Unknown robot: %s", msg.robot_name)

def update_robot2_aim_yaw(msg):
    global robot2_ang_wish
    robot2_ang_wish = math.radians(msg.aim_yaw)

def update_robot3_aim_yaw(msg):
    global robot3_ang_wish
    robot3_ang_wish = math.radians(msg.aim_yaw)

def leader_follow_init():
    global robot2_vel_pub, robot3_vel_pub
    global robot2_e_d_pub, robot2_e_theta_pub, robot3_e_d_pub, robot3_e_theta_pub

    rospy.init_node("leader_follow_main", anonymous=True)
    rospy.Subscriber("/robot_poses", pose, select_robot_msg)
    rospy.Subscriber("/robot2/aim_yaw", follower_yaw, update_robot2_aim_yaw)
    rospy.Subscriber("/robot3/aim_yaw", follower_yaw, update_robot3_aim_yaw)

    robot2_vel_pub = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=10)
    robot3_vel_pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=10)

    robot2_e_d_pub = rospy.Publisher('/robot2/e_d', Float32, queue_size=10)
    robot2_e_theta_pub = rospy.Publisher('/robot2/e_theta', Float32, queue_size=10)
    robot3_e_d_pub = rospy.Publisher('/robot3/e_d', Float32, queue_size=10)
    robot3_e_theta_pub = rospy.Publisher('/robot3/e_theta', Float32, queue_size=10)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        leader_follow()
        rate.sleep()
    
if __name__ == '__main__':
    leader_follow_init()