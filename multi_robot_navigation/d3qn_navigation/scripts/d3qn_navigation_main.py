#!/usr/bin/env python
# coding = utf-8
# use 'rosrun d3qn_navigation d3qn_navigation_main.py' to start
import rospy
import math
from def_msgs.msg import follower_yaw
from geometry_msgs.msg import Twist
from listeners import robot1_odom_listener, robot2_odom_listener, robot3_odom_listener, robot1_scan_listener, robot1_pose_listener, x_target, y_target, robot2_scan_listener, robot3_scan_listener, robot2_ed_listener, robot3_ed_listener
from behavior_choice_env import BehaviorChoiceEnv
from behavior_choice_d3qn import BehaviorChoiceD3qn
from navigation_control_d3qn import NavigationControlD3qn
from navigation_control_env import NavigationControlEnv
from buffer import ReplayBuffer

x_target = x_target
y_target = y_target

Kp = 1
Ki = 0.1
Kd = 0.3

integral = 0.0
prev_error = 0.0

robot1_vel_pub = None
robot2_vel_pub = None
robot3_vel_pub = None

def aim_drive_model():
    global x_target, y_target, prev_error, integral, Kp, Ki, Kd
    prev_time, x_robot, y_robot, _, yaw_robot = robot1_pose_listener.get_data()

    dx = x_target - x_robot
    dy = y_target - y_robot
    distance = math.hypot(dx, dy)
    if distance < 0.1:
        return 0.0
    
    yaw_target = math.atan2(dy, dx)
    yaw_error = yaw_target - yaw_robot
    yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

    dt = rospy.Time.now().to_sec() - prev_time
    integral += yaw_error *  dt
    derivate = yaw_error - prev_error
    prev_error = yaw_error

    omega = Kp * yaw_error + Ki * integral + Kd * derivate

    return omega

def navigation_init():
    rospy.init_node('d3qn_navigation_main', anonymous=True)
    global robot1_vel_pub, robot2_yaw_pub, robot3_yaw_pub

    robot1_odom_listener.start()
    robot2_odom_listener.start()
    robot3_odom_listener.start()

    robot1_scan_listener.start()
    robot2_scan_listener.start()
    robot3_scan_listener.start()

    robot1_pose_listener.start()

    robot2_ed_listener.start()
    robot3_ed_listener.start()

    robot1_vel_pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=10)
    robot2_yaw_pub = rospy.Publisher("/robot2/aim_yaw", follower_yaw, queue_size=10)
    robot3_yaw_pub = rospy.Publisher("/robot3/aim_yaw", follower_yaw, queue_size=10)

def avoid_control(n_action):
    angular_velocity_map = {
        0 : -1.0,
        1 : -0.7,
        2 : -0.3,
        3 : 0,
        4 : 0.3,
        5 : 0.7,
        6 : 1.0
    }
    return angular_velocity_map[n_action]

    

def robot_control(b_action, n_action = 3):
    global robot1_vel_pub, robot2_yaw_pub, robot3_yaw_pub

    if b_action == 0:
        omega = aim_drive_model()
        robot2_aim_yaw = 180
        robot3_aim_yaw = 180
    elif b_action == 1:
        omega = avoid_control(n_action)
        robot2_aim_yaw = 180
        robot3_aim_yaw = 180

    robot1_vel_cmd = Twist()
    robot2_yaw_cmd = follower_yaw()
    robot3_yaw_cmd = follower_yaw()

    robot1_vel_cmd.linear.x = 0.2
    robot1_vel_cmd.angular.z = omega
    robot2_yaw_cmd.robot_name = "robot2"
    robot2_yaw_cmd.time = rospy.Time.now().to_sec()
    robot2_yaw_cmd.aim_yaw = robot2_aim_yaw
    robot3_yaw_cmd.robot_name = "robot3"
    robot3_yaw_cmd.time = rospy.Time.now().to_sec()
    robot3_yaw_cmd.aim_yaw = robot3_aim_yaw

    robot1_vel_pub.publish(robot1_vel_cmd)
    robot2_yaw_pub.publish(robot2_yaw_cmd)
    robot3_yaw_pub.publish(robot3_yaw_cmd)

def navigation_main():
    navigation_init()

    behavior_choice_env = BehaviorChoiceEnv()
    navigation_control_env = NavigationControlEnv()

    b_alpha = 0.0001
    b_state_dim = 123
    b_action_dim = 2
    max_episodes = 100000
    max_steps = 4000
    b_fc1_dim = 512
    b_fc2_dim = 256
    b_checkpoint_dir = './behavior/'

    n_alpha = 0.0001
    n_state_dim = 360
    n_action_dim = 7
    n_fc1_dim = 512
    n_fc2_dim = 256
    n_checkpoint_dir = './navigation/'

    target_step_interval = 1
    obstacle_step_interval = 5

    b_agent = BehaviorChoiceD3qn(alpha=b_alpha, state_dim=b_state_dim, action_dim=b_action_dim, fc1_dim=b_fc1_dim, fc2_dim=b_fc2_dim, ckpt_dir=b_checkpoint_dir)
    n_agent = NavigationControlD3qn(alpha=n_alpha, state_dim=n_state_dim, action_dim=n_action_dim, fc1_dim=n_fc1_dim, fc2_dim=n_fc2_dim, ckpt_dir=n_checkpoint_dir)

    rate = rospy.Rate(10)
    rospy.sleep(3)

    while not rospy.is_shutdown():
        for episode in range(max_episodes):
            b_initial_state, _, _, _, _ = behavior_choice_env.get_state()
            n_initial_state = navigation_control_env.get_state()
            reset_state = behavior_choice_env.reset(robot_control)
            if reset_state == False:
                break
            b_state = b_initial_state
            n_state = n_initial_state
            b_action = b_agent.choose_action(b_state)
            episode_b_reward = 0
            episode_n_reward = 0
            b_step_count = 0
            n_step_count = 0
            for step in range(max_steps):
                if step % (target_step_interval if b_action == 0 else obstacle_step_interval) == 0:
                    b_action = b_agent.choose_action(b_state)
                n_action = n_agent.choose_action(n_state)
                robot_control(b_action, n_action)
                rospy.sleep(0.05)
                b_reward, b_next_state, done, target = behavior_choice_env.step(b_action)
                b_agent.remember(b_state, b_action, b_reward, b_next_state, done)
                b_state = b_next_state
                b_agent.learn()
                episode_b_reward += b_reward
                b_step_count += 1

                rospy.logwarn("b_reward:{}".format(b_reward))

                if b_action == 1:
                    n_reward, n_next_state = navigation_control_env.step(n_action)
                    n_agent.remember(n_state, n_action, n_reward, n_next_state, done)
                    n_state = n_next_state
                    n_agent.learn()
                    episode_n_reward += n_reward
                    n_step_count += 1
                    rospy.logwarn("n_reward:{}".format(n_reward))
                else:
                    rospy.logwarn("aim_draiving")

                
                
                if done:
                    break

            if b_step_count > 0:
                b_average_reward = episode_b_reward / b_step_count

            if n_step_count > 0:
                n_average_reward = episode_n_reward / n_step_count

            if b_step_count > 0 and n_step_count > 0:
                rospy.logwarn("b_average_reward:{} n_average_reward:{}, steps:b{}n{}".format(b_average_reward, n_average_reward, b_step_count, n_step_count))
                

        

if __name__ == '__main__':
    navigation_main()
