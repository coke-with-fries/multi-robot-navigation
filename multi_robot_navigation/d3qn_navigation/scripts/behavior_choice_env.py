#!/usr/bin/env python
# coding = utf-8
import rospy
import math
from def_msgs.msg import behavior
from listeners import robot1_scan_listener, robot1_odom_listener, robot1_pose_listener, x_target, y_target, robot2_scan_listener, robot3_scan_listener, robot2_ed_listener, robot3_ed_listener
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist



class BehaviorChoiceEnv:
    def __init__(self, lambdad = 50.0, d_thres = 0.4, lambdao = 50, r_obstacle = 30):
        self.action_space = [0, 1]
        self.behavior_pub = rospy.Publisher("/behavior", behavior, queue_size=10)

        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)

        self.lambdad = lambdad
        self.d_thres = d_thres
        self.lambdao = lambdao
        self.r_obstacle = r_obstacle

        self.distance_last_time = math.sqrt(x_target**2 + y_target**2)

        self.initial_positions = {
                'robot1': (2.0, 0.0, 0.0, 1.0),
	            'robot2': (1.0, 0.0, 0.0, 1.0),
	            'robot3': (0.0, 0.0, 0.0, 1.0)
	        }

        self.already_reset = False
        
    def step(self, action):
        behavior_cmd = behavior()
        behavior_cmd.time = rospy.Time.now().to_sec()
        behavior_cmd.behavior = action
        self.behavior_pub.publish(behavior_cmd)

        state, scan_data, robot2_scan_data, robot3_scan_data, distance = self.get_state()
        reward = self.reward(distance, scan_data, robot2_scan_data, robot3_scan_data, action)

        d_t1 = min(scan_data)
        d_t2 = min(robot2_scan_data)
        d_t3 = min(robot3_scan_data)

        if d_t1 < 0.13 or d_t2 < 0.13 or d_t3 < 0.13 or distance < 0.13:
            done = True
        else:
            done = False

        target = True if distance < 0.13 else False

        return reward, state, done, target

    def reward(self, distance, scan_data, robot2_scan_data, robot3_scan_data, action):
        rd_t = self.lambdad * math.exp(-distance / 5)
        
        d_t1 = min(scan_data)
        d_t2 = min(robot2_scan_data)
        d_t3 = min(robot3_scan_data)

        ro_t1 = -self.lambdao * math.exp(-(d_t1 / self.d_thres)**2) if d_t1 < self.d_thres else 0
        ro_t2 = -self.lambdao * math.exp(-(d_t2 / self.d_thres)**2) if d_t2 < self.d_thres else 0
        ro_t3 = -self.lambdao * math.exp(-(d_t3 / self.d_thres)**2) if d_t3 < self.d_thres else 0
        ro_t = ro_t1 + ro_t2 + ro_t3

        rs_t1 = -100 if d_t1 < 0.13 else 0  
        rs_t2 = -100 if d_t2 < 0.13 else 0  
        rs_t3 = -100 if d_t3 < 0.13 else 0
        rs_t = rs_t1 + rs_t2 + rs_t3

        rk_t1 = -50 if d_t1 >= 0.8 and action == 1 else 50

        rk_t = rk_t1

        rh_t = rd_t + ro_t + rs_t + rk_t

        return rh_t

    def get_state(self):
        scan_data = robot1_scan_listener.get_front_scan()
        robot2_scan_data = robot2_scan_listener.get_front_scan()
        robot3_scan_data = robot3_scan_listener.get_front_scan()
        v = robot1_odom_listener.get_v()
        omega = robot1_odom_listener.get_omega()
        _, x, y, _, _ = robot1_pose_listener.get_data()
        distance = math.sqrt((x_target - x)**2 + (y_target - y)**2)
        state = scan_data + [v] + [omega] + [distance]

        return state, scan_data, robot2_scan_data, robot3_scan_data, distance

    def reset(self, control):
        try:
            self.pause()
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            # Reset each robot's position in Gazebo
            for model_name, pose in self.initial_positions.items():
                
                state = ModelState()
                state.model_name = model_name
                state.reference_frame = "world"

                state.pose.position.x = pose[0]
                state.pose.position.y = pose[1]
                state.pose.position.z = pose[2]

                state.pose.orientation.w = 1.0

                state.twist.linear.x = 0.0
                state.twist.linear.y = 0.0
                state.twist.linear.z = 0.0
                state.twist.angular.x = 0.0
                state.twist.angular.y = 0.0
                state.twist.angular.z = 0.0

                # Update AMCL initial pose for the robot
                if model_name.startswith('robot'):
                    amcl_pose_topic = '/{}/initialpose'.format(model_name)
                    amcl_pose_pub = rospy.Publisher(amcl_pose_topic, PoseWithCovarianceStamped, queue_size=10)
                    
                    amcl_pose = PoseWithCovarianceStamped()
                    amcl_pose.header.frame_id = "map"
                    amcl_pose.header.stamp = rospy.Time.now()

                    amcl_pose.pose.pose.position.x = pose[0]
                    amcl_pose.pose.pose.position.y = pose[1]
                    amcl_pose.pose.pose.position.z = pose[2]
                    amcl_pose.pose.pose.orientation.w = 1.0

                    amcl_pose_pub.publish(amcl_pose)

                try:
	                rospy.wait_for_service('/gazebo/set_model_state', timeout=5)
	                set_state(state)
                except rospy.ServiceException as e:
	                rospy.logerr("State setting failed for {}: {}".format(model_name, str(e)))
	                continue


            self.unpause()

            reset_state = self.judge_respan(control)
            
            return reset_state

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(str(e)))
            return False

    def judge_respan(self, control):
        time_begin = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - time_begin < 10:
            control(0)
            robot2_ed = robot2_ed_listener.get_data()
            robot3_ed = robot3_ed_listener.get_data()
            robot1_v = robot1_odom_listener.get_v()
            if robot2_ed > 1.5 or robot3_ed > 1.5:
                return False
            if rospy.Time.now().to_sec() - time_begin > 9 and robot1_v < 0.17:
                return False
        return True


