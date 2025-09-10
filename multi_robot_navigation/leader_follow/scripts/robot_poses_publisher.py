#!/usr/bin/env python
# coding = utf-8

import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from def_msgs.msg import pose

pose_pub = None

# Dictionary to store the last known positions and orientations for each robot
robot_poses = {}

def extract_pose_info(robot_name, msg):
    """
    Extracts position and orientation from the received pose message and updates 
    the last known pose for the specified robot.

    Args:
        robot_name (str): The name of the robot whose pose is being updated.
        msg (PoseWithCovarianceStamped): The incoming message containing the pose data.
    """
    global robot_poses
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    # Convert quaternion orientation to yaw angle using atan2
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # Update the last known pose for the robot in the dictionary
    robot_poses[robot_name] = [position.x, position.y, position.z, yaw]

def publish_robot_pose(robot_name):
    """
    Publishes the current pose of a specified robot.

    Args:
        robot_name (str): The name of the robot whose pose is being published.
    """
    global pose_pub, robot_poses

    # Retrieve the latest position and yaw for the specified robot
    x, y, z, yaw= robot_poses[robot_name]
    
    # Create a new message for publishing
    pose_msg = pose()
    
    # Populate message fields with relevant data
    pose_msg.robot_name = robot_name
    pose_msg.time = rospy.Time.now().to_sec()
    pose_msg.x = x
    pose_msg.y = y
    pose_msg.z = z
    pose_msg.yaw = yaw

    # Publish the message to /robot_poses topic
    pose_pub.publish(pose_msg)

def get_robot1_pose_callback(msg):
    """Callback function for receiving Robot1's poses."""
    extract_pose_info("robot1", msg)

def get_robot2_pose_callback(msg):
    """Callback function for receiving Robot2's poses."""
    extract_pose_info("robot2", msg)

def get_robot3_pose_callback(msg):
    """Callback function for receiving Robot3's poses."""
    extract_pose_info("robot3", msg)

def get_lf_pose():
    """
    Initializes ROS node and sets up subscribers and publishers for 
    managing robots' poses.
    Continuously publishes their poses at a set rate until shutdown.
    """
    global pose_pub, robot_poses

    rospy.init_node("robot_poses_publisher", anonymous=True)  # Initialize ROS node

    robot_poses = {
        "robot1": [0.0, 0.0, 0.0, 0.0],  #time, x, y, z, yaw, vel_x, omega_z (orientation)
        "robot2": [0.0, 0.0, 0.0, 0.0],
        "robot3": [0.0, 0.0, 0.0, 0.0],
    }
    
    # Create a publisher that will publish messages on /robot_poses topic
    pose_pub = rospy.Publisher("/robot_poses", pose, queue_size=10)

    # Subscribe to each robot's AMCL Pose topic to receive their positions
    rospy.Subscriber("/robot1/amcl_pose", PoseWithCovarianceStamped, get_robot1_pose_callback)
    
    rospy.Subscriber("/robot2/amcl_pose", PoseWithCovarianceStamped, get_robot2_pose_callback)
    
    rospy.Subscriber("/robot3/amcl_pose", PoseWithCovarianceStamped, get_robot3_pose_callback)

    rate = rospy.Rate(100)  # Set loop rate to 10 Hz
    
    while not rospy.is_shutdown():  # Main loop runs until ROS is shut down
            for robot in ["robot1", "robot2", "robot3"]:
                publish_robot_pose(robot)  # Publish each robot's current pose
            rate.sleep()                 # Sleep to maintain loop rate

if __name__ == '__main__':
    get_lf_pose()                 # Execute main function when script is run directly
