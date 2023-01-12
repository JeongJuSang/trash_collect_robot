#! /usr/bin/env python3
import rospy
import actionlib
import tf
import math
import numpy as np
import pyrealsense2 as rs2
from sensor_msgs.msg import Image, CameraInfo
from array import array
from math import cos, sin, acos
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float64, Int64
from nav_msgs.msg import Odometry


trash_detected = False
x = 0
y = 0
x_center = 0
y_center = 0
angle_degrees = 0
angle = 0
depth = 0
intrinsics = None
robot_pose = [0, 0, 0]
robot_orientation = [0, 0, 0, 1]
goal_coordination = [0, 0]
base = 0
height = 570


def x_sub(data):
    global x_center
    x_center = data.data
    # print(x_center)
    return x_center

def y_sub(data):
    global y_center
    y_center = data.data
    # print(y_center)
    
    return y_center

def dep_sub(data):
    global depth
    depth = data.data
    
    return depth

def camera_info(cameraInfo):
    global intrinsics
    # import pdb; pdb.set_trace()
    if intrinsics:
        return
    intrinsics = rs2.intrinsics()
    intrinsics.width = cameraInfo.width
    intrinsics.height = cameraInfo.height
    intrinsics.ppx = cameraInfo.K[2]
    intrinsics.ppy = cameraInfo.K[5]
    intrinsics.fx = cameraInfo.K[0]
    intrinsics.fy = cameraInfo.K[4]
    if cameraInfo.distortion_model == 'plumb_bob':
        intrinsics.model = rs2.distortion.brown_conrady
    elif cameraInfo.distortion_model == 'equidistant':
        intrinsics.model = rs2.distortion.kannala_brandt4
    intrinsics.coeffs = [i for i in cameraInfo.D]

    return intrinsics

def get_camera_position(intrinsics):
    # Extract the intrinsic parameters of the camera

    fx = intrinsics.fx
    fy = intrinsics.fy
    ppx = intrinsics.ppx
    ppy = intrinsics.ppy

    # Calculate the position of the camera in 3D space
    camera_pos = np.array([ppx, ppy])

    return camera_pos


def Calculate_angle(x, y, z):
    global angle
    global angle_degrees

    object_pos = np.array([x, intrinsics.ppy])
    camera_pos = get_camera_position(intrinsics)
    inner_CO = np.dot(camera_pos, object_pos)
    CO = np.linalg.norm(camera_pos) * np.linalg.norm(object_pos)
    angle = float(np.arccos(inner_CO/CO))
    angle_degrees = np.rad2deg(angle)
    
    return angle, base, angle_degrees

def bounding_boxes_callback(x ,y):
    
    global trash_detected

    if x == 0 and y == 0 :
        trash_detected = False
    else :
        trash_detected = True

    return trash_detected

def odom_callback(msg):

    global robot_pose, robot_orientation
    global robot_orientation

    robot_pose[0] = msg.pose.pose.position.x
    robot_pose[1] = msg.pose.pose.position.y
    robot_pose[2] = 0
    robot_orientation[0] = msg.pose.pose.orientation.x
    robot_orientation[1] = msg.pose.pose.orientation.y
    robot_orientation[2] = msg.pose.pose.orientation.z
    robot_orientation[3] = msg.pose.pose.orientation.w
    
    # base = math.sqrt(depth**2 - height**2) # 밑변의 길이
    # goal_x = odom_x + 0.001*base*cos(angle)
    # goal_y = odom_y + 0.001*base*sin(angle)

    return robot_pose[0], robot_pose[1]

def Calculate_goal(x, y):
    global depth
    global angle

    global height
    global goal_coordination
    global base

    # base = math.sqrt(depth**2 - height**2) 
    base = (depth**2 - height**2)**0.5
    goal_coordination[0] = robot_pose[0] + 0.001*base*cos(angle)
    goal_coordination[1] = -1*(robot_pose[1] + 0.001*base*cos(angle))
    
    return goal_coordination, base

def collect_trash(x, y):

    global goal_coordination

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = goal_coordination[0]
    goal.target_pose.pose.position.y = goal_coordination[1]
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    client.wait_for_result()


if __name__ == '__main__':
    rospy.init_node('trash_collector')
    twist = Twist()

    # bounding_boxes_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, bounding_boxes_callback)
    depth_sub = rospy.Subscriber('/depth', Int64, dep_sub)
    x_center_sub = rospy.Subscriber('x_center_value', Int64, x_sub)
    y_center_sub = rospy.Subscriber('y_center_value', Int64, y_sub)
    sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, camera_info)
    # odom_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, odom_callback)
    odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, odom_callback)
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    rate = rospy.Rate(10)
    # move_pub= rospy.Publisher('move_base/goal', MoveBaseGoal, queue_size=10)
    while not rospy.is_shutdown():
        bounding_boxes_callback(x_center, y_center)
        if trash_detected == True:
            Calculate_angle(x_center, y_center, depth)
            print(angle_degrees)
            # print(angle)
            Calculate_goal(robot_pose[0], robot_pose[1])
            # print(camera_pos)
            collect_trash(goal_coordination[0], goal_coordination[1])
            # print(goal_coordination[0], goal_coordination[1])
            # trash_detected = False
        else :
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            print('no trash')
        rate.sleep()

    rospy.spin()