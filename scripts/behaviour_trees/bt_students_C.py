#!/usr/bin/env python3


import re

from matplotlib.pyplot import cla
import py_trees as pt, py_trees_ros as ptr, rospy
# from behaviours_student import *
from reactive_sequence import RSequence

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from sensor_msgs.msg import JointState

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes
import tf2_ros
import tf2_geometry_msgs
import sys
from dataclasses import dataclass

@dataclass
class CheckState:
    node_name: str = "Student BT"
    arm_tucked: bool = False
    head_state: str = "unknown"
    detected: bool = False
    picked: bool = False
    placed: bool = False
    cube_on_table: bool = False

# 初始化状态
CHECK = CheckState()

class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):

        rospy.loginfo("Initialising behaviour tree")

        # Tuck arm sequence
        tuck_arm_sequence = pt.composites.Sequence(
            name="Arm Tuck Sequence",
            children=[move("backward", 10), TuckArm(), move("forward", 10)]
        )
        check_tuck_selector = pt.composites.Selector(
            name="Check if Arm is Tucked",
            children=[check_tuck(), tuck_arm_sequence]
        )

        # Move head down
        move_head_down = move_head("down")

        # Detect cube for picking
        detect_cube = DetectForPick()

        # Pick cube
        pick_cube_action = pick_cube()

        # Move to second table
        move_to_second_table = move_to("second_table")

        # Place cube sequence
        place_cube_action = PlaceCube()
        place_cube_sequence = pt.composites.Sequence(
            name="Place Cube Sequence",
            children=[place_cube_action, move("clockwise", 30), TuckArm(), move("counter_clockwise", 30)]
        )

        # Check if cube is on table
        check_if_cube_on_table = check_cube_on_table()
        check_cube_selector = pt.composites.Selector(
            name="Check if Cube is on Table",
            children=[check_if_cube_on_table, move_to("first_table")]
        )

        # Reset Check State
        reset_check_state = reset_CHECK()

        # Main behaviour tree sequence
        main_sequence = RSequence(
            name="Main Task Sequence",
            children=[
                check_tuck_selector, 
                move_head_down, 
                detect_cube, 
                pick_cube_action, 
                move_to_second_table, 
                place_cube_sequence, 
                check_cube_selector, 
                reset_check_state
            ]
        )

        super(BehaviourTree, self).__init__(main_sequence)

        # Execute the behaviour tree
        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown():
            self.tick_tock(1)


class check_tuck(pt.behaviour.Behaviour):
	def __init__(self):
		super(check_tuck, self).__init__("Check arm condifiton")
	def update(self):
		if CHECK.arm_tucked:
			return pt.common.Status.SUCCESS
		else:
			return pt.common.Status.FAILURE


class reset_CHECK(pt.behaviour.Behaviour):
	def __init__(self):
		super(reset_CHECK, self).__init__("Reset CHECK")
	def update(self):
		global CHECK
		# success if done
		CHECK.detected = False
		CHECK.picked = False
		CHECK.cube_on_table = False
		CHECK.placed = False

		rospy.loginfo("%s: Initial state initialized.", )
		return pt.common.Status.SUCCESS

	
class DetectForPick(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("%s: Initializing detect cube...", CHECK.node_name)
        
        # tf2 设置
        self.robot_base_frame = rospy.get_param('~robot_base_frame')  # 获取机器人基座坐标系参数
        self.tf_buffer = tf2_ros.Buffer()  # 创建一个缓冲区来存储 TF 数据
        self.listener = tf2_ros.TransformListener(self.tf_buffer)  # 创建一个监听器，将 TF 数据填充到缓冲区中
        
        # 初始化发布者
        aruco_pose_topic = rospy.get_param('~aruco_pose_topic')  # 获取 ArUco 位姿话题参数
        self.aruco_pose_pub = rospy.Publisher(aruco_pose_topic, PoseStamped, queue_size=10)  # 创建一个发布 ArUco 位姿的发布者
        
        # 成为一个行为
        super(DetectForPick, self).__init__("Detect for pick!")  # 初始化行为
    
    def update(self):
        global CHECK
        try:
            # 获取传感器坐标系中的位姿
            detected_pose = rospy.wait_for_message('/robotics_intro/aruco_single/pose', PoseStamped, timeout=1)  # 等待 ArUco 位姿消息
            
            # 转换到机器人基座坐标系并发布 aruco_pose_topic
            trans = self.tf_buffer.lookup_transform(self.robot_base_frame, 'xtion_rgb_optical_frame', rospy.Time(0), rospy.Duration(1))  # 查找从传感器坐标系到机器人基座坐标系的变换
            transformed_pose = tf2_geometry_msgs.do_transform_pose(detected_pose, trans)  # 将检测到的位姿转换到机器人基座坐标系中
            
            rospy.loginfo("%s: Detect cube succeeded!", CHECK.node_name)  # 记录成功日志
            self.aruco_pose_pub.publish(transformed_pose)  # 发布转换后的位姿
            CHECK.detected = True  # 更新 CHECK 状态
            return pt.common.Status.SUCCESS  # 返回成功状态
        except (rospy.ROSException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("%s: Detect cube failed: %s", CHECK.node_name, str(e))  # 记录失败日志
            CHECK.detected = False  # 更新 CHECK 状态
            return pt.common.Status.FAILURE  # 返回失败状态
		
class check_cube_on_table(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("%s: Initializing check place...", CHECK.node_name)

		super(check_cube_on_table, self).__init__("Check place action")

	def update(self):
		global CHECK
		if CHECK.cube_on_table:
			rospy.loginfo("%s: Cube already place on table!", CHECK.node_name)
			return pt.common.Status.SUCCESS
		else:
			try:
				rospy.wait_for_message('/robotics_intro/aruco_single/pose', PoseStamped, timeout=1)
				rospy.loginfo("%s: Check cube on table succeeded! End of task!", CHECK.node_name)
				sys.exit(0)
				return pt.common.Status.SUCCESS
			except rospy.ROSException as e:
				rospy.loginfo("%s: Check cube on table failed!", CHECK.node_name)
				# print("Error: %s"%e)
				return pt.common.Status.FAILURE

class pick_cube(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("%s: Initializing pick cube...", CHECK.node_name)
		# Server
		pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		self.pick_srv = rospy.ServiceProxy(pick_srv_nm, SetBool)
		rospy.wait_for_service(pick_srv_nm, timeout=30)
		# become a behaviour
		super(pick_cube, self).__init__("Pick cube!")

	def update(self):
		global CHECK
		# success if done
		if CHECK.picked:
			rospy.loginfo("%s: Cube already picked!", CHECK.node_name)
			return pt.common.Status.SUCCESS
		else:
			self.pick_srv_req = self.pick_srv()
			# if succesful
			if self.pick_srv_req.success:
				CHECK.picked = True
				CHECK.arm_tucked= False
				rospy.loginfo("%s: Pick cube succeded!", CHECK.node_name)
				return pt.common.Status.SUCCESS
			# if failed
			elif not self.pick_srv_req.success:
				CHECK.arm_tucked= False
				rospy.loginfo("%s: Pick cube failed!", CHECK.node_name)
				return pt.common.Status.FAILURE
			# if still trying
			else:
				return pt.common.Status.RUNNING

class PlaceCube(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("%s: Initializing place cube...", CHECK.node_name)
        
        # Server setup
        place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.place_srv = rospy.ServiceProxy(place_srv_nm, SetBool)
        rospy.wait_for_service(place_srv_nm, timeout=30)

        # Initialize the behaviour
        super(PlaceCube, self).__init__("Place cube!")

    def update(self):
        """更新放置方块的状态"""
        if self.is_cube_placed():
            rospy.loginfo("%s: Cube already placed!", CHECK.node_name)
            return pt.common.Status.SUCCESS
        else:
            return self.attempt_to_place_cube()

    def is_cube_placed(self):
        """检查方块是否已被放置"""
        return CHECK.placed

    def attempt_to_place_cube(self):
        """尝试放置方块并返回状态"""
        try:
            response = self.place_srv()
            if response.success:
                CHECK.placed = True
                CHECK.arm_tucked = False
                rospy.loginfo("%s: Place cube finished!", CHECK.node_name)
                return pt.common.Status.SUCCESS
            else:
                CHECK.arm_tucked = False
                rospy.logwarn("%s: Place cube failed!", CHECK.node_name)
                return pt.common.Status.FAILURE
        except rospy.ServiceException as e:
            rospy.logerr("%s: Service call failed: %s", CHECK.node_name, str(e))
            return pt.common.Status.FAILURE


class move(pt.behaviour.Behaviour):
    def __init__(self, direction, steps):
        self.rate = rospy.Rate(10)  # 设置发布频率为 10Hz
        self.move_msg = Twist()  # 创建一个 Twist 消息来控制机器人的运动
        self.direction = direction  # 记录运动方向
        self.steps = steps  # 记录步数
        
        # 根据方向设置运动消息
        if self.direction == "forward":
            self.move_msg.linear.x = 0.2  # 前进
        elif self.direction == "backward":
            self.move_msg.linear.x = -0.5  # 后退
        elif self.direction == "clockwise":
            self.move_msg.angular.z = -1.0  # 顺时针旋转
        elif self.direction == "counter_clockwise":
            self.move_msg.angular.z = 1.0  # 逆时针旋转
        
        # 初始化发布者
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic')  # 获取速度控制话题参数
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)  # 创建一个发布速度控制的发布者
        
        # 成为一个行为
        super(move, self).__init__("Move!")  # 初始化行为
    
    def update(self):
        rospy.loginfo("%s: Moving %s...", CHECK.node_name, self.direction)  # 记录运动方向
        count = 0
        while not rospy.is_shutdown() and count < self.steps:
            self.cmd_vel_pub.publish(self.move_msg)  # 发布运动消息
            self.rate.sleep()  # 根据设定的频率休眠
            count += 1  # 计数加一
        
        # 完成后停止机器人
        self.move_msg = Twist()  # 重置运动消息
        self.cmd_vel_pub.publish(self.move_msg)  # 发布停止消息
        rospy.sleep(1)  # 等待 1 秒
        rospy.loginfo("%s: Done", CHECK.node_name)  # 记录完成日志
        return pt.common.Status.SUCCESS  # 返回成功状态

class move_to(pt.behaviour.Behaviour):
    """
    A behavior to move the robot towards a specified goal (e.g., first_table, second_table).
    
    Attributes:
    - goal: The destination of the robot (either 'first_table' or 'second_table').
    """
    
    def __init__(self, goal):
        super(move_to, self).__init__("Move to goal!")
        rospy.loginfo(f"{CHECK.node_name}: Initializing move to {goal}...")

        self.goal = goal
        self.direction = 1 if goal == "second_table" else -1  # 根据目标设置方向
        self.rate = rospy.Rate(10)  # 设定频率为 10Hz
        self.move_msg = Twist()  # 创建一个 Twist 消息来控制机器人的运动
        
        # 初始化发布者，用于发布速度控制
        cmd_vel_topic = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    def _rotate(self, duration, angular_speed):
        """根据给定的时间和角速度旋转机器人。"""
        self.move_msg.angular.z = angular_speed
        self._publish_for_duration(duration)

    def _move_forward(self, duration, linear_speed):
        """根据给定的时间和线速度前进机器人。"""
        self.move_msg.linear.x = linear_speed
        self._publish_for_duration(duration)

    def _publish_for_duration(self, duration):
        """在指定的时间内发布运动消息。"""
        count = 0
        while not rospy.is_shutdown() and count < duration:
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
            count += 1
    
    def update(self):
        """行为的主要更新方法，控制机器人朝向目标移动。"""
        rospy.loginfo(f"{CHECK.node_name}: Moving towards {self.goal}...")

        # 根据方向旋转机器人（持续 33 个周期，角速度基于方向）
        self._rotate(duration=33, angular_speed=-1 * self.direction)
        
        # 停止旋转
        self.move_msg.angular.z = 0
        self.cmd_vel_pub.publish(self.move_msg)
        rospy.sleep(1)  # 暂停 1 秒
        
        # 向目标前进（持续 20 个周期，线速度为 0.5）
        self._move_forward(duration=20, linear_speed=0.5)
        
        # 停止机器人
        self.move_msg.linear.x = 0
        self.move_msg.angular.z = 0
        self.cmd_vel_pub.publish(self.move_msg)
        
        rospy.loginfo(f"{CHECK.node_name}: Reached {self.goal}")
        return pt.common.Status.SUCCESS  # 返回成功状态



class TuckArm(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("Initialising tuck arm behaviour.")
		# Set up action client
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
		# personal goal setting
		self.goal = PlayMotionGoal()
		self.goal.motion_name = 'home'
		self.goal.skip_planning = True
		# become a behaviour
		super(TuckArm, self).__init__("Tuck arm!")

	def update(self):
		global CHECK
		# already tucked the arm
		if CHECK.arm_tucked:
			rospy.loginfo("%s: Arm already tucked", CHECK.node_name)
			return pt.common.Status.SUCCESS
		else:
			# send the goal
			self.play_motion_ac.send_goal(self.goal)
			# if I was succesful! :)))))))))
			while not self.play_motion_ac.get_result():
				pass
			# then I'm finished!
			CHECK.arm_tucked = True
			# rospy.sleep(10) # wait for arm to tuck in gazebo
			rospy.loginfo("%s: Tuck arm succeeded", CHECK.node_name)
			return pt.common.Status.SUCCESS


class move_head(pt.behaviour.Behaviour):
    """
    控制机器人的头部按照指定方向（'up' 或 'down'）移动的行为。
    
    参数:
    - direction: 头部移动的方向（'up' 或 'down'）。
    """

    def __init__(self, direction):
        super(move_head, self).__init__("Move head!")
        rospy.loginfo("初始化头部移动行为。")

        # 获取用于移动头部的服务名称参数
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        
        # 创建一个服务代理，用于调用头部移动服务
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        
        # 等待服务可用，超时时间为 30 秒
        try:
            rospy.wait_for_service(mv_head_srv_nm, timeout=30)
        except rospy.ROSException as e:
            rospy.logerr(f"服务 {mv_head_srv_nm} 不可用: {str(e)}")
        
        # 存储要移动头部的方向
        self.direction = direction

    def update(self):
        """
        头部移动的主要更新方法。此方法检查头部是否已经处于目标位置，如果没有，则发送移动命令。
        """
        global CHECK

        # 如果头部已经处于期望的位置
        if CHECK.head_state == self.direction:
            rospy.loginfo(f"{CHECK.node_name}: 头部已经在 {CHECK.head_state} 位置")
            return pt.common.Status.SUCCESS
        
        # 发送头部移动请求
        try:
            self.move_head_req = self.move_head_srv(self.direction)
            
            # 如果头部移动成功
            if self.move_head_req.success:
                rospy.sleep(3)  # 模拟在 Gazebo 中等待头部移动完成
                CHECK.head_state = self.direction
                rospy.loginfo(f"头部 {self.direction} 移动成功")
                return pt.common.Status.SUCCESS
            
            # 如果头部移动失败
            else:
                rospy.logwarn(f"头部 {self.direction} 移动失败")
                return pt.common.Status.FAILURE

        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {str(e)}")
            return pt.common.Status.FAILURE


if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()