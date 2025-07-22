#!/usr/bin/env python3

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    def __init__(self):
        
        self.node_name = "StudentStateMachine"
        # Access rosparams (获取ROS参数)
        self.cmd_vel_topic = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.move_head_service_name = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.cube_position_param = rospy.get_param(rospy.get_name() + "/cube_pose")
        # 将字符串转换为浮点数列表
        cube_position_list = self.cube_position_param.split(",")
        cube_position_floats = []
        for x in cube_position_list:
            float_value = float(x)
            cube_position_floats.append(float_value)
        self.cube_position = cube_position_floats
        self.aruco_pose_topic = rospy.get_param(rospy.get_name() + "/aruco_pose_topic")
        self.pick_service_name = rospy.get_param(rospy.get_name() + "/pick_srv")
        self.place_service_name = rospy.get_param(rospy.get_name() + "/place_srv")
        # 等待服务可用
        rospy.wait_for_service(self.move_head_service_name, timeout=30)
        rospy.wait_for_service(self.pick_service_name, timeout=30)
        rospy.wait_for_service(self.place_service_name, timeout=30)
        # 初始化发布者
        self.velocity_publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.cube_pose_publisher = rospy.Publisher(self.aruco_pose_topic, PoseStamped, queue_size=10)

        # 设置动作客户端
        self.play_motion_client = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_client.wait_for_server(rospy.Duration(1000)):
            exit()
        # 初始化状态机
        self.state = 0
        rospy.sleep(3)
        self.execute_states()

    def execute_states(self):

        while not rospy.is_shutdown() and self.state != 4:
            # 状态 0：收回机械臂
            if self.state == 0:

                # 创建一个PlayMotionGoal对象，用于定义动作目标
                goal = PlayMotionGoal()

                # 设置目标动作名称为'home'，表示将机械臂移动到home位置
                goal.motion_name = 'home'

                # 跳过路径规划，直接执行动作
                goal.skip_planning = True

                # 发送目标给play_motion客户端，执行机械臂移动动作
                self.play_motion_client.send_goal(goal)

                # 等待结果，超时时间设置为100秒，检查机械臂是否成功完成动作
                arm_tuck_success = self.play_motion_client.wait_for_result(rospy.Duration(100.0))

                # 如果机械臂成功移动到目标位置
                if arm_tuck_success:
                    # 更新状态为1，表示成功
                    self.state = 1
                else:
                    # 如果机械臂未成功到达目标，取消目标动作
                    self.play_motion_client.cancel_goal()

                    # 更新状态为-1，表示失败
                    self.state = -1

                # 等待1秒，给系统留出时间稳定
                rospy.sleep(1)

            # 状态 1：抓取立方体
            if self.state == 1:
                velocity_command = Twist()
                velocity_command.linear.x = 0.125
                velocity_command.angular.z = 0
                self.velocity_publisher.publish(velocity_command)              
                # 创建位置信息消息
                # cube_pose_msg = PoseStamped()
                # cube_pose_msg.pose.position.x = self.cube_position[0]
                # cube_pose_msg.pose.position.y = self.cube_position[1]
                # cube_pose_msg.pose.position.z = self.cube_position[2]
                # cube_pose_msg.pose.orientation.x = self.cube_position[3]
                # cube_pose_msg.pose.orientation.y = self.cube_position[4]
                # cube_pose_msg.pose.orientation.z = self.cube_position[5]
                # cube_pose_msg.pose.orientation.w = self.cube_position[6]


                cube_pose_msg = PoseStamped()
                # 分离位置和方向的索引
                position = self.cube_position[:3]
                orientation = self.cube_position[3:]
                # 将位置赋值
                cube_pose_msg.pose.position.x, cube_pose_msg.pose.position.y, cube_pose_msg.pose.position.z = position
                # 将四元数方向赋值
                cube_pose_msg.pose.orientation.x, cube_pose_msg.pose.orientation.y, cube_pose_msg.pose.orientation.z, cube_pose_msg.pose.orientation.w = orientation
                cube_pose_msg.header.frame_id = 'base_footprint'
                # 发布立方体位置信息
                self.cube_pose_publisher.publish(cube_pose_msg)
                # 调用抓取服务
                pick_service = rospy.ServiceProxy(self.pick_service_name, SetBool)
                pick_request = pick_service(True)
                # 检查抓取请求是否成功
                if pick_request.success:               
                    self.state = 2
                else:                 
                    self.state = -1
                rospy.sleep(3)
            # 状态 2：移动机器人到另一个桌子
            if self.state == 2:
                # 创建一个Twist消息对象，用于设置线速度和角速度
                velocity_command = Twist()

                # 设置机器人逆时针旋转（角速度为负值，表示向左旋转）
                velocity_command.angular.z = -1

                # 设置发布速率为10Hz（每秒发布10次）
                rate = rospy.Rate(10)

                # 初始化计数器，控制循环执行的次数
                count = 0

                # 在ROS节点没有关闭的情况下，并且循环次数少于30时，执行以下操作
                while not rospy.is_shutdown() and count < 30:
                    # 发布旋转的速度指令
                    self.velocity_publisher.publish(velocity_command)
                    
                    # 以10Hz的频率睡眠，这样整个循环会持续约3秒钟（30次循环，每秒10次）
                    rate.sleep()
                    
                    # 增加计数器
                    count += 1

                # 等待1秒，给机器人一点时间稳定
                rospy.sleep(1)

                # 设置机器人前进，线速度为1（向前移动），角速度为0（直线运动）
                velocity_command.linear.x = 1
                velocity_command.angular.z = 0

                # 重置计数器
                count = 0

                # 在ROS节点没有关闭的情况下，并且循环次数少于11时，执行以下操作
                while not rospy.is_shutdown() and count < 11:
                    # 发布前进的速度指令
                    self.velocity_publisher.publish(velocity_command)
                    
                    # 以10Hz的频率睡眠，这样整个循环会持续约1.1秒钟（11次循环，每秒10次）
                    rate.sleep()
                    
                    # 增加计数器
                    count += 1

                # 停止机器人，发布一个空的Twist消息（所有速度设为0）
                self.velocity_publisher.publish(Twist())  # 停止移动

                # 更新状态为3
                self.state = 3

                # 再次等待1秒，确保机器人完全停止
                rospy.sleep(1)
                    
            # 状态 3：将立方体放置在另一个桌子上
            if self.state == 3:                
                # 调用放置服务
                # 创建一个与服务交互的代理对象，指定服务名称和服务类型
                # 这里使用 `self.place_service_name` 服务名称，并使用 `SetBool` 服务类型
                place_service = rospy.ServiceProxy(self.place_service_name, SetBool)

                # 发送服务请求，调用 place 服务
                place_request = place_service()

                # 检查服务请求是否成功返回
                if place_request.success:
                    # 如果请求成功，跳出循环（服务成功执行）
                    break
                else:
                    # 如果请求失败，设置状态为-1，表示执行失败或错误
                    self.state = -1

                # 等待2秒，给系统足够时间处理（例如稳定服务状态或避免过快请求）
                rospy.sleep(2)

            # 错误处理
            if self.state == -1:               
                return        
        return


# import py_trees as pt, py_trees_ros as ptr

# class BehaviourTree(ptr.trees.BehaviourTree):

# 	def __init__(self):

# 		rospy.loginfo("Initialising behaviour tree")

# 		# go to door until at door
# 		b0 = pt.composites.Selector(
# 			name="Go to door fallback", 
# 			children=[Counter(30, "At door?"), Go("Go to door!", 1, 0)]
# 		)

# 		# tuck the arm
# 		b1 = TuckArm()

# 		# go to table
# 		b2 = pt.composites.Selector(
# 			name="Go to table fallback",
# 			children=[Counter(5, "At table?"), Go("Go to table!", 0, -1)]
# 		)

# 		# move to chair
# 		b3 = pt.composites.Selector(
# 			name="Go to chair fallback",
# 			children=[Counter(13, "At chair?"), Go("Go to chair!", 1, 0)]
# 		)

# 		# lower head
# 		b4 = LowerHead()

# 		# become the tree
# 		tree = pt.composites.Sequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
# 		super(BehaviourTree, self).__init__(tree)

# 		# execute the behaviour tree
# 		self.setup(timeout=10000)
# 		while not rospy.is_shutdown(): self.tick_tock(1)


# class Counter(pt.behaviour.Behaviour):

# 	def __init__(self, n, name):

# 		# counter
# 		self.i = 0
# 		self.n = n

# 		# become a behaviour
# 		super(Counter, self).__init__(name)

# 	def update(self):

# 		# count until n
# 		while self.i <= self.n:

# 			# increment count
# 			self.i += 1

# 			# return failure :(
# 			return pt.common.Status.FAILURE

# 		# succeed after counter done :)
# 		return pt.common.Status.SUCCESS


# class Go(pt.behaviour.Behaviour):

# 	def __init__(self, name, linear, angular):

# 		# action space
# 		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
# 		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

# 		# command
# 		self.move_msg = Twist()
# 		self.move_msg.linear.x = linear
# 		self.move_msg.angular.z = angular

# 		# become a behaviour
# 		super(Go, self).__init__(name)

# 	def update(self):

# 		# send the message
# 		rate = rospy.Rate(10)
# 		self.cmd_vel_pub.publish(self.move_msg)
# 		rate.sleep()

# 		# tell the tree that you're running
# 		return pt.common.Status.RUNNING


# class TuckArm(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# Set up action client
# 		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

# 		# personal goal setting
# 		self.goal = PlayMotionGoal()
# 		self.goal.motion_name = 'home'
# 		self.goal.skip_planning = True

# 		# execution checker
# 		self.sent_goal = False
# 		self.finished = False

# 		# become a behaviour
# 		super(TuckArm, self).__init__("Tuck arm!")

# 	def update(self):

# 		# already tucked the arm
# 		if self.finished: 
# 			return pt.common.Status.SUCCESS
		
# 		# command to tuck arm if haven't already
# 		elif not self.sent_goal:

# 			# send the goal
# 			self.play_motion_ac.send_goal(self.goal)
# 			self.sent_goal = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# if I was succesful! :)))))))))
# 		elif self.play_motion_ac.get_result():

# 			# than I'm finished!
# 			self.finished = True
# 			return pt.common.Status.SUCCESS

# 		# if I'm still trying :|
# 		else:
# 			return pt.common.Status.RUNNING
		


# class LowerHead(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# server
# 		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
# 		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
# 		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

# 		# execution checker
# 		self.tried = False
# 		self.tucked = False

# 		# become a behaviour
# 		super(LowerHead, self).__init__("Lower head!")

# 	def update(self):

# 		# try to tuck head if haven't already
# 		if not self.tried:

# 			# command
# 			self.move_head_req = self.move_head_srv("down")
# 			self.tried = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# react to outcome
# 		else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE


	

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		#StateMachine()
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()