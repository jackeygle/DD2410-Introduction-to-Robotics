#!/usr/bin/env python3

import re

import py_trees as pt, py_trees_ros as ptr, rospy

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
import numpy as np
import math
from actionlib_msgs.msg import GoalStatus
import py_trees as pt, py_trees_ros as ptr
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
import py_trees as pt
# global variables
node_name = "Student BT"

class StateManager:
    def __init__(self):
        self._state = {
            "arm_tucked": False,
            "head_state": "unknown",
            "detected": False,
            "picked": False,
            "placed": False,
            "cube_on_table": False,
            "localized": False,
            "navigated_pick": False,
            "navigated_place": False,
            "kidnapped": False
        }

    def get_state(self, key):
        return self._state.get(key, None)

    def set_state(self, key, value):
        if key in self._state:
            self._state[key] = value
            return True
        return False
    def update_states(self, updates):
        for key, value in updates.items():
            self.set_state(key, value)


# 示例使用
manager = StateManager()
# 创建状态检查器实例
# check_nav_pick = StateChecker(
#     "navigated_pick",
#     "Check navigate to pick",
#     "navigated pick the cube!!!!!!!!!!!",
#     "not navigated pick!!!!!!!!!!!"
# )
#
# check_detect = StateChecker("detected", "Check detected cube")
# check_pick = StateChecker("picked", "Check pick cube")
# check_nav_place = StateChecker("navigated_place", "Check navigate to second table")
# check_place = StateChecker("placed", "Check place cube")

class BehaviourTree(ptr.trees.BehaviourTree):
    def __init__(self):
        rospy.loginfo("Initialising behaviour tree")
        tree = self.create_main_sequence()
        super(BehaviourTree, self).__init__(tree)

    def create_main_sequence(self):
        # 主任务序列
        main_sequence = RSequence(
            name="Main sequence",
            children=[
                # 检查是否被劫持并处理
                pt.composites.Selector(
                    name="Kidnapped Check",
                    children=[
                        # 如果没有被劫持，返回SUCCESS
                        StateCheck("kidnapped", False),
                        # 如果被劫持，执行恢复序列
                        self.create_recovery_sequence()
                    ]
                ),
                # 正常任务序列
                self.create_tuck_fallback(),
                self.create_head_fallback("up"),
                self.create_localization_without_kidnapped(),
                self.create_head_fallback("down"),
                self.create_detect_fallback(),
                self.create_pick_fallback(),
                self.create_localization_without_kidnapped(place=True),
                self.create_place_fallback(),
                self.create_check_cube_fallback()
            ]
        )
        return main_sequence

    def create_recovery_sequence(self):
        """创建恢复序列"""
        return pt.composites.Sequence(
            name="Recovery Steps",
            children=[
                # 重新定位序列
                move_head("up"),  # 抬头以便更好地定位
                localization(),  # 重新定位
                move_head("down"),  # 低头继续任务
                # 继续未完成的任务
                pt.composites.Selector(
                    name="Resume Task",
                    children=[
                        # 如果还没有检测到方块，继续去拾取位置
                        pt.composites.Sequence(
                            name="Resume Pick Navigation",
                            children=[
                                StateCheck("detected", False),
                                navigation("pick")
                            ]
                        ),
                        # 如果已经检测到方块，继续去放置位置
                        pt.composites.Sequence(
                            name="Resume Place Navigation",
                            children=[
                                StateCheck("picked", True),
                                navigation("place")
                            ]
                        )
                    ]
                )
            ]
        )

    def create_tuck_fallback(self):
        return pt.composites.Selector(
            name="Tuck arm?",
            children=[tuck_check, pt.composites.Sequence(
                name="Tuck sequence",
                children=[move("backward", 5), tuck_arm(), move("forward", 5)]
            )]
        )

    def create_head_fallback(self, direction):
        return pt.composites.Selector(
            name=f"move head {direction}?",
            children=[
                StateCheck("head_state", direction, f"Check head {direction}"),
                move_head(direction)
            ]
        )

    def create_localization_without_kidnapped(self, place=False):
        check_type = "place" if place else "pick"
        loc_and_nav = pt.composites.Sequence(
            name=f"loc and nav {check_type}",
            children=[
                self.create_localization_fallback(),
                self.create_navigation_fallback(check_type)
            ]
        )
        return pt.composites.Selector(
            name=f"localization without kidnapped {' place' if place else ''}",
            children=[
                CheckRobotState(check_type),
                loc_and_nav
            ]
        )

    def create_localization_fallback(self):
        return pt.composites.Selector(
            name="Localise?",
            children=[localization_check, localization()]
        )







    def create_check_cube_fallback(self):
        return pt.composites.Selector(
            name="Check tube?",
            children=[check_cube_on_table(), reset_check()]
        )

    def create_navigation_fallback(self, goal):
        return pt.composites.Selector(
            name=f"Move {goal}?",
            children=[
                StateChecker(f"navigated_{goal}",
                             f"Check navigate to {goal}",
                             f"navigated {goal}!!!!!!!!!!!",
                             f"not navigated {goal}!!!!!!!!!!!"
                             ),
                navigation(goal)
            ]
        )

    def create_detect_fallback(self):
        return pt.composites.Selector(
            name="Detect?",
            children=[
                StateChecker("detected", "Check detected cube"),
                detect_for_pick()
            ]
        )

    def create_pick_fallback(self):
        return pt.composites.Selector(
            name="Pick?",
            children=[
                StateChecker("picked", "Check pick cube"),
                pick_cube()
            ]
        )

    def create_place_fallback(self):
        place_cube_sequence = pt.composites.Sequence(
            name="Place cube sequence",
            children=[
                place_cube(),
                move("clockwise", 30),
                tuck_arm(),
                move("counter_clockwise", 30)
            ]
        )
        return pt.composites.Selector(
            name="Place?",
            children=[
                StateChecker("placed", "Check place cube"),
                place_cube_sequence,
                check_cube_on_table()
            ]
        )
    def run(self):
        self.setup(timeout=100000)
        rospy.sleep(5)
        rospy.loginfo("Starting Behaviour Tree")

        while not rospy.is_shutdown():
            self.tick_tock(100)
            rospy.sleep(1)
class StateCheck(pt.behaviour.Behaviour):
    def __init__(self, state_key, expected_value, name=None):
        # Dynamically set the name based on the state being checked if not provided
        name = name if name else f"Check {state_key}"
        super(StateCheck, self).__init__(name)
        self.state_key = state_key
        self.expected_value = expected_value

    def update(self):
        current_value = manager.get_state(self.state_key)
        if current_value == self.expected_value:
            rospy.loginfo(f"{self.state_key} is as expected: {self.expected_value}")
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo(f"{self.state_key} is not as expected: {self.expected_value}, current: {current_value}")
            return pt.common.Status.FAILURE


tuck_check = StateCheck("arm_tucked", True, "Check arm tuck")

localization_check = StateCheck("localized", True, "Check localization")


class localization(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("%s: Initializing localization...", node_name)
        # 基本初始化
        super(localization, self).__init__("Localization!")
        self.rate = rospy.Rate(10)
        self.start = False

        # 设置订阅器和服务
        self.setup_subscribers_and_services()

    def setup_subscribers_and_services(self):
        # AMCL pose 订阅
        self.amcl_pose = PoseWithCovarianceStamped()
        self.amcl_pose_sub = rospy.Subscriber(
            rospy.get_param(rospy.get_name() + '/amcl_estimate'),
            PoseWithCovarianceStamped,
            self.amcl_pose_callback
        )

        # 服务客户端设置
        self.global_loc_srv = rospy.ServiceProxy(
            rospy.get_param(rospy.get_name() + '/global_loc_srv'),
            Empty
        )
        self.clear_costmap_srv = rospy.ServiceProxy(
            rospy.get_param(rospy.get_name() + '/clear_costmaps_srv'),
            Empty
        )

        # CMD_VEL发布器
        self.cmd_vel_pub = rospy.Publisher(
            rospy.get_param(rospy.get_name() + '/cmd_vel_topic'),
            Twist,
            queue_size=10
        )

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg

    def start_localization(self):
        """开始定位过程"""
        self.global_loc_srv()
        self.start = True
        status_msg = 'Relocalizing !!!!!!' if manager.get_state("kidnapped") else 'localizing !!!!!!'
        rospy.loginfo(status_msg)
        return pt.common.Status.RUNNING

    def spin_and_check(self):
        """旋转机器人并检查定位状态"""
        # 发送旋转命令
        move_msg = Twist()
        move_msg.angular.z = 0.5
        self.cmd_vel_pub.publish(move_msg)
        self.rate.sleep()

        # 检查定位精度
        norm = np.linalg.norm(self.amcl_pose.pose.covariance)
        if norm < 0.02:
            return self.handle_localization_success(norm)
        else:
            rospy.loginfo("%s: amcl_pose not converged (norm = %f), keep spinning...", node_name, norm)
            return pt.common.Status.RUNNING

    def handle_localization_success(self, norm):
        """处理定位成功的情况"""
        rospy.loginfo("%s: amcl_pose converged (norm = %f), localized succeeded!", node_name, norm)
        self.clear_costmap_srv()
        self.start = False
        manager.set_state("localized", True)
        manager.set_state("kidnapped", False)
        rospy.loginfo("localized!!!!!!!!!!!")
        return pt.common.Status.SUCCESS

    def update(self):
        # 如果已经定位成功，直接返回成功
        if manager.get_state("localized"):
            return pt.common.Status.SUCCESS

        # 如果还没开始定位，启动定位过程
        if not self.start:
            return self.start_localization()

        # 正在定位中，继续旋转和检查
        return self.spin_and_check()



class CheckRobotState(pt.behaviour.Behaviour):
    def __init__(self, check_type="pick"):
        """
        初始化机器人状态检查
        :param check_type: 检查类型，"pick" 或 "place"
        """
        name = f"Check Robot State for {check_type}"
        super(CheckRobotState, self).__init__(name)
        self.check_type = check_type
        self.state_checks = {
            "kidnapped": (True, "Robot is kidnapped"),
            "localized": (False, "Robot is not localized"),
            f"navigated_{check_type}": (False, f"Robot is not navigated to {check_type}")
        }

    def update(self):
        # 检查每个状态
        for state, (failure_value, message) in self.state_checks.items():
            current_value = manager.get_state(state)
            if current_value == failure_value:
                rospy.loginfo(message)
                return pt.common.Status.FAILURE

        # 所有检查都通过
        rospy.loginfo(f"Robot is localized and navigated to {self.check_type} and not kidnapped.")
        return pt.common.Status.SUCCESS

class navigation(pt.behaviour.Behaviour):
    def __init__(self, goal):
        rospy.loginfo("%s: Initializing navigation...", node_name)
        self.rate = rospy.Rate(10)
        self.goal_str = goal
        self.goal = MoveBaseGoal()

        # 设置订阅器和发布器
        self.setup_subscribers_and_publishers()
        self.setup_move_base_client()
        super(navigation, self).__init__("Navigation")

    def setup_subscribers_and_publishers(self):
        # AMCL pose subscriber
        amcl_pose_top_nm = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        self.amcl_pose_sub = rospy.Subscriber(
            amcl_pose_top_nm,
            PoseWithCovarianceStamped,
            self.amcl_pose_callback
        )

        # Goal pose subscriber
        if self.goal_str == "pick":
            self.goal_pose_top_nm = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        elif self.goal_str == "place":
            self.goal_pose_top_nm = rospy.get_param(rospy.get_name() + '/place_pose_topic')

        if hasattr(self, 'goal_pose_top_nm'):
            self.goal_pose_sub = rospy.Subscriber(
                self.goal_pose_top_nm,
                PoseStamped,
                self.goal_callback
            )

        # CMD_VEL publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def setup_move_base_client(self):
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(10)):
            rospy.logerr("%s: Failed connecting to /move_base server", node_name)
            raise RuntimeError("Failed to connect to move_base server")

    def check_covariance_threshold(self, covariance_matrix):
        pos_cov = covariance_matrix[0] + covariance_matrix[7]
        return pos_cov > 0.07

    def amcl_pose_callback(self, amcl_pose_current):
        if self.goal_str in ["pick", "place"]:  # 只在导航去拾取或放置时检测劫持
            if self.check_covariance_threshold(amcl_pose_current.pose.covariance):
                rospy.loginfo("%s: Robot kidnapped during navigation!", node_name)
                manager.set_state("kidnapped", True)
                manager.set_state("localized", False)

                # 立即停止机器人
                self.stop_robot()

                # 取消导航目标
                self.move_base_ac.cancel_all_goals()

                # 更新导航状态，但保留任务进度信息
                if not manager.get_state("detected"):
                    manager.set_state("navigated_pick", False)
                elif manager.get_state("picked"):
                    manager.set_state("navigated_place", False)

    def stop_robot(self):
        zero_twist = Twist()
        self.cmd_vel_pub.publish(zero_twist)

    def goal_callback(self, msg):
        self.goal.target_pose = msg

    def execute_navigation(self, goal_type):
        # 检查是否被劫持
        if manager.get_state("kidnapped"):
            rospy.loginfo("%s: Navigation cancelled - robot is kidnapped", node_name)
            self.stop_robot()
            return pt.common.Status.FAILURE

        try:
            self.move_base_ac.send_goal(self.goal)

            # 等待导航结果时也要检查劫持状态
            while not self.move_base_ac.wait_for_result(rospy.Duration(0.5)):
                if manager.get_state("kidnapped"):
                    self.stop_robot()
                    self.move_base_ac.cancel_goal()
                    return pt.common.Status.FAILURE

            # 检查导航结果
            if self.move_base_ac.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"%s: Navigation to {goal_type} succeeded!", node_name)
                manager.set_state(f"navigated_{goal_type}", True)
                return pt.common.Status.SUCCESS
            else:
                return self.handle_navigation_failure(goal_type)

        except Exception as e:
            rospy.logerr(f"Navigation error: {str(e)}")
            return self.handle_navigation_failure(goal_type)

    def handle_navigation_failure(self, goal_type):
        self.move_base_ac.cancel_goal()
        rospy.loginfo(f"%s: Navigation to {goal_type} failed!", node_name)
        manager.set_state(f"navigated_{goal_type}", False)
        return pt.common.Status.FAILURE

    def update(self):
        if self.goal_str == "pick":
            if manager.get_state("navigated_pick"):
                return pt.common.Status.SUCCESS
            return self.execute_navigation("pick")
        elif self.goal_str == "place":
            if manager.get_state("navigated_place"):
                return pt.common.Status.SUCCESS
            return self.execute_navigation("place")
        else:
            rospy.logwarn("%s: Invalid navigation goal", node_name)
            return pt.common.Status.FAILURE

class StateChecker(pt.behaviour.Behaviour):
    """通用状态检查类"""
    def __init__(self, state_key, description=None, success_message=None, failure_message=None):
        """
        初始化状态检查器
        :param state_key: 要检查的状态键
        :param description: 检查描述（用于命名）
        :param success_message: 成功时的消息（可选）
        :param failure_message: 失败时的消息（可选）
        """
        name = description or f"Check {state_key}"
        super(StateChecker, self).__init__(name)
        self.state_key = state_key
        self.success_message = success_message
        self.failure_message = failure_message

    def update(self):
        state_value = manager.get_state(self.state_key)
        if state_value:
            if self.success_message:
                rospy.loginfo(self.success_message)
            return pt.common.Status.SUCCESS
        else:
            if self.failure_message:
                rospy.loginfo(self.failure_message)
            return pt.common.Status.FAILURE

class reset_check(pt.behaviour.Behaviour):
	def __init__(self):
		super(reset_check, self).__init__("Reset CHECK")
	def update(self):
 
		manager.update_states({
			"detected": False,
			"picked": False,
			"placed": False,
			"cube_on_table": False,
			"localized": True,
			"navigated_pick": False,
			"navigated_place": False
		})
		rospy.loginfo("%s: Initial state initialized.", node_name)
		return pt.common.Status.SUCCESS


class detect_for_pick(pt.behaviour.Behaviour):
    def __init__(self):
        super(detect_for_pick, self).__init__("Detect for pick!")
        rospy.loginfo("%s: Initializing detect cube...", node_name)
        self.setup_tf()
        self.setup_publisher()

    def setup_tf(self):
        """设置TF监听器"""
        self.robot_base_frame = rospy.get_param(rospy.get_name() + '/robot_base_frame')
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def setup_publisher(self):
        """设置发布器"""
        aruco_pose_topic = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.aruco_pose_pub = rospy.Publisher(
            aruco_pose_topic,
            PoseStamped,
            queue_size=10
        )

    def detect_and_transform_pose(self):
        """检测并转换姿态"""
        # 获取传感器框架中的姿态
        detected_pose = rospy.wait_for_message(
            '/robotics_intro/aruco_single/pose',
            PoseStamped,
            timeout=1
        )

        # 转换到机器人基座标系
        trans = self.tf_buffer.lookup_transform(
            self.robot_base_frame,
            'xtion_rgb_optical_frame',
            rospy.Time.now(),
            rospy.Duration(1)
        )
        return tf2_geometry_msgs.do_transform_pose(detected_pose, trans)

    def update(self):
        try:
            # 检测和转换姿态
            transformed_pose = self.detect_and_transform_pose()

            # 发布转换后的姿态
            self.aruco_pose_pub.publish(transformed_pose)

            # 更新状态并返回成功
            manager.set_state("detected", True)
            rospy.loginfo("%s: Detect cube succeeded!", node_name)
            return pt.common.Status.SUCCESS

        except rospy.ROSException as e:
            # 处理检测失败
            manager.set_state("detected", False)
            rospy.loginfo("%s: Detect cube failed! Error: %s", node_name, str(e))
            return pt.common.Status.FAILURE


class check_cube_on_table(pt.behaviour.Behaviour):
    def __init__(self):
        super(check_cube_on_table, self).__init__("Check place action")
        rospy.loginfo("%s: Initializing check place...", node_name)
        self.setup_model_state()
        self.setup_service()

    def setup_model_state(self):
        """设置模型状态"""
        self.model_state = ModelState()
        # 设置基本属性
        self.model_state.model_name = 'aruco_cube'
        self.model_state.reference_frame = 'map'

        # 设置位置
        self.model_state.pose.position.x, y, z = -1.130530, -6.653650, 0.86250
        self.model_state.pose.position.y = y
        self.model_state.pose.position.z = z

        # 设置方向（默认朝向）
        self.model_state.pose.orientation.w = 1
        self.model_state.twist = Twist()

    def setup_service(self):
        """设置服务客户端"""
        service_name = '/gazebo/set_model_state'
        rospy.wait_for_service(service_name)
        self.set_model_state_srv = rospy.ServiceProxy(service_name, SetModelState)

    def update(self):
        if manager.get_state("cube_on_table"):
            rospy.loginfo("%s: Cube already place on table!", node_name)
            return pt.common.Status.SUCCESS

        try:
            # 检查方块是否在桌子上
            rospy.wait_for_message(
                '/robotics_intro/aruco_single/pose',
                PoseStamped,
                timeout=1
            )
            rospy.loginfo("%s: Check cube on table succeeded! End of task!", node_name)
            sys.exit(0)
            return pt.common.Status.SUCCESS

        except rospy.ROSException:
            rospy.loginfo("%s: Check cube on table failed!", node_name)
            self.set_model_state_srv(self.model_state)
            return pt.common.Status.FAILURE


class CubeManipulator(pt.behaviour.Behaviour):
    """基类：用于抓取和放置方块的通用行为"""

    def __init__(self, action_type):
        super(CubeManipulator, self).__init__(f"{action_type.capitalize()} cube!")
        rospy.loginfo(f"%s: Initializing {action_type} cube...", node_name)

        # 设置服务客户端
        service_name = rospy.get_param(rospy.get_name() + f'/{action_type}_srv')
        self.service = rospy.ServiceProxy(service_name, SetBool)
        rospy.wait_for_service(service_name, timeout=30)

        self.action_type = action_type
        self.state_key = f"{action_type}ed"

    def update(self):
        if manager.get_state(self.state_key):
            rospy.loginfo(f"%s: Cube already {self.action_type}ed!", node_name)
            return pt.common.Status.SUCCESS

        try:
            response = self.service()
            manager.set_state("arm_tucked", False)

            if response.success:
                manager.set_state(self.state_key, True)
                rospy.loginfo(f"%s: {self.action_type.capitalize()} cube succeeded!", node_name)
                return pt.common.Status.SUCCESS
            else:
                rospy.loginfo(f"%s: {self.action_type.capitalize()} cube failed!", node_name)
                return pt.common.Status.FAILURE

        except rospy.ServiceException:
            return pt.common.Status.RUNNING


class pick_cube(CubeManipulator):
    def __init__(self):
        super(pick_cube, self).__init__("pick")


class place_cube(CubeManipulator):
    def __init__(self):
        super(place_cube, self).__init__("place")


class RobotMover(pt.behaviour.Behaviour):
    """基类：机器人移动行为"""

    def __init__(self, name):
        super(RobotMover, self).__init__(name)
        self.rate = rospy.Rate(10)
        self.move_msg = Twist()
        self.cmd_vel_pub = rospy.Publisher(
            rospy.get_param(rospy.get_name() + '/cmd_vel_topic'),
            Twist,
            queue_size=10
        )

    def stop_robot(self):
        """停止机器人"""
        self.move_msg = Twist()
        self.cmd_vel_pub.publish(self.move_msg)
        rospy.sleep(1)


class move(RobotMover):
    """简单移动行为"""
    MOVE_COMMANDS = {
        "forward": (0.2, 0),
        "backward": (-0.5, 0),
        "clockwise": (0, -1),
        "counter_clockwise": (0, 1)
    }

    def __init__(self, direction, steps):
        super(move, self).__init__("Move!")
        linear, angular = self.MOVE_COMMANDS.get(direction, (0, 0))
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular
        self.steps = steps

    def update(self):
        for _ in range(self.steps):
            if rospy.is_shutdown():
                break
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()

        self.stop_robot()
        rospy.loginfo("%s: Done", node_name)
        return pt.common.Status.SUCCESS


class move_to(RobotMover):
    """目标导航行为"""

    def __init__(self, goal):
        super(move_to, self).__init__("Move to goal!")
        self.goal = goal
        self.direction = 1 if goal == "second_table" else -1

    def update(self):
        rospy.loginfo("%s: Moving towards the %s...", node_name, self.goal)

        # 转向
        self.execute_turn()

        # 前进
        self.execute_forward()

        rospy.loginfo("%s: Reached the %s", node_name, self.goal)
        return pt.common.Status.SUCCESS

    def execute_turn(self):
        """执行转向动作"""
        self.move_msg.angular.z = -0.1 * self.direction
        for _ in range(314):
            if rospy.is_shutdown(): break
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
        self.stop_robot()

    def execute_forward(self):
        """执行前进动作"""
        self.move_msg.linear.x = 0.5
        for _ in range(20):
            if rospy.is_shutdown(): break
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
        self.stop_robot()


class tuck_arm(pt.behaviour.Behaviour):
    def __init__(self):
        super(tuck_arm, self).__init__("Tuck arm!")
        rospy.loginfo("Initialising tuck arm behaviour.")
        self.setup_action_client()

    def setup_action_client(self):
        """设置动作客户端"""
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        self.goal = PlayMotionGoal(motion_name='home', skip_planning=True)

    def update(self):
        if manager.get_state("arm_tucked"):
            rospy.loginfo("%s: Arm already tucked", node_name)
            return pt.common.Status.SUCCESS

        self.play_motion_ac.send_goal(self.goal)
        self.play_motion_ac.wait_for_result()
        manager.set_state("arm_tucked", True)
        rospy.loginfo("%s: Tuck arm succeeded", node_name)
        return pt.common.Status.SUCCESS


class move_head(pt.behaviour.Behaviour):
    def __init__(self, direction):
        super(move_head, self).__init__("Move head!")
        rospy.loginfo("Initialising move head behaviour.")
        self.setup_service(direction)

    def setup_service(self, direction):
        """设置服务客户端"""
        service_name = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(service_name, MoveHead)
        rospy.wait_for_service(service_name, timeout=30)
        self.direction = direction

    def update(self):
        if manager.get_state("head_state") == self.direction:
            rospy.loginfo("%s: Head already %s", node_name, self.direction)
            return pt.common.Status.SUCCESS

        response = self.move_head_srv(self.direction)
        if response.success:
            rospy.sleep(3)
            manager.set_state("head_state", self.direction)
            rospy.loginfo("Head %s succeeded", self.direction)
            return pt.common.Status.SUCCESS

        rospy.loginfo("Head %s failed!!!", self.direction)
        return pt.common.Status.FAILURE
if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		#BehaviourTree()
		behaviour_tree = BehaviourTree()
		behaviour_tree.run()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
