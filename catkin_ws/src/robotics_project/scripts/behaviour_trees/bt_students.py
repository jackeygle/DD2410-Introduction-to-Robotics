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
            "kidnapped": False,
            "navigation_failed": False
            
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
                    StateCheck("kidnapped", False),
                    self.create_recovery_sequence()
                ]
            ),
            # 检查导航失败并处理
            pt.composites.Selector(
                name="Navigation Failure Check",
                children=[
                    StateCheck("navigation_failed", False),
                    self.create_navigation_recovery_sequence()
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
    def create_navigation_recovery_sequence(self):
        """创建导航失败恢复序列"""
        return pt.composites.Sequence(
            name="Navigation Recovery Steps",
            children=[
                # 重置导航失败状态
                ResetNavigationFailure(),
                # 重新定位序列
                move_head("up"),

                # 重新导航
                pt.composites.Selector(
                    name="Retry Navigation",
                    children=[
                        # 如果还没有检测到方块，重试去拾取位置
                        pt.composites.Sequence(
                            name="Retry Pick Navigation",
                            children=[
                                StateCheck("detected", False),
                                navigation("pick")
                            ]
                        ),
                        # 如果已经检测到方块，重试去放置位置
                        pt.composites.Sequence(
                            name="Retry Place Navigation",
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
        # 根据place参数决定检查类型，如果是place则检查“place”，否则检查“pick”
        check_type = "place" if place else "pick"
        
        # 创建一个顺序节点loc_and_nav包含两步：
        # 1. 定位（localization_fallback）
        # 2. 导航（navigation_fallback）
        #
        # 逻辑：先保证机器人完成定位（若未定位则执行localization()行为），
        # 然后执行navigation()行为前往指定地点（pick或place）。
        loc_and_nav = pt.composites.Sequence(
            name=f"loc and nav {check_type}",
            children=[
                self.create_localization_fallback(),  # 尝试定位，如果失败则执行定位行为
                self.create_navigation_fallback(check_type)  # 尝试导航，如果失败则进行导航行为
            ]
        )
        
        # 返回一个选择器节点(Selector)，它包含两个子节点：
        # 1. CheckRobotState(check_type) - 用于检查机器人状态（如是否已经定位、没有被绑架、且已导航到目标点）
        #    - 若此检查成功（例如已经定位且已到达目标点），则不需要重新定位和导航
        # 2. loc_and_nav - 如果状态检查失败，则尝试执行定位和导航的顺序动作
        #
        # 逻辑：优先检查当前机器人状态是否满足要求（例如已经localized且导航到位），若满足则不再重复定位和导航。
        # 若不满足（例如尚未定位或尚未导航至目标点），则执行loc_and_nav序列来完成定位与导航步骤。
        return pt.composites.Selector(
            name=f"localization without kidnapped {' place' if place else ''}",
            children=[
                CheckRobotState(check_type),  # 检查机器人状态（如未被劫持、已定位、已导航）
                loc_and_nav  # 若状态不符合，则执行定位与导航行为
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
                StateChecker(
                    f"navigated_{goal}",          # 要检查的状态键。例如如果goal是"pick"则状态键为"navigated_pick"。
                    f"Check navigate to {goal}",  # 描述信息，用于在行为树中标识该状态检查节点的名称。会显示为“Check navigate to pick”或“Check navigate to place”。
                    f"navigated {goal}!!!!!!!!!!!",    # 当状态为True时的成功提示消息，在日志中输出。例如“navigated pick!!!!!!!!!!!”。
                    f"not navigated {goal}!!!!!!!!!!!" # 当状态为False时的失败提示消息，在日志中输出。例如“not navigated place!!!!!!!!!!!”。
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
# 添加新的行为类来处理状态重置
class ResetNavigationFailure(pt.behaviour.Behaviour):
    def __init__(self):
        super(ResetNavigationFailure, self).__init__("Reset Navigation Failure")
        
    def update(self):
        rospy.loginfo("%s: Resetting navigation failure state", node_name)
        manager.set_state("navigation_failed", False)
        # 不重置其他状态，保持任务进度
        return pt.common.Status.SUCCESS

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
        初始化机器人状态检查节点。

        :param check_type: 检查类型，可选值为"pick"或"place"。
                           - 当check_type为"pick"时，表示检查机器人在拾取物品之前的状态是否正常。
                           - 当check_type为"place"时，表示检查机器人在放置物品之前的状态是否正常。
        """
        # 根据检查类型，为该行为节点创建一个名称，便于在调试和查看行为树结构时识别
        name = f"Check Robot State for {check_type}"
        super(CheckRobotState, self).__init__(name)

        self.check_type = check_type

        # 定义一个状态检查字典self.state_checks，其中包含多个要检查的状态项。
        # 字典的形式为：
        #   state_key: (failure_value, message)
        # 这表示如果state_key在状态管理器中等于failure_value则认为检查失败，并输出message日志。
        self.state_checks = {
            "kidnapped": (True, "Robot is kidnapped"),                     # kidnapped状态为True表示机器人被“劫持”或定位丢失
            "localized": (False, "Robot is not localized"),                # localized状态为False表示机器人尚未定位好
            f"navigated_{check_type}": (False, f"Robot is not navigated to {check_type}")
            # navigated_pick 或 navigated_place 为False，表示机器人未成功导航到目标地点（pick点或place点）
        }

    def update(self):
        # 在update方法中，依次检查self.state_checks中定义的状态项。
        for state, (failure_value, message) in self.state_checks.items():
            current_value = manager.get_state(state)
            # 如果当前状态值等于定义的failure_value，则认为这一条检查失败
            if current_value == failure_value:
                rospy.loginfo(message)  # 输出对应的日志信息，便于调试和定位问题
                return pt.common.Status.FAILURE  # 返回FAILURE表示检查不通过

        # 如果所有状态检查都通过，则表示机器人处于预期状态：
        # - 没有被kidnapped
        # - 已经localized为True
        # - navigated_pick或navigated_place为True（根据check_type不同）
        rospy.loginfo(f"Robot is localized and navigated to {self.check_type} and not kidnapped.")
        return pt.common.Status.SUCCESS


class navigation(pt.behaviour.Behaviour):
    def __init__(self, goal):
        # 初始化navigation行为类，在构造函数中进行基本信息打印和参数初始化
        rospy.loginfo("%s: Initializing navigation...", node_name)
        self.rate = rospy.Rate(10)
        self.goal_str = goal                     # 保存导航目标类型（"pick"或"place"）
        self.goal = MoveBaseGoal()               # 创建一个MoveBaseGoal对象，用于向move_base发送导航目标

        # 设置话题订阅和发布器（用于获取位姿信息、目标点等）
        self.setup_subscribers_and_publishers()

        # 设置move_base动作客户端（Action Client），用于与move_base动作服务器通信，进行路径规划和导航
        self.setup_move_base_client()

        # 调用父类构造函数并命名此行为为"Navigation"
        super(navigation, self).__init__("Navigation")

    def setup_subscribers_and_publishers(self):
        # 从参数服务器获取AMCL姿态估计话题名称
        amcl_pose_top_nm = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        # 创建AMCL位姿订阅者，用于监听机器人当前位姿的估计值
        self.amcl_pose_sub = rospy.Subscriber(
            amcl_pose_top_nm,
            PoseWithCovarianceStamped,
            self.amcl_pose_callback
        )

        # 根据goal_str确定要去的位置类型，从参数服务器获取相应的目标姿态话题
        if self.goal_str == "pick":
            self.goal_pose_top_nm = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        elif self.goal_str == "place":
            self.goal_pose_top_nm = rospy.get_param(rospy.get_name() + '/place_pose_topic')

        # 如果成功获取到目标话题名称，则订阅该话题，获取目标点位姿（PoseStamped）
        if hasattr(self, 'goal_pose_top_nm'):
            self.goal_pose_sub = rospy.Subscriber(
                self.goal_pose_top_nm,
                PoseStamped,
                self.goal_callback
            )

        # 创建一个发布器，用于向 /cmd_vel 话题发布速度指令，以在必要时停止机器人或进行简易移动控制
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def setup_move_base_client(self):
        # 创建一个move_base动作客户端，用于向move_base动作服务器发送导航目标
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)

        # 等待move_base服务器启动，超时10秒。如果未成功连接则报错
        if not self.move_base_ac.wait_for_server(rospy.Duration(10)):
            rospy.logerr("%s: Failed connecting to /move_base server", node_name)
            raise RuntimeError("Failed to connect to move_base server")

    def check_covariance_threshold(self, covariance_matrix):
        # 从姿态协方差中提取位置相关的协方差元素，用于判断机器人的定位状态是否异常
        pos_cov = covariance_matrix[0] + covariance_matrix[7]
        # 如果位置协方差大于0.07则认为机器人“被绑架”（kidnapped），定位精度丧失
        return pos_cov > 0.07

    def amcl_pose_callback(self, amcl_pose_current):
        # 在导航去“pick”或“place”目标时，如果AMCL定位协方差超过阈值，认为机器人定位丢失（kidnapped）
        if self.goal_str in ["pick", "place"]:
            if self.check_covariance_threshold(amcl_pose_current.pose.covariance):
                rospy.loginfo("%s: Robot kidnapped during navigation!", node_name)
                # 更新状态，表示机器人被劫持且不再处于localized状态
                manager.set_state("kidnapped", True)
                manager.set_state("localized", False)

                # 立即停止机器人
                self.stop_robot()

                # 取消当前导航目标
                self.move_base_ac.cancel_all_goals()

                # 根据当前任务进度更新状态
                # 如果还没检测到物体，则表示pick的导航失败；若已拾取物体，则表示place导航失败
                if not manager.get_state("detected"):
                    manager.set_state("navigated_pick", False)
                elif manager.get_state("picked"):
                    manager.set_state("navigated_place", False)

    def stop_robot(self):
        # 发布零速度消息，停止机器人移动
        zero_twist = Twist()
        self.cmd_vel_pub.publish(zero_twist)

    def goal_callback(self, msg):
        # 当接收到目标点位姿消息时，将其存入move_base的导航目标中
        self.goal.target_pose = msg

    def execute_navigation(self, goal_type):
        # 执行导航行为的核心函数
        # 在开始导航之前检查是否被劫持，如是则直接失败
        if manager.get_state("kidnapped"):
            rospy.loginfo("%s: Navigation cancelled - robot is kidnapped", node_name)
            self.stop_robot()
            return pt.common.Status.FAILURE

        try:
            # 发送导航目标给move_base，让move_base开始路径规划和移动
            self.move_base_ac.send_goal(self.goal)

            # 等待导航结果期间不断轮询kidnapped状态，如发生劫持则中途取消目标
            while not self.move_base_ac.wait_for_result(rospy.Duration(0.5)):
                if manager.get_state("kidnapped"):
                    self.stop_robot()
                    self.move_base_ac.cancel_goal()
                    return pt.common.Status.FAILURE

            # 如果move_base返回结果了，检查状态
            if self.move_base_ac.get_state() == GoalStatus.SUCCEEDED:
                # 导航成功，将相应的 navigated_* 状态更新为True
                rospy.loginfo(f"%s: Navigation to {goal_type} succeeded!", node_name)
                manager.set_state(f"navigated_{goal_type}", True)
                return pt.common.Status.SUCCESS
            else:
                # 导航失败则调用failure处理逻辑
                return self.handle_navigation_failure(goal_type)

        except Exception as e:
            # 如果出现异常，则打印错误信息并进行失败处理
            rospy.logerr(f"Navigation error: {str(e)}")
            return self.handle_navigation_failure(goal_type)

    def handle_navigation_failure(self, goal_type):
        """处理导航失败的情况"""
        # 取消当前导航目标
        self.move_base_ac.cancel_goal()
        rospy.loginfo(f"%s: Navigation to {goal_type} failed!", node_name)
        
        # 设置导航失败状态
        manager.set_state("navigation_failed", True)
        # 重置相应的导航状态
        manager.set_state(f"navigated_{goal_type}", False)
        # 重置定位状态以便后续重新定位
        manager.set_state("localized", False)
        
        return pt.common.Status.FAILURE

    def update(self):
        # 每次tick时调用此方法，根据goal_str检查对应的导航状态
        if self.goal_str == "pick":
            # 如果已经导航到拾取点则直接成功，否则执行导航
            if manager.get_state("navigated_pick"):
                return pt.common.Status.SUCCESS
            return self.execute_navigation("pick")
        elif self.goal_str == "place":
            # 如果已经导航到放置点则直接成功，否则执行导航
            if manager.get_state("navigated_place"):
                return pt.common.Status.SUCCESS
            return self.execute_navigation("place")
        else:
            # 无效的导航目标类型
            rospy.logwarn("%s: Invalid navigation goal", node_name)
            return pt.common.Status.FAILURE



class StateChecker(pt.behaviour.Behaviour):
    """通用状态检查类：用于在行为树中检查指定的状态变量是否为True。
       如果状态为True返回成功，否则返回失败。"""

    def __init__(self, state_key, description=None, success_message=None, failure_message=None):
        """
        初始化状态检查行为。
        :param state_key: 要检查的状态键（字符串），用于从状态管理器中获取状态值。
        :param description: 对该状态检查的描述，用于行为名称显示（可选）。
        :param success_message: 当状态为True时的日志消息（可选）。
        :param failure_message: 当状态为False时的日志消息（可选）。
        """

        # 如果description为空，则以"Check {state_key}"作为行为名称
        name = description or f"Check {state_key}"
        super(StateChecker, self).__init__(name)

        self.state_key = state_key
        self.success_message = success_message
        self.failure_message = failure_message

    def update(self):
        # 从状态管理器获取当前状态值
        state_value = manager.get_state(self.state_key)

        # 如果状态值为True（代表条件满足）
        if state_value:
            # 如果有成功提示消息则输出日志
            if self.success_message:
                rospy.loginfo(self.success_message)
            # 返回行为树成功状态
            return pt.common.Status.SUCCESS
        else:
            # 状态为False时，如有失败提示消息则输出日志
            if self.failure_message:
                rospy.loginfo(self.failure_message)
            # 返回行为树失败状态
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
            manager.set_state("navigated_pick", False)  # 重置导航状态
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
    # 定义移动命令与对应的线性和角速度
    MOVE_COMMANDS = {
        "forward": (0.2, 0),            # 向前移动，正线性速度
        "backward": (-0.5, 0),          # 向后移动，负线性速度
        "clockwise": (0, -1),           # 顺时针旋转，负角速度
        "counter_clockwise": (0, 1)     # 逆时针旋转，正角速度
    }

    def __init__(self, direction, steps):
        super(move, self).__init__("Move!")
        # 从字典中获取指定方向的速度设置
        linear, angular = self.MOVE_COMMANDS.get(direction, (0, 0))
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular
        self.steps = steps          # 移动的步数

    def update(self):
        # 执行移动指定的步数
        for _ in range(self.steps):
            if rospy.is_shutdown():
                break              # 检测到ROS节点关闭则停止移动
            self.cmd_vel_pub.publish(self.move_msg)  # 发布速度指令
            self.rate.sleep()

        self.stop_robot()            # 停止机器人
        rospy.loginfo("%s: Done", node_name)  # 记录信息到ROS日志
        return pt.common.Status.SUCCESS   # 返回成功状态



# class move_to(RobotMover):
#     """目标导航行为"""

#     def __init__(self, goal):
#         super(move_to, self).__init__("Move to goal!")
#         self.goal = goal
#         self.direction = 1 if goal == "second_table" else -1

#     def update(self):
#         rospy.loginfo("%s: Moving towards the %s...", node_name, self.goal)

#         # 转向
#         self.execute_turn()

#         # 前进
#         self.execute_forward()

#         rospy.loginfo("%s: Reached the %s", node_name, self.goal)
#         return pt.common.Status.SUCCESS

#     def execute_turn(self):
#         """执行转向动作"""
#         self.move_msg.angular.z = -0.1 * self.direction
#         for _ in range(314):
#             if rospy.is_shutdown(): break
#             self.cmd_vel_pub.publish(self.move_msg)
#             self.rate.sleep()
#         self.stop_robot()

#     def execute_forward(self):
#         """执行前进动作"""
#         self.move_msg.linear.x = 0.5
#         for _ in range(20):
#             if rospy.is_shutdown(): break
#             self.cmd_vel_pub.publish(self.move_msg)
#             self.rate.sleep()
#         self.stop_robot()


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