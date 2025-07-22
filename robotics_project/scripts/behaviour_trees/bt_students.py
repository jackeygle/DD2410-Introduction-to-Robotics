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


# Example usage
manager = StateManager()

class BehaviourTree(ptr.trees.BehaviourTree):
    def __init__(self):
        rospy.loginfo("Initialising behaviour tree")
        tree = self.create_main_sequence()
        super(BehaviourTree, self).__init__(tree)

    def create_main_sequence(self):
        # Main task sequence
        main_sequence = RSequence(
            name="Main sequence",
            children=[
            # Check if kidnapped and handle
            pt.composites.Selector(
                name="Kidnapped Check",
                children=[
                    StateCheck("kidnapped", False),
                    self.create_recovery_sequence()
                ]
            ),
            # Check navigation failure and handle
            pt.composites.Selector(
                name="Navigation Failure Check",
                children=[
                    StateCheck("navigation_failed", False),
                    self.create_navigation_recovery_sequence()
                ]
            ),
                # Normal task sequence
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
        """Create recovery sequence"""
        return pt.composites.Sequence(
            name="Recovery Steps",
            children=[
                # Re-localization sequence
                move_head("up"),  # Raise head for better localization
                localization(),  # Re-localize

                # Continue unfinished tasks
                pt.composites.Selector(
                    name="Resume Task",
                    children=[
                        # If cube not detected yet, continue to pick location
                        pt.composites.Sequence(
                            name="Resume Pick Navigation",
                            children=[
                                StateCheck("detected", False),
                                navigation("pick")
                            ]
                        ),
                        # If cube already detected, continue to place location
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
        """Create navigation failure recovery sequence"""
        return pt.composites.Sequence(
            name="Navigation Recovery Steps",
            children=[
                # Reset navigation failure state
                ResetNavigationFailure(),
                # Re-localization sequence
                move_head("up"),

                # Re-navigate
                pt.composites.Selector(
                    name="Retry Navigation",
                    children=[
                        # If cube not detected yet, retry to pick location
                        pt.composites.Sequence(
                            name="Retry Pick Navigation",
                            children=[
                                StateCheck("detected", False),
                                navigation("pick")
                            ]
                        ),
                        # If cube already detected, retry to place location
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
        # Determine check type based on place parameter: "place" if place, otherwise "pick"
        check_type = "place" if place else "pick"
        
        # Create a sequence node loc_and_nav with two steps:
        # 1. Localization (localization_fallback)
        # 2. Navigation (navigation_fallback)
        #
        # Logic: First ensure robot completes localization (execute localization() if not localized),
        # then execute navigation() behavior to go to specified location (pick or place).
        loc_and_nav = pt.composites.Sequence(
            name=f"loc and nav {check_type}",
            children=[
                self.create_localization_fallback(),  # Try localization, execute localization behavior if failed
                self.create_navigation_fallback(check_type)  # Try navigation, perform navigation behavior if failed
            ]
        )
        
        # Return a Selector node containing two child nodes:
        # 1. CheckRobotState(check_type) - Check robot state (localized, not kidnapped, navigated to target)
        #    - If this check succeeds (e.g., already localized and reached target), no need to re-localize and navigate
        # 2. loc_and_nav - If state check fails, attempt to execute localization and navigation sequence
        #
        # Logic: First check if current robot state meets requirements (e.g., already localized and navigated),
        # if satisfied, skip re-localization and navigation.
        # If not satisfied (e.g., not localized or not navigated to target), execute loc_and_nav sequence.
        return pt.composites.Selector(
            name=f"localization without kidnapped {' place' if place else ''}",
            children=[
                CheckRobotState(check_type),  # Check robot state (not kidnapped, localized, navigated)
                loc_and_nav  # If state doesn't match, execute localization and navigation behaviors
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
                    f"navigated_{goal}",          # State key to check. E.g., if goal is "pick", state key is "navigated_pick"
                    f"Check navigate to {goal}",  # Description for the state check node name in behavior tree
                    f"navigated {goal}!!!!!!!!!!!",    # Success message when state is True
                    f"not navigated {goal}!!!!!!!!!!!" # Failure message when state is False
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

# Add new behavior class to handle state reset
class ResetNavigationFailure(pt.behaviour.Behaviour):
    def __init__(self):
        super(ResetNavigationFailure, self).__init__("Reset Navigation Failure")
        
    def update(self):
        rospy.loginfo("%s: Resetting navigation failure state", node_name)
        manager.set_state("navigation_failed", False)
        # Don't reset other states, maintain task progress
        return pt.common.Status.SUCCESS

tuck_check = StateCheck("arm_tucked", True, "Check arm tuck")

localization_check = StateCheck("localized", True, "Check localization")


class localization(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("%s: Initializing localization...", node_name)
        # Basic initialization
        super(localization, self).__init__("Localization!")
        self.rate = rospy.Rate(10)
        self.start = False

        # Setup subscribers and services
        self.setup_subscribers_and_services()

    def setup_subscribers_and_services(self):
        # AMCL pose subscription
        self.amcl_pose = PoseWithCovarianceStamped()
        self.amcl_pose_sub = rospy.Subscriber(
            rospy.get_param(rospy.get_name() + '/amcl_estimate'),
            PoseWithCovarianceStamped,
            self.amcl_pose_callback
        )

        # Service client setup
        self.global_loc_srv = rospy.ServiceProxy(
            rospy.get_param(rospy.get_name() + '/global_loc_srv'),
            Empty
        )
        self.clear_costmap_srv = rospy.ServiceProxy(
            rospy.get_param(rospy.get_name() + '/clear_costmaps_srv'),
            Empty
        )

        # CMD_VEL publisher
        self.cmd_vel_pub = rospy.Publisher(
            rospy.get_param(rospy.get_name() + '/cmd_vel_topic'),
            Twist,
            queue_size=10
        )

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg

    def start_localization(self):
        """Start localization process"""
        self.global_loc_srv()
        self.start = True
        status_msg = 'Relocalizing !!!!!!' if manager.get_state("kidnapped") else 'localizing !!!!!!'
        rospy.loginfo(status_msg)
        return pt.common.Status.RUNNING

    def spin_and_check(self):
        """Spin robot and check localization status"""
        # Send rotation command
        move_msg = Twist()
        move_msg.angular.z = 0.5
        self.cmd_vel_pub.publish(move_msg)
        self.rate.sleep()

        # Check localization accuracy
        norm = np.linalg.norm(self.amcl_pose.pose.covariance)
        if norm < 0.02:
            return self.handle_localization_success(norm)
        else:
            rospy.loginfo("%s: amcl_pose not converged (norm = %f), keep spinning...", node_name, norm)
            return pt.common.Status.RUNNING

    def handle_localization_success(self, norm):
        """Handle successful localization"""
        rospy.loginfo("%s: amcl_pose converged (norm = %f), localized succeeded!", node_name, norm)
        self.clear_costmap_srv()
        self.start = False
        manager.set_state("localized", True)
        manager.set_state("kidnapped", False)
        rospy.loginfo("localized!!!!!!!!!!!")
        return pt.common.Status.SUCCESS

    def update(self):
        # If already localized successfully, return success directly
        if manager.get_state("localized"):
            return pt.common.Status.SUCCESS

        # If not started localization yet, start localization process
        if not self.start:
            return self.start_localization()

        # In progress of localization, continue spinning and checking
        return self.spin_and_check()



class CheckRobotState(pt.behaviour.Behaviour):
    def __init__(self, check_type="pick"):
        """
        Initialize robot state check node.

        :param check_type: Check type, options are "pick" or "place".
                           - When check_type is "pick", check robot state before picking objects.
                           - When check_type is "place", check robot state before placing objects.
        """
        # Create a name for this behavior node based on check type for debugging and viewing behavior tree structure
        name = f"Check Robot State for {check_type}"
        super(CheckRobotState, self).__init__(name)

        self.check_type = check_type

        # Define a state check dictionary self.state_checks with multiple state items to check.
        # Dictionary format:
        #   state_key: (failure_value, message)
        # This means if state_key equals failure_value in state manager, check fails and outputs message log.
        self.state_checks = {
            "kidnapped": (True, "Robot is kidnapped"),                     # kidnapped state True means robot is "kidnapped" or lost localization
            "localized": (False, "Robot is not localized"),                # localized state False means robot is not yet localized
            f"navigated_{check_type}": (False, f"Robot is not navigated to {check_type}")
            # navigated_pick or navigated_place False means robot hasn't successfully navigated to target (pick or place point)
        }

    def update(self):
        # In update method, check each state item defined in self.state_checks sequentially.
        for state, (failure_value, message) in self.state_checks.items():
            current_value = manager.get_state(state)
            # If current state value equals defined failure_value, this check fails
            if current_value == failure_value:
                rospy.loginfo(message)  # Output corresponding log message for debugging and problem locating
                return pt.common.Status.FAILURE  # Return FAILURE indicating check failed

        # If all state checks pass, robot is in expected state:
        # - Not kidnapped
        # - Already localized as True
        # - navigated_pick or navigated_place as True (depending on check_type)
        rospy.loginfo(f"Robot is localized and navigated to {self.check_type} and not kidnapped.")
        return pt.common.Status.SUCCESS


class navigation(pt.behaviour.Behaviour):
    def __init__(self, goal):
        # Initialize navigation behavior class, print basic info and initialize parameters in constructor
        rospy.loginfo("%s: Initializing navigation...", node_name)
        self.rate = rospy.Rate(10)
        self.goal_str = goal                     # Save navigation goal type ("pick" or "place")
        self.goal = MoveBaseGoal()               # Create MoveBaseGoal object to send navigation goals to move_base

        # Setup topic subscription and publishers (for getting pose info, goal points, etc.)
        self.setup_subscribers_and_publishers()

        # Setup move_base action client to communicate with move_base action server for path planning and navigation
        self.setup_move_base_client()

        # Call parent constructor and name this behavior as "Navigation"
        super(navigation, self).__init__("Navigation")

    def setup_subscribers_and_publishers(self):
        # Get AMCL pose estimation topic name from parameter server
        amcl_pose_top_nm = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        # Create AMCL pose subscriber to monitor robot's current pose estimation
        self.amcl_pose_sub = rospy.Subscriber(
            amcl_pose_top_nm,
            PoseWithCovarianceStamped,
            self.amcl_pose_callback
        )

        # Determine location type based on goal_str, get corresponding target pose topic from parameter server
        if self.goal_str == "pick":
            self.goal_pose_top_nm = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        elif self.goal_str == "place":
            self.goal_pose_top_nm = rospy.get_param(rospy.get_name() + '/place_pose_topic')

        # If successfully got target topic name, subscribe to that topic to get target point pose (PoseStamped)
        if hasattr(self, 'goal_pose_top_nm'):
            self.goal_pose_sub = rospy.Subscriber(
                self.goal_pose_top_nm,
                PoseStamped,
                self.goal_callback
            )

        # Create publisher to publish velocity commands to /cmd_vel topic for stopping robot or simple movement control when necessary
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def setup_move_base_client(self):
        # Create move_base action client to send navigation goals to move_base action server
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)

        # Wait for move_base server to start, timeout 10 seconds. Report error if connection fails
        if not self.move_base_ac.wait_for_server(rospy.Duration(10)):
            rospy.logerr("%s: Failed connecting to /move_base server", node_name)
            raise RuntimeError("Failed to connect to move_base server")

    def check_covariance_threshold(self, covariance_matrix):
        # Extract position-related covariance elements from pose covariance to determine if robot's localization state is abnormal
        pos_cov = covariance_matrix[0] + covariance_matrix[7]
        # If position covariance > 0.07, consider robot "kidnapped", lost localization accuracy
        return pos_cov > 0.07

    def amcl_pose_callback(self, amcl_pose_current):
        # During navigation to "pick" or "place" target, if AMCL localization covariance exceeds threshold, consider robot localization lost (kidnapped)
        if self.goal_str in ["pick", "place"]:
            if self.check_covariance_threshold(amcl_pose_current.pose.covariance):
                rospy.loginfo("%s: Robot kidnapped during navigation!", node_name)
                # Update state indicating robot is kidnapped and no longer in localized state
                manager.set_state("kidnapped", True)
                manager.set_state("localized", False)

                # Stop robot immediately
                self.stop_robot()

                # Cancel current navigation goal
                self.move_base_ac.cancel_all_goals()

                # Update state based on current task progress
                # If object not detected yet, pick navigation failed; if object already picked, place navigation failed
                if not manager.get_state("detected"):
                    manager.set_state("navigated_pick", False)
                elif manager.get_state("picked"):
                    manager.set_state("navigated_place", False)

    def stop_robot(self):
        # Publish zero velocity message to stop robot movement
        zero_twist = Twist()
        self.cmd_vel_pub.publish(zero_twist)

    def goal_callback(self, msg):
        # When receiving target point pose message, store it in move_base navigation goal
        self.goal.target_pose = msg

    def execute_navigation(self, goal_type):
        # Core function for executing navigation behavior
        # Check if kidnapped before starting navigation, fail directly if yes
        if manager.get_state("kidnapped"):
            rospy.loginfo("%s: Navigation cancelled - robot is kidnapped", node_name)
            self.stop_robot()
            return pt.common.Status.FAILURE

        try:
            # Send navigation goal to move_base, let move_base start path planning and movement
            self.move_base_ac.send_goal(self.goal)

            # While waiting for navigation result, continuously poll kidnapped state, cancel goal if kidnapped occurs
            while not self.move_base_ac.wait_for_result(rospy.Duration(0.5)):
                if manager.get_state("kidnapped"):
                    self.stop_robot()
                    self.move_base_ac.cancel_goal()
                    return pt.common.Status.FAILURE

            # If move_base returned result, check status
            if self.move_base_ac.get_state() == GoalStatus.SUCCEEDED:
                # Navigation succeeded, update corresponding navigated_* state to True
                rospy.loginfo(f"%s: Navigation to {goal_type} succeeded!", node_name)
                manager.set_state(f"navigated_{goal_type}", True)
                return pt.common.Status.SUCCESS
            else:
                # Navigation failed, call failure handling logic
                return self.handle_navigation_failure(goal_type)

        except Exception as e:
            # If exception occurs, print error message and handle failure
            rospy.logerr(f"Navigation error: {str(e)}")
            return self.handle_navigation_failure(goal_type)

    def handle_navigation_failure(self, goal_type):
        """Handle navigation failure"""
        # Cancel current navigation goal
        self.move_base_ac.cancel_goal()
        rospy.loginfo(f"%s: Navigation to {goal_type} failed!", node_name)
        
        # Set navigation failure state
        manager.set_state("navigation_failed", True)
        # Reset corresponding navigation state
        manager.set_state(f"navigated_{goal_type}", False)
        # Reset localization state for subsequent re-localization
        manager.set_state("localized", False)
        
        return pt.common.Status.FAILURE

    def update(self):
        # Called every tick, check corresponding navigation state based on goal_str
        if self.goal_str == "pick":
            # If already navigated to pick point, return success directly, otherwise execute navigation
            if manager.get_state("navigated_pick"):
                return pt.common.Status.SUCCESS
            return self.execute_navigation("pick")
        elif self.goal_str == "place":
            # If already navigated to place point, return success directly, otherwise execute navigation
            if manager.get_state("navigated_place"):
                return pt.common.Status.SUCCESS
            return self.execute_navigation("place")
        else:
            # Invalid navigation goal type
            rospy.logwarn("%s: Invalid navigation goal", node_name)
            return pt.common.Status.FAILURE



class StateChecker(pt.behaviour.Behaviour):
    """Generic state checker class: Used to check if specified state variable is True in behavior tree.
       Returns success if state is True, otherwise returns failure."""

    def __init__(self, state_key, description=None, success_message=None, failure_message=None):
        """
        Initialize state check behavior.
        :param state_key: State key to check (string), used to get state value from state manager.
        :param description: Description of this state check for behavior name display (optional).
        :param success_message: Log message when state is True (optional).
        :param failure_message: Log message when state is False (optional).
        """

        # If description is empty, use "Check {state_key}" as behavior name
        name = description or f"Check {state_key}"
        super(StateChecker, self).__init__(name)

        self.state_key = state_key
        self.success_message = success_message
        self.failure_message = failure_message

    def update(self):
        # Get current state value from state manager
        state_value = manager.get_state(self.state_key)

        # If state value is True (condition met)
        if state_value:
            # Output log if success message exists
            if self.success_message:
                rospy.loginfo(self.success_message)
            # Return behavior tree success status
            return pt.common.Status.SUCCESS
        else:
            # When state is False, output log if failure message exists
            if self.failure_message:
                rospy.loginfo(self.failure_message)
            # Return behavior tree failure status
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
        """Setup TF listener"""
        self.robot_base_frame = rospy.get_param(rospy.get_name() + '/robot_base_frame')
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def setup_publisher(self):
        """Setup publisher"""
        aruco_pose_topic = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.aruco_pose_pub = rospy.Publisher(
            aruco_pose_topic,
            PoseStamped,
            queue_size=10
        )

    def detect_and_transform_pose(self):
        """Detect and transform pose"""
        # Get pose in sensor frame
        detected_pose = rospy.wait_for_message(
            '/robotics_intro/aruco_single/pose',
            PoseStamped,
            timeout=1
        )

        # Transform to robot base frame
        trans = self.tf_buffer.lookup_transform(
            self.robot_base_frame,
            'xtion_rgb_optical_frame',
            rospy.Time.now(),
            rospy.Duration(1)
        )
        return tf2_geometry_msgs.do_transform_pose(detected_pose, trans)

    def update(self):
        try:
            # Detect and transform pose
            transformed_pose = self.detect_and_transform_pose()

            # Publish transformed pose
            self.aruco_pose_pub.publish(transformed_pose)

            # Update state and return success
            manager.set_state("detected", True)
            
            rospy.loginfo("%s: Detect cube succeeded!", node_name)
            return pt.common.Status.SUCCESS

        except rospy.ROSException as e:
            # Handle detection failure
            manager.set_state("detected", False)
            manager.set_state("navigated_pick", False)  # Reset navigation state
            rospy.loginfo("%s: Detect cube failed! Error: %s", node_name, str(e))
            return pt.common.Status.FAILURE


class check_cube_on_table(pt.behaviour.Behaviour):
    def __init__(self):
        super(check_cube_on_table, self).__init__("Check place action")
        rospy.loginfo("%s: Initializing check place...", node_name)
        self.setup_model_state()
        self.setup_service()

    def setup_model_state(self):
        """Setup model state"""
        self.model_state = ModelState()
        # Set basic attributes
        self.model_state.model_name = 'aruco_cube'
        self.model_state.reference_frame = 'map'

        # Set position
        self.model_state.pose.position.x, y, z = -1.130530, -6.653650, 0.86250
        self.model_state.pose.position.y = y
        self.model_state.pose.position.z = z

        # Set orientation (default orientation)
        self.model_state.pose.orientation.w = 1
        self.model_state.twist = Twist()

    def setup_service(self):
        """Setup service client"""
        service_name = '/gazebo/set_model_state'
        rospy.wait_for_service(service_name)
        self.set_model_state_srv = rospy.ServiceProxy(service_name, SetModelState)

    def update(self):
        if manager.get_state("cube_on_table"):
            rospy.loginfo("%s: Cube already place on table!", node_name)
            return pt.common.Status.SUCCESS

        try:
            # Check if cube is on table
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
    """Base class: Generic behavior for grasping and placing cubes"""

    def __init__(self, action_type):
        super(CubeManipulator, self).__init__(f"{action_type.capitalize()} cube!")
        rospy.loginfo(f"%s: Initializing {action_type} cube...", node_name)

        # Setup service client
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
    """Base class: Robot movement behavior"""

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
        """Stop robot"""
        self.move_msg = Twist()
        self.cmd_vel_pub.publish(self.move_msg)
        rospy.sleep(1)


class move(RobotMover):
    """Simple movement behavior"""
    # Define movement commands with corresponding linear and angular velocities
    MOVE_COMMANDS = {
        "forward": (0.2, 0),            # Move forward, positive linear velocity
        "backward": (-0.5, 0),          # Move backward, negative linear velocity
        "clockwise": (0, -1),           # Clockwise rotation, negative angular velocity
        "counter_clockwise": (0, 1)     # Counter-clockwise rotation, positive angular velocity
    }

    def __init__(self, direction, steps):
        super(move, self).__init__("Move!")
        # Get velocity settings for specified direction from dictionary
        linear, angular = self.MOVE_COMMANDS.get(direction, (0, 0))
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular
        self.steps = steps          # Number of movement steps

    def update(self):
        # Execute movement for specified number of steps
        for _ in range(self.steps):
            if rospy.is_shutdown():
                break              # Stop movement if ROS node shutdown detected
            self.cmd_vel_pub.publish(self.move_msg)  # Publish velocity command
            self.rate.sleep()

        self.stop_robot()            # Stop robot
        rospy.loginfo("%s: Done", node_name)  # Log info to ROS log
        return pt.common.Status.SUCCESS   # Return success status


class tuck_arm(pt.behaviour.Behaviour):
    def __init__(self):
        super(tuck_arm, self).__init__("Tuck arm!")
        rospy.loginfo("Initialising tuck arm behaviour.")
        self.setup_action_client()

    def setup_action_client(self):
        """Setup action client"""
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
        """Setup service client"""
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