#!/usr/bin/env python3

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
import numpy as np
import math

# global variables
node_name = "Student BT"
CHECK = {"arm_tucked": False, "head_state": "unkown", "detected": False, "picked": False, "placed": False, "cube_on_table": False, "localized": False, "navigated_pick": False, "navigated_place": False, "kidnapped": False}

class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):

        rospy.loginfo("Initialising behaviour tree")

        # tuck arm
        tuck_arm_safe = pt.composites.Sequence(
             name = "Tuck sequence",
             children=[move("backward", 10), tuck_arm(), move("forward", 10)]
        )
        tuck_fallback = pt.composites.Selector(
            name = "Tuck arm?",
            children=[check_state("arm_tucked"), tuck_arm()]
        )

        # lower head
        head_down_fallback = pt.composites.Selector(
            name = "move head down?",
            children=[check_head("down"), move_head("down")]
        )
        
        # higher head
        head_up_fallback = pt.composites.Selector(
            name = "move head up?",
            children=[check_head("up"), move_head("up")]
        )

        # detect cube
        detect_fallback = pt.composites.Selector(
            name = "Detect?",
            children=[check_state("detected"), detect_for_pick()]
        )

        # move to pick
        move_pick_fallback = pt.composites.Selector(
            name = "Move pick?",
            children=[check_state("navigated_pick"), navigation("pick")]
        )
        
        # pick cube
        pick_fallback = pt.composites.Selector(
            name = "Pick?",
            children=[check_state("picked"), pick_cube()]
        )
        
        # move to pick
        move_place_fallback = pt.composites.Selector(
            name = "Move place?",
            children=[check_state("navigated_place"), navigation("place")]
        )
        
        # place cube
        place_cube_safe = pt.composites.Sequence(
            name = "Place cube sequence",
            children=[place_cube(), move("clockwise", 30), tuck_arm(), move("counter_clockwise", 30)]
        )
        place_fallback = pt.composites.Selector(
            name = "Place?",
            children=[check_state("placed"), place_cube_safe]
        )
        
        # check cube on table
        reset_sequence = pt.composites.Sequence(
            name = "Reset sequence",
            children=[reset_CHECK(), tuck_fallback, head_up_fallback]
        )
        check_cube_fallback = pt.composites.Selector(
            name = "Check tube?",
            children=[check_cube_on_table(), reset_CHECK()]
        )

        # localization
        localization_fallback = pt.composites.Selector(
            name = "Localise?",
            children=[check_state("localized"), localization()]
        )
        
        # become the tree
        tree = RSequence(name="Main sequence", children=[tuck_fallback, head_up_fallback, localization_fallback, move_pick_fallback, head_down_fallback, detect_fallback, pick_fallback, move_place_fallback, place_fallback, check_cube_fallback])
        super(BehaviourTree, self).__init__(tree)

        # execute the behaviour tree
        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): self.tick_tock(1)
        
class check_state(pt.behaviour.Behaviour):
    def __init__(self, key):
        self.key = key
        super(check_state, self).__init__("check_"+ key)
    def update(self):
        if CHECK[self.key]:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class check_head(pt.behaviour.Behaviour):
    def __init__(self, key):
        self.key = key
        super(check_head, self).__init__("check_head_state")
    def update(self):
        if CHECK["head_state"] == self.key:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class localization(pt.behaviour.Behaviour):
    
    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg
    
    def __init__(self):
        # http://wiki.ros.org/amcl
        rospy.loginfo("%s: Initializing localization...", node_name)
        self.rate = rospy.Rate(10)
        self.start = False
        # Subscriber: Robot's estimated pose in the map, with covariance
        self.amcl_pose_top_nm = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        self.amcl_pose = PoseWithCovarianceStamped()
        self.amcl_pose_sub = rospy.Subscriber(self.amcl_pose_top_nm, PoseWithCovarianceStamped, self.amcl_pose_callback)
        # Server
        # Initiate global localization, wherein all particles are dispersed randomly through the free space in the map.
        global_loc_srv_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.global_loc_srv = rospy.ServiceProxy(global_loc_srv_nm, Empty)
        rospy.wait_for_service(global_loc_srv_nm, timeout=30)
        # Server of clear costmap
        clear_costmap_srv_nm = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        self.clear_costmap_srv = rospy.ServiceProxy(clear_costmap_srv_nm, Empty)
        rospy.wait_for_service(clear_costmap_srv_nm, timeout=30)
        # Instantiate publishers
        cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_top, Twist, queue_size=10)

        super(localization, self).__init__("Localization!")
    def update(self):
        global CHECK
        if CHECK["localized"]:
            return pt.common.Status.SUCCESS
        elif not self.start:
            self.global_loc_srv()
            self.start = True
            return pt.common.Status.RUNNING
        else:
            move_msg = Twist()
            move_msg.angular.z = 0.5
            self.cmd_vel_pub.publish(move_msg)
            self.rate.sleep()
            norm = np.linalg.norm(self.amcl_pose.pose.covariance)
            # self.clear_costmap_srv() # clear the costmap
            # norm = max(self.amcl_pose.pose.covariance)
            if norm < 0.02:
                rospy.loginfo("%s: amcl_pose converged (norm = %f), localized succeeded!", node_name, norm)
                self.clear_costmap_srv() # clear the costmap
                self.start = False
                CHECK["localized"] = True
                CHECK["kidnapped"] = False
                return pt.common.Status.SUCCESS
            else:
                rospy.loginfo("%s: amcl_pose not converged (norm = %f), keep spinning...", node_name, norm)
                return pt.common.Status.RUNNING

    def calculate_covariance(self):
        rospy.loginfo("%s: Calculating covariance...", node_name)
        cov = 0.0
        return cov

class navigation(pt.behaviour.Behaviour):
    def check_kidnapped(self, pose1, pose2, pos_threshold=3, ori_threshold=0.5):
        pos_error = math.sqrt((pose1.position.x - pose2.position.x)**2 +
                                (pose1.position.y - pose2.position.y)**2 +
                                (pose1.position.z - pose2.position.z)**2)
        ori_error = math.sqrt((pose1.orientation.x - pose2.orientation.x)**2 +
                                 (pose1.orientation.y - pose2.orientation.y)**2 +
                                 (pose1.orientation.z - pose2.orientation.z)**2 +
                                 (pose1.orientation.w - pose2.orientation.w)**2)
        print("pos_error: ", pos_error, " ori_error: ", ori_error)
        if pos_error > pos_threshold and ori_error > ori_threshold:
            return True
        else:
            return False

    def amcl_pose_callback(self, amcl_pose_current):
        global CHECK
        if len(self.amcl_pose_list) < 5:
            self.amcl_pose_list.append(amcl_pose_current)
            return False
        else:
            self.amcl_pose_list.append(amcl_pose_current)
            while len(self.amcl_pose_list) > 5:
                self.amcl_pose_list.pop(0)
            if self.check_kidnapped(self.amcl_pose_list[-1].pose.pose, self.amcl_pose_list[0].pose.pose):
                CHECK["kidnapped"] = True
                CHECK["localized"] = False
            else:
                return False
    
    def goal_callback(self, msg):
        self.goal.target_pose = msg
    
    def move_feedback_callback(self, msg):
        self.move_feedback_pose = msg.feedback.base_position
    
    def __init__(self, goal):
        rospy.loginfo("%s: Initializing navigation...", node_name)
        self.rate = rospy.Rate(10)
        # Subscriber of amcl estimated pose
        self.amcl_pose_list = []
        amcl_pose_top_nm = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        rospy.Subscriber(amcl_pose_top_nm, PoseWithCovarianceStamped, self.amcl_pose_callback)
        # Subscriber of goal pose
        self.goal_str = goal
        self.goal = MoveBaseGoal()
        if goal == "pick":
            self.goal_pose_top_nm = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        elif goal == "place":
            self.goal_pose_top_nm = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        else:
            rospy.loginfo("%s: Invalid goal for navigation", node_name)
            self.goal_pose_top_nm = None
        if self.goal_pose_top_nm:
            self.goal_pose_sub = rospy.Subscriber(self.goal_pose_top_nm, PoseStamped, self.goal_callback)
        # Client of move_base
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if self.move_base_ac.wait_for_server(rospy.Duration(10)):
            rospy.loginfo("%s: Connected to /move_base server", node_name)
        else:
            rospy.loginfo("%s: Failed connecting to /move_base server", node_name)

        # become a behaviour
        super(navigation, self).__init__("Navigation")
    
    def update(self):
        # http://wiki.ros.org/move_ base#move_base-1
        # "The recommended way to send goals to move_base if you care about tracking their status is by using the SimpleActionClient"
        # https://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29
        global CHECK
        if CHECK["kidnapped"]:
            rospy.loginfo("%s: Kidnapped!!!")
            return pt.common.Status.FAILURE
        elif self.goal_str == "pick":
            if CHECK["navigated_pick"]:
                return pt.common.Status.SUCCESS
            else:
                self.move_base_ac.send_goal(self.goal)
                if self.move_base_ac.wait_for_result():
                    rospy.loginfo("%s: Navigation pick succeeded!", node_name)
                    CHECK["navigated_pick"] = True
                    return pt.common.Status.SUCCESS
                else:
                    self.move_base_ac.cancel_goal()
                    rospy.loginfo("%s: Navigation pick failed!", node_name)
                    return pt.common.Status.FAILURE
        elif self.goal_str == "place":
            if CHECK["navigated_place"]:
                return pt.common.Status.SUCCESS
            else:
                self.move_base_ac.send_goal(self.goal)
                if self.move_base_ac.wait_for_result():
                    rospy.loginfo("%s: Navigation place succeeded!", node_name)
                    CHECK["navigated_place"] = True
                    return pt.common.Status.SUCCESS
                else:
                    self.move_base_ac.cancel_goal()
                    rospy.loginfo("%s: Navigation place failed!", node_name)
                    return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

class reset_CHECK(pt.behaviour.Behaviour):
    def __init__(self):
        super(reset_CHECK, self).__init__("Reset CHECK")
    def update(self):
        global CHECK
        CHECK.update({"detected": False, "picked": False, "placed": False, "cube_on_table": False, "localized": True, "navigated_pick": False, "navigated_place": False})
        rospy.loginfo("%s: Initial state initialized.", node_name)
        return pt.common.Status.SUCCESS


class detect_for_pick(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("%s: Initializing detect cube...", node_name)
        # tf2 setup
        self.robot_base_frame = rospy.get_param(rospy.get_name() + '/robot_base_frame')
        self.tf_buffer = tf2_ros.Buffer()
        self.lisener = tf2_ros.TransformListener(self.tf_buffer)
        # Instantiate publishers
        aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.aruco_pose_pub = rospy.Publisher(aruco_pose_top, PoseStamped, queue_size=10)
        # become a behaviour
        super(detect_for_pick, self).__init__("Detect for pick!")
    
    def update(self):
        global CHECK
        # real-time check topic /robotics_intro/aruco_single/pose
        try:
            # get the pose in sensor frame
            detected_pose = rospy.wait_for_message('/robotics_intro/aruco_single/pose', PoseStamped, timeout=1)
            # transform into robot_base_frame and publish aruco_pose_top
            trans = self.tf_buffer.lookup_transform(self.robot_base_frame, 'xtion_rgb_optical_frame', rospy.Time.now(),rospy.Duration(1))
            detected_pose = tf2_geometry_msgs.do_transform_pose(detected_pose, trans)
            rospy.loginfo("%s: Detect cube succeded!", node_name)
            self.aruco_pose_pub.publish(detected_pose)
            CHECK["detected"] = True
            return pt.common.Status.SUCCESS
        except rospy.ROSException as e:
            rospy.loginfo("%s: Detect cube failed!", node_name)
            print("Error: %s"%e)
            CHECK["detected"] = False
            return pt.common.Status.FAILURE

class check_cube_on_table(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("%s: Initializing check place...", node_name)

        super(check_cube_on_table, self).__init__("Check place action")

    def update(self):
        global CHECK
        if CHECK["cube_on_table"]:
            rospy.loginfo("%s: Cube already place on table!", node_name)
            return pt.common.Status.SUCCESS
        else:
            try:
                rospy.wait_for_message('/robotics_intro/aruco_single/pose', PoseStamped, timeout=1)
                rospy.loginfo("%s: Check cube on table succeeded! End of task!", node_name)
                sys.exit(1)
                return pt.common.Status.SUCCESS
            except rospy.ROSException as e:
                rospy.loginfo("%s: Check cube on table failed!", node_name)
                # print("Error: %s"%e)
                return pt.common.Status.FAILURE

class pick_cube(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("%s: Initializing pick cube...", node_name)
        # Server
        pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.pick_srv = rospy.ServiceProxy(pick_srv_nm, SetBool)
        rospy.wait_for_service(pick_srv_nm, timeout=30)
        # become a behaviour
        super(pick_cube, self).__init__("Pick cube!")

    def update(self):
        global CHECK
        # success if done
        if CHECK["picked"]:
            rospy.loginfo("%s: Cube already picked!", node_name)
            return pt.common.Status.SUCCESS
        else:
            self.pick_srv_req = self.pick_srv()
            # if succesful
            if self.pick_srv_req.success:
                CHECK["picked"] = True
                CHECK["arm_tucked"] = False
                rospy.loginfo("%s: Pick cube succeded!", node_name)
                return pt.common.Status.SUCCESS
            # if failed
            elif not self.pick_srv_req.success:
                CHECK["arm_tucked"] = False
                rospy.loginfo("%s: Pick cube failed!", node_name)
                return pt.common.Status.FAILURE
            # if still trying
            else:
                return pt.common.Status.RUNNING

class place_cube(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("%s: Initializing place cube...", node_name)
        # Server
        place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.pĺace_srv = rospy.ServiceProxy(place_srv_nm, SetBool)
        rospy.wait_for_service(place_srv_nm, timeout=30)
        # become a behaviour
        super(place_cube, self).__init__("Place cube!")

    def update(self):
        global CHECK
        # success if done
        if CHECK["placed"]:
            rospy.loginfo("%s: Cube already placed!", node_name)
            return pt.common.Status.SUCCESS
        else:
            self.place_srv_req = self.pĺace_srv()
            # if succesful
            if self.place_srv_req.success:
                CHECK["placed"] = True
                CHECK["arm_tucked"] = False
                rospy.loginfo("%s: Place cube finished!", node_name)
                return pt.common.Status.SUCCESS
            # if failed
            elif not self.place_srv_req.success:
                CHECK["arm_tucked"] = False
                rospy.loginfo("%s: Place cube failed!", node_name)
                return pt.common.Status.FAILURE
            # if still trying
            else:
                return pt.common.Status.RUNNING

class move(pt.behaviour.Behaviour):
    def __init__(self, direction, steps):
        self.rate = rospy.Rate(10)
        self.move_msg = Twist()
        self.direction = direction
        self.steps = steps
        if self.direction == "forward":
            self.move_msg.linear.x = 0.2
        elif self.direction == "backward":
            self.move_msg.linear.x = -0.5
        elif self.direction == "clockwise":
            self.move_msg.angular.z = -1
        elif self.direction == "counter_clockwise":
            self.move_msg.angular.z = 1
        # Instantiate publishers
        cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_top, Twist, queue_size=10)
        # become a behaviour
        super(move, self).__init__("Move!")

    def update(self):
        rospy.loginfo("%s: Moving %s...", node_name, self.direction)
        cnt = 0
        while not rospy.is_shutdown() and cnt < self.steps:
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
            # print(cnt)
            cnt += 1
        # stop after finish
        self.move_msg = Twist()
        self.cmd_vel_pub.publish(self.move_msg)
        rospy.sleep(1)
        rospy.loginfo("%s: Done", node_name)
        return pt.common.Status.SUCCESS

class move_to(pt.behaviour.Behaviour):
    def __init__(self, goal):
        rospy.loginfo("%s: Initializing move to...", node_name)
        self.goal = goal
        self.reach_goal = False
        self.direction = 1
        if goal == "second_table":
            self.direction = 1
        elif goal == "first_table":
            self.direction = -1
        self.rate = rospy.Rate(10)
        self.move_msg = Twist()
        # Instantiate publishers
        cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_top, Twist, queue_size=10)
        # become a behaviour
        super(move_to, self).__init__("Move to goal!")

    def update(self):
        rospy.loginfo("%s: Moving towards the %s...", node_name, self.goal)
        # turn around
        self.move_msg.angular.z = -1 * self.direction
        cnt = 0
        while not rospy.is_shutdown() and cnt < 33:
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
            cnt += 1
        # pause
        self.move_msg.angular.z = 0
        self.cmd_vel_pub.publish(self.move_msg)
        rospy.sleep(1)
        # go straight forward
        self.move_msg.linear.x = 0.5
        cnt = 0
        while not rospy.is_shutdown() and cnt < 20:
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
            cnt += 1
        # stop before the second table
        self.move_msg.linear.x = 0
        self.move_msg.angular.z = 0
        self.cmd_vel_pub.publish(self.move_msg)
        rospy.loginfo("%s: Reached the %s", node_name, self.goal)
        return pt.common.Status.SUCCESS

class tuck_arm(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("Initialising tuck arm behaviour.")
        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True
        # become a behaviour
        super(tuck_arm, self).__init__("Tuck arm!")

    def update(self):
        global CHECK
        # already tucked the arm
        if CHECK["arm_tucked"]:
            rospy.loginfo("%s: Arm already tucked", node_name)
            return pt.common.Status.SUCCESS
        else:
            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            # if I was succesful! :)))))))))
            while not self.play_motion_ac.get_result():
                pass
            # then I'm finished!
            CHECK["arm_tucked"] = True
            # rospy.sleep(10) # wait for arm to tuck in gazebo
            rospy.loginfo("%s: Tuck arm succeeded", node_name)
            return pt.common.Status.SUCCESS
            # if failed
            # elif not self.play_motion_ac.get_result():
            # 	rospy.loginfo("%s: Tuck arm failed", node_name)
            # 	return pt.common.Status.FAILURE
            # # if I'm still trying :|
            # else:
            # 	return pt.common.Status.RUNNING

class move_head(pt.behaviour.Behaviour):
    def __init__(self, direction):
        rospy.loginfo("Initialising move head behaviour.")
        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)
        # head movement direction; "down" or "up"
        self.direction = direction
        # become a behaviour
        super(move_head, self).__init__("Move head!")

    def update(self):
        global CHECK
        if CHECK["head_state"] == self.direction:
            rospy.loginfo("%s: Head already %s", node_name, CHECK["head_state"])
            return pt.common.Status.SUCCESS
        else:
            self.move_head_req = self.move_head_srv(self.direction)
            # if succesful
            if self.move_head_req.success:
                rospy.sleep(3) # wait for head move in gazebo
                CHECK["head_state"] = self.direction
                print("Head", self.direction, "succeeded")
                return pt.common.Status.SUCCESS
            # if failed
            elif not self.move_head_req.success:
                print("Head",self.direction,"failed!!!")
                return pt.common.Status.FAILURE
            # if still trying
            else:
                return pt.common.Status.RUNNING

if __name__ == "__main__":


    rospy.init_node('main_state_machine')
    try:
        BehaviourTree()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()