

import numpy as np
from numpy import linalg as LA
import time

import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback, MoveBaseActionGoal

from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest 

DICT = {"ARMED_TUCKED": False, "HEAD_UP": False, "HEAD_DOWN": False, "LOCALIZED": False, "STARTED_LOC": False, "PICKED": False, "PLACED": False, "DETECTED": False, "RESPAWNED": True, "AT_PICK": False}

class tuckarm(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):
        global DICT

        if DICT["ARMED_TUCKED"]: 
            return pt.common.Status.SUCCESS
        
        self.play_motion_ac.send_goal(self.goal)

        if self.play_motion_ac.get_result():
            DICT["ARMED_TUCKED"] = True
            rospy.sleep(10)
            return pt.common.Status.SUCCESS

        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        else:
            return pt.common.Status.RUNNING

class movehead_up(pt.behaviour.Behaviour):

    def __init__(self):
        rospy.loginfo("Initialising move head up behaviour.")
        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # become a behaviour
        super(movehead_up, self).__init__("Upper head!")

    def update(self):
        global DICT

        if DICT["HEAD_UP"]:
            return pt.common.Status.SUCCESS

        self.move_head_req = self.move_head_srv("up")
        if self.move_head_req:
            DICT["HEAD_DOWN"] = False
            DICT["HEAD_UP"] = True
            rospy.sleep(3)
            return pt.common.Status.SUCCESS

        return pt.common.Status.RUNNING

class movehead_down(pt.behaviour.Behaviour):

    def __init__(self):
        rospy.loginfo("Initialising move head down behaviour.")
        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # become a behaviour
        super(movehead_down, self).__init__("Lower head!")

    def update(self):
        global DICT

        if DICT["HEAD_DOWN"]:
            return pt.common.Status.SUCCESS

        self.move_head_req = self.move_head_srv("down")
        if self.move_head_req.success:
            DICT["HEAD_DOWN"] = True
            DICT["HEAD_UP"] = False
            rospy.sleep(3)
            return pt.common.Status.SUCCESS

        return pt.common.Status.RUNNING

class pickup(pt.behaviour.Behaviour):
    def __init__(self):

        # Parameters
        self.pick_srv = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.aruco_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        
        rospy.wait_for_service(self.pick_srv, timeout=30)
        
        super(pickup, self).__init__("PickUp Cube!")

    def update(self):
        global DICT

        service = rospy.ServiceProxy(self.pick_srv, SetBool)
        answer = service(False)
        if answer.success == True:
            DICT["PICKED"] = True
            return pt.common.Status.SUCCESS
        elif answer.success == False:
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

class pickup_cond(pt.behaviour.Behaviour):
    def __init__(self):
        super(pickup_cond, self).__init__("PickUp Cube!")

    def update(self):
        global DICT
        if DICT["PICKED"]:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class place(pt.behaviour.Behaviour):
    def __init__(self):

        # Parameters
        self.place_srv = rospy.get_param(rospy.get_name() + '/place_srv')
        self.aruco_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        
        rospy.wait_for_service(self.place_srv, timeout=30)
        
        super(place, self).__init__("Place Cube!")

    def update(self):
        global DICT

        service = rospy.ServiceProxy(self.place_srv, SetBool)
        answer = service(False)
        if answer.success == True:
            DICT["PLACED"] = True
            return pt.common.Status.SUCCESS
        elif answer.success == False:
            DICT["PICKED"] = False
            DICT["PLACED"] = False
            DICT["ARMED_TUCKED"] = False
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

class place_cond(pt.behaviour.Behaviour):
    def __init__(self):
        super(place_cond, self).__init__("PickUp Cube?!")

    def update(self):
        global DICT
        if DICT["PLACED"]:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class movebase(pt.behaviour.Behaviour):
    def __init__(self, pose):

        rospy.loginfo("Initialising move base behavior.")

        self.pose = pose
        self.at_position = False
        self.going = False

        # Parameters
        self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        self.clear_costmap_service = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        self.amcl_est = rospy.get_param(rospy.get_name() + '/amcl_estimate')

        # Services
        self.clear_costmap_srv = rospy.ServiceProxy(self.clear_costmap_service, Empty)
        rospy.wait_for_service(self.clear_costmap_service, timeout=30)

        # Subscribers
        rospy.Subscriber(self.pick_pose_top, PoseStamped, self.callback_pick_pose)
        rospy.Subscriber(self.place_pose_top, PoseStamped, self.callback_place_pose)
        rospy.Subscriber(self.amcl_est, PoseWithCovarianceStamped, self.callback_amcl_est)

        # Setting up server
        rospy.loginfo("Waiting for /move_base action server.")
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(20)):
            rospy.loginfo("Could not connect to /move_base action server...")
            exit()
        rospy.loginfo("Connected to /move_base action server!")

        super(movebase, self).__init__("Move base!!")

    def update(self):
        global DICT

        self.clear_costmap_srv()

        goal = MoveBaseGoal()
        if self.pose == "pick":
            goal.target_pose = self.pick_pose
        elif self.pose == "place":
            goal.target_pose = self.place_pose

        self.at_position = True
        self.move_base_ac.send_goal(goal)
        success = self.move_base_ac.wait_for_result()

        if not self.at_position:
            return pt.common.Status.FAILURE
        if success:
            self.at_position = True
            return pt.common.Status.SUCCESS

    def callback_pick_pose(self, data):
        self.pick_pose = data

    def callback_place_pose(self, data):
        self.place_pose = data

    def callback_amcl_est(self, data):
        global DICT
        self.amcl_converged = data
        #rospy.loginfo(LA.norm(self.amcl_converged.pose.covariance))
        if LA.norm(self.amcl_converged.pose.covariance) > 0.049:
            self.at_position = False
            self.move_base_ac.cancel_all_goals()
            self.clear_costmap_srv()
            DICT["LOCALIZED"] = False
            

class localization(pt.behaviour.Behaviour):
    def __init__(self):

        rospy.loginfo("Initialising localizatiion behavior.")

        self.localized = False

        # Parameters
        self.global_loc_srv = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.amcl_est = rospy.get_param(rospy.get_name() + '/amcl_estimate')

        # Subscriber
        rospy.Subscriber(self.amcl_est, PoseWithCovarianceStamped, self.callback_amcl_est)

        # Publisher
        self.cmd_vel_top_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # Waiting for service
        self.localization_service = rospy.ServiceProxy(self.global_loc_srv, Empty)
        rospy.wait_for_service(self.global_loc_srv, timeout=30)

        super(localization, self).__init__("Localize!!")

    def update(self):
        global DICT

        if DICT["LOCALIZED"]:
            return pt.common.Status.SUCCESS

        if not DICT["STARTED_LOC"]:
            DICT["STARTED_LOC"] = True
            self.localization_service()

        rate = rospy.Rate(10)
        rate.sleep()

        if LA.norm(self.amcl_converged.pose.covariance) > 0.02:
            msg = Twist()
            msg.angular.z = 0.5
            self.cmd_vel_top_pub.publish(msg)
            return pt.common.Status.RUNNING
        else:
            msg = Twist()
            msg.angular.z = 0
            self.cmd_vel_top_pub.publish(msg)
            DICT["LOCALIZED"] = True
            DICT["STARTED_LOC"] = False
            self.cnt = 0
            return pt.common.Status.SUCCESS

    def callback_amcl_est(self, data):
        self.amcl_converged = data

class localization_cond(pt.behaviour.Behaviour):
    def __init__(self):
        super(localization_cond, self).__init__("Localize?!")

    def update(self):
        global DICT
        if DICT["LOCALIZED"]:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class respawn(pt.behaviour.Behaviour):
    def __init__(self):

        rospy.loginfo("Initialising respawn behavior.")

        self.delete_cube_name = "/gazebo/delete_model"
        self.spawn_cube_name = "/gazebo/spawn_sdf_model"
        
        rospy.wait_for_service(self.delete_cube_name, timeout=30)
        rospy.wait_for_service(self.spawn_cube_name, timeout=30)

        self.delete_model_srv = rospy.ServiceProxy(self.delete_cube_name, DeleteModel)
        self.spawn_model_srv = rospy.ServiceProxy(self.spawn_cube_name, SpawnModel)

        self.delete_cube = DeleteModelRequest('aruco_cube')
        
        self.initial_pose = Pose()
        self.initial_pose.position.x = -1.130530
        self.initial_pose.position.y = -6.653650
        self.initial_pose.position.z = 0.862500

        f = open('/home/n/i/nisven/catkin_ws/src/robi_final_project/tiago_simulation/tiago_gazebo/models/aruco_cube/aruco_cube.sdf','r')
        self.sdffile = f.read()

        super(respawn, self).__init__("Respawn!!")

    def update(self):
        global DICT

        self.delete_model_srv(self.delete_cube)
        rate = rospy.Rate(1)
        rate.sleep()
        self.spawn_model_srv("aruco_cube", self.sdffile, "/", self.initial_pose, "world")
        return pt.common.Status.SUCCESS

class detect_aruco(pt.behaviour.Behaviour):
    def __init__(self):

        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        self.last_call = time.time()

        rospy.Subscriber(self.aruco_pose_top, PoseStamped, self.callback_aruco_pose)
        
        super(detect_aruco, self).__init__("Detect!!")

    def update(self):
        global DICT

        rospy.sleep(1)
        if DICT["DETECTED"]:
            return pt.common.Status.SUCCESS

        if abs(self.last_call - time.time()) > 0.4:
            rospy.loginfo("not detected")
            DICT["DETECTED"] = False
            DICT["PICKED"] = False
            DICT["PLACED"] = False
            DICT["ARMED_TUCKED"] = False
            return pt.common.Status.FAILURE
        else:
            rospy.loginfo("detected")
            DETECTED = True
            return pt.common.Status.SUCCESS

    def callback_aruco_pose(self, data):
        self.aruco_pose = data
        self.last_call = time.time()
