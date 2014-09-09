#! /usr/bin/env python
'''
@author: Sammy Pfeiffer


'''
import rospy
from sensor_msgs.msg import JointState
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest, ExecuteKnownTrajectoryResponse
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from learn_from_robot.srv import Enabler, EnablerRequest, EnablerResponse

import copy
from cgitb import enable
from learn_from_robot.srv._Enabler import Enabler

JOINT_STATES_TOPIC = '/joint_states'
JOINT_ERROR_STATES_TOPIC = '/joint_error_states'
EXECUTE_KNOWN_TRAJ_SRV = '/execute_kinematic_path'
SENDER_ENABLE_SRV = '/activation_manager_pose_sender'

MAX_JOINT_ERROR = 0.05

def createRobotTrajectoryFromJointStates(joint_states, interesting_joint_list):
    """Given a joint_states message and the joints we are interested in moving create
    a RobotTrajectory that satisfies it"""
    curr_joint_states = copy.deepcopy(joint_states)
    rt = RobotTrajectory()
    jt = JointTrajectory()
    jt_point = JointTrajectoryPoint()
    
    jt.joint_names = interesting_joint_list
    jt.header.stamp = rospy.Time.now()
    for joint in interesting_joint_list:
        # Search for the position of the joint in the joint_states message
        idx_joint = curr_joint_states.name.index(joint)
        jt_point.positions.append(curr_joint_states.position[idx_joint])
        jt_point.velocities.append(0.0) # Set speeds to 0.0, in theory movements will be little and also with no effort

    jt_point.time_from_start = rospy.Duration(0.3)  # Arbitrary time, we are not dealing with speed issues here
    jt.points.append(jt_point)
    rt.joint_trajectory = jt
    return rt

def createExecuteKnownTrajectoryRequest(trajectory, wait_for_execution=True):
    """Create a ExecuteKnownTrajectoryRequest from the given data,
    trajectory must be a RobotTrajectory probably filled from a GetCartesianPath call"""
    ektr = ExecuteKnownTrajectoryRequest()
    ektr.trajectory = trajectory
    ektr.wait_for_execution = wait_for_execution
    return ektr

class PositionKeeper():
    def __init__(self):
        self.joints_sub = None #rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self.joint_state_cb) 
        self.current_joint_states = None

        self.joints_error_sub = None #rospy.Subscriber(JOINT_ERROR_STATES_TOPIC, JointState, self.joint_error_state_cb) 
        self.current_joint_error_states = None

        self.execute_known_traj_service = None #rospy.ServiceProxy(EXECUTE_KNOWN_TRAJ_SRV, ExecuteKnownTrajectory)

        self.head       = ['head_1_joint', 'head_2_joint']
        self.torso      = ['torso_1_joint', 'torso_2_joint']
        self.left_arm   = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.right_arm  = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_hand  = ['hand_left_index_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint']
        self.right_hand = ['hand_right_index_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint']

        self.all_joints = self.torso + self.head + self.left_arm + self.right_arm + self.left_hand + self.right_hand
        
        self.default_joints = self.right_arm
        self.ignore_list = self.right_hand + self.left_hand + self.torso + self.head + self.left_arm
        
        self.service_enabled = False
        
        rospy.Service(SENDER_ENABLE_SRV, Enabler, self.enabler_cb)
        # Here we fake a servicecall to activate so we only have one point of un/subscribing to topics
        self.enabler_cb(EnablerRequest(True))
        rospy.loginfo("Finished initialization!")

    def enabler_cb(self, data):
        """Callback for enabling and disabling the sender calls and in general the node"""
        resp = EnablerResponse()
        if data.enable == self.service_enabled: # Give "error" if nothing is done
            resp.error = "We are already in this state: " + str(self.service_enabled)
        else: # Do stuff
            if data.enable == False:
                # Unsubscribe stuff
                self.joints_sub.unregister()
                self.joints_error_sub.unregister()
                self.execute_known_traj_service.close()
                self.service_enabled = False

            elif data.enable == True:
                # Resubscribe stuff
                rospy.loginfo("Subscribing to joint_states")
                self.joints_sub = rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self.joint_state_cb)
                while self.current_joint_states == None:
                    rospy.sleep(0.2)

                rospy.loginfo("Subscribing to joint_error_states")
                self.joints_error_sub = rospy.Subscriber(JOINT_ERROR_STATES_TOPIC, JointState, self.joint_error_state_cb)
                while self.current_joint_error_states == None:
                    rospy.sleep(0.2)

                rospy.loginfo("Connecting to MoveIt! known trajectory executor server '" + EXECUTE_KNOWN_TRAJ_SRV + "'...")
                self.execute_known_traj_service = rospy.ServiceProxy(EXECUTE_KNOWN_TRAJ_SRV, ExecuteKnownTrajectory)
                self.execute_known_traj_service.wait_for_service()
                
                self.service_enabled = True
            else:
                rospy.logerr("This should never happen, enable is not True nor False")
        resp.enabled_status = self.service_enabled
        return resp


    def joint_state_cb(self, data):
        self.current_joint_states = data
        
    def joint_error_state_cb(self, data):
        self.current_joint_error_states = data
        
    def get_joints_that_need_new_goal(self):
        """Read the error of the joints and give back the name
        of the joints that have enough errors to need relocation"""
        joints_to_move = []
        for joint, error in zip(self.current_joint_error_states.name, self.current_joint_error_states.position):
            if abs(error) > MAX_JOINT_ERROR and (joint not in self.ignore_list):
                joints_to_move.append(joint)
        return joints_to_move

    def decide_group_by_joints_to_move(self, joints_to_move):
        """Given a list of joints to move return the group that contains these joints"""
        # TODO: implement logic to parse joints and try to use the tiniest group  that contains the joints
        group_to_move = self.all_joints
        
        return group_to_move

    def get_joint_states_of_only_wanted_joints(self, curr_joint_states, interesting_joints):
        """Given a JointState message, with current joints, and the joints we are interested,
        return a JointState message with only the interesting joints"""
        js = JointState()
        for i_j_name in interesting_joints:
            for j_name, j_value in zip(curr_joint_states.name, curr_joint_states.position):
                if i_j_name == j_name:
                    js.name.append(i_j_name)
                    js.position.append(j_value)
        rospy.loginfo("JS to send will be: " + str(js))
        return js

    def run(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            # Check if there is error in the current joint states enough to need to send new goal position
            if not self.service_enabled:
                r.sleep()
                continue
            new_goal_joints = self.get_joints_that_need_new_goal()
            if len(new_goal_joints) > 0: # If there is any joint to update...
                rospy.loginfo("Sending new pose because of joints: " + str(new_goal_joints))
                # Create a robot trajectory with the current joint state and the actuated joints
                to_move_jointstates = self.get_joint_states_of_only_wanted_joints(self.current_joint_states, self.default_joints)
                rt = createRobotTrajectoryFromJointStates(to_move_jointstates, self.default_joints)
                #rospy.loginfo("robot traj is: \n" + str(rt))
                # Create a goal for the execute_known_trajectory service which will take care of groups and joints
                ektr = createExecuteKnownTrajectoryRequest(rt)
                self.execute_known_traj_service.call(ektr)
            r.sleep()
            

if __name__ == '__main__':
    rospy.init_node('keep_curr_position')
    pk = PositionKeeper()
    pk.run()
    
    