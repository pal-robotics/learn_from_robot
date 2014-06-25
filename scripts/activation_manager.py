#! /usr/bin/env python
'''
@author: Sammy Pfeiffer

Node to interact with screen messages to activate
or deactivate joints to do learning by demonstration
'''
import rospy
from learn_from_robot.msg import JointActivation
from dynamic_reconfigure import client, DynamicReconfigureParameterException

ACTIVATION_TOPIC = '/active_joints'
#TODO: Define this topic for real and it's interface
JOINTS_DYN_REC = '/joints_dyn_rec'

ACTIVATED = 1.0
DEACTIVATED = 0.01

class activation_manager():
    def __init__(self):
        rospy.loginfo("Subscribing to '" + ACTIVATION_TOPIC + "'")
        self.activation_subs = rospy.Subscriber(ACTIVATION_TOPIC, JointActivation, self.activation_cb)
        
        rospy.loginfo("Trying to connect a service client to '" + JOINTS_DYN_REC + "' dynamic reconfigure...")
        self.client = client.Client(JOINTS_DYN_REC)
        
        self.head = ['head_1_joint', 'head_2_joint'] # Does not work :( motors are different
        self.torso = ['torso_1_joint', 'torso_2_joint']
        self.left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_hand = ['hand_left_index_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint'] # Only the actuated
        self.right_hand = ['hand_right_index_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint'] # Only the actuated
        
        self.group_names = ['head', 'torso', 'left_arm', 'right_arm', 'left_hand', 'right_hand']

        
    def activation_cb(self, data):
        # Check if we got a group or a single joint to activate/deactivate
        joints = str(data.joints.data)
        if joints in self.group_names:
            list_of_joints = getattr(self, joints)
        else:
            list_of_joints = [joints]
        
        # Set the mode of the joint
        for joint in list_of_joints:
            if data.active.data: # If True, activate
                rospy.loginfo("Setting joint '" + joint + "' to ACTIVE")
                try:
                    config = self.client.update_configuration({joint : ACTIVATED})
                except DynamicReconfigureParameterException:
                    rospy.logwarn("Couldn't set '" + joint + "' to ACTIVATED")
            else:
                rospy.loginfo("Setting joint '" + joint + "' to DEACTIVATED")
                try:
                    config = self.client.update_configuration({joint : DEACTIVATED})
                except DynamicReconfigureParameterException:
                    rospy.logwarn("Couldn't set '" + joint + "' to DEACTIVATED")
        
        
if __name__ == '__main__':
    rospy.init_node('activation_manager_')
    am = activation_manager()
    rospy.spin()