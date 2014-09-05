#! /usr/bin/env python
'''
@author: Sammy Pfeiffer

Node to interact with screen messages to activate
or deactivate joints to do learning by demonstration
'''
import rospy
from pal_common_msgs.msg import JointCurrent
#from pal_control_msgs.srv import CurrentLimit, CurrentLimitRequest

from pal_control_msgs.msg import ActuatorCurrentLimit

CURR_LIMIT_TOPIC = '/current_limit_controller/command'
CURR_LIMIT_STATE_TOPIC = '/current_limit_controller/state' 

ACTIVATION_TOPIC = '/active_joints'
ACTIVATION_SRV = '/active_joints_enabler'
CURRENT_LIMIT_SRV = '/current_limit_controller/set_current_limit'


class activation_manager():
    def __init__(self):
        rospy.loginfo("Subscribing to '" + ACTIVATION_TOPIC + "'")
        self.activation_subs = rospy.Subscriber(ACTIVATION_TOPIC, JointCurrent, self.new_activation_cb)
        
#         rospy.loginfo("Trying to connect to service'" + CURRENT_LIMIT_SRV + "'...")
#         self.current_limit_srv = rospy.ServiceProxy(CURRENT_LIMIT_SRV, CurrentLimit)
#         self.current_limit_srv.wait_for_service()

        rospy.loginfo("Setting up publisher for '" + CURR_LIMIT_TOPIC + "'")
        self.curr_limit_pub = rospy.Publisher(CURR_LIMIT_TOPIC, ActuatorCurrentLimit)
        rospy.sleep(0.3) # Publisher needs a bit of time to be able to publish for some reason
        
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
        
        rospy.loginfo("Finished initialization.")
        

        
#     def activation_cb(self, data):
#         # Check if we got a group or a single joint to activate/deactivate
#         joints = str(data.joints)
#         if joints in self.group_names:
#             list_of_joints = getattr(self, joints)
#         else:
#             list_of_joints = [joints]
#         
#         # Set the mode of the joint
#         for joint in list_of_joints:
#             req = CurrentLimitRequest()
#             req.actuator_name = joint.replace('joint', 'motor')
#             if 0.0 <= data.current_limit <= 1.0:
#                 rospy.loginfo("Setting joint '" + joint + "' to " + str(data.current_limit))
#                 req.current_limit = data.current_limit
#                 self.current_limit_srv.call(req)
#             else:
#                 rospy.logwarn("Sent current_limit value: " + str(data.current_limit) + " not in between 0.0 and 1.0")

    def new_activation_cb(self, data):
        rospy.loginfo("We got:\n" + str(data))
        # Check if we got a group or a joint to activate/deactivate
        joints = str(data.joints)
        if joints in self.group_names: # At the time of writting unused...
            list_of_joints = getattr(self, joints)
        else:
            list_of_joints = [joints]
            
        joint_to_motor = []
        for joint in list_of_joints:
            joint_to_motor.append(joint.replace('joint', 'motor'))
            
        updated_limits = ActuatorCurrentLimit()
        updated_limits.actuator_names = joint_to_motor
        updated_limits.current_limits = [data.current_limit] * len(joint_to_motor)
        curr_limit_values = rospy.wait_for_message(CURR_LIMIT_STATE_TOPIC, ActuatorCurrentLimit)
        to_send_msg = self.merge_values(curr_limit_values, updated_limits)
        self.curr_limit_pub.publish(to_send_msg)
        rospy.loginfo("We sent:\n"  + str(to_send_msg))
        
    def merge_values(self, msg_with_all_joints, msg_with_new_values):
        """Given a message with all the joints (got from the state)
        set the new values found in the msg_with_new_values.
        This is needed because the actual current controller needs to feeded
        with a message with the current for all joints"""
        new_limits_list = [] # As we can't update the value in a tuple (Python) we need to form a new list
        for limit_val in msg_with_all_joints.current_limits:
            new_limits_list.append(limit_val)
            
        for motor_name, limit_val in zip(msg_with_new_values.actuator_names, msg_with_new_values.current_limits):
            idx_motor = msg_with_all_joints.actuator_names.index(motor_name)
            new_limits_list[idx_motor] = limit_val
        msg_with_all_joints.current_limits = new_limits_list
        return msg_with_all_joints
        
        
if __name__ == '__main__':
    rospy.init_node('activation_manager_')
    am = activation_manager()
    rospy.spin()