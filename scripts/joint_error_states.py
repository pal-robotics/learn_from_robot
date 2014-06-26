#! /usr/bin/env python
'''
@author: Sammy Pfeiffer


'''
import rospy
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

JOINT_ERROR_STATES_TOPIC = '/joint_error_states'
PUBLISHING_RATE = 50

class JointErrorStates():
    def __init__(self):
        self.controllers = ['head_controller', 'left_arm_controller',
                            'right_arm_controller', 'torso_controller',
                            'right_hand_controller', 'left_hand_controller'] # This could be automated"""
        
        rospy.loginfo("Subscribing to states of: " + str(self.controllers))
        self.state_subs = [] # Subscribers for the different topics
        # Create dicts for every controller, it's errors
        self.state_errors = {}
        for controller in self.controllers:
            self.state_subs.append(rospy.Subscriber('/' + controller + '/state', JointTrajectoryControllerState, 
                                                    self.state_cb, callback_args=controller))
        
        self.joint_error_states_pub = rospy.Publisher(JOINT_ERROR_STATES_TOPIC, JointState)
        rospy.sleep(2) # Give a little moment to start up, actually we should wait for every topic to hace something published...


    def state_cb(self, data, cb_args):
        # Assign to every key (controller name) the contents of the position error (and the names of the joints)
        self.state_errors.update({cb_args : [data.joint_names, data.error.positions]})
        
        
    def createJointErrorStatesFromCurrentStates(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        for controller in self.state_errors:
            joints_names, errors = self.state_errors[controller]
            js.name.extend(joints_names)
            js.position.extend(errors)
        return js
        
    def run(self):
        r = rospy.Rate(PUBLISHING_RATE)
        while not rospy.is_shutdown():
            joint_error_states = self.createJointErrorStatesFromCurrentStates()
            #print "Joint error states looks like: " + str(joint_error_states)
            self.joint_error_states_pub.publish(joint_error_states)
            r.sleep()
            
        
if __name__ == '__main__':
    rospy.init_node('pub_error_states')
    jes = JointErrorStates()
    jes.run()
        
        