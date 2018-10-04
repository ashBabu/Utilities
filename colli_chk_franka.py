#!/usr/bin/env python

# This program checks if any of th joints is in collision

from threading import Lock
import sys
import moveit_commander
import moveit_msgs.srv
import rospy
from copy import deepcopy


class MoveArm(object):

    def __init__(self):
        # Prepare the mutex for synchronization
        self.mutex = Lock()

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity', moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # Initialize MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.group_name = "panda_arm"
        self.curr_jnt_state = self.robot.get_current_state()

    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        # current_joint_state = deepcopy(self.joint_state)
        # current_joint_state.position = list(current_joint_state.position)
        # self.joint_state_from_q(current_joint_state, q)
        
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state = q.joint_state
        res = self.state_valid_service(req)
        # print res.valid
        return res.valid
        ################################################################


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    while not rospy.is_shutdown():
        ma = MoveArm()
        # print "============ Printing robot state"
        # print ma.curr_jnt_state.joint_state.position
        # print ""
        joint_state_to_chk = deepcopy(ma.curr_jnt_state)
        # print '############### joint_state to chk', joint_state_to_chk
        # print '#######################'
        joint_state_to_chk.joint_state.position = list([0.5, 2.0, -5.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0])
        # print '############### joint_state to chkUPDATE     ', joint_state_to_chk
        print '#######################'
        print 'Ash State validity', ma.is_state_valid(joint_state_to_chk)
        print '%%%%%%%%%%%%%%%%%%%%%%%%'
#!/usr/bin/env python

# This program checks if any of th joints is in collision

from threading import Lock
import sys
import moveit_commander
import moveit_msgs.srv
import rospy
from copy import deepcopy


class MoveArm(object):

    def __init__(self):
        # Prepare the mutex for synchronization
        self.mutex = Lock()

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity', moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # Initialize MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.group_name = "panda_arm"
        self.curr_jnt_state = self.robot.get_current_state()

    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        # current_joint_state = deepcopy(self.joint_state)
        # current_joint_state.position = list(current_joint_state.position)
        # self.joint_state_from_q(current_joint_state, q)
        
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state = q.joint_state
        res = self.state_valid_service(req)
        # print res.valid
        return res.valid
        ################################################################


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    while not rospy.is_shutdown():
        ma = MoveArm()
        # print "============ Printing robot state"
        # print ma.curr_jnt_state.joint_state.position
        # print ""
        joint_state_to_chk = deepcopy(ma.curr_jnt_state)
        # print '############### joint_state to chk', joint_state_to_chk
        # print '#######################'
        joint_state_to_chk.joint_state.position = list([0.5, 2.0, -5.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0])
        # print '############### joint_state to chkUPDATE     ', joint_state_to_chk
        print '#######################'
        print 'Ash State validity', ma.is_state_valid(joint_state_to_chk)
        print '%%%%%%%%%%%%%%%%%%%%%%%%'
