#!/usr/bin/env python

from threading import Lock
import numpy
import sys
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import sensor_msgs.msg
import tf
# from movefranka_joint import MoveGroupPythonIntefaceTutorial as movefranka

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]
    return t


class MoveArm(object):

    def __init__(self):
        print "Motion Planning Initializing..."
        # Prepare the mutex for synchronization
        self.mutex = Lock()

        # Initialize variables
        self.joint_state = sensor_msgs.msg.JointState()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        # rospy.wait_for_service("check_state_validity")
        # self.state_valid_service = rospy.ServiceProxy('check_state_validity', moveit_msgs.srv.GetStateValidity)
        # print "State validity service ready"

        # Initialize MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        print "MoveIt! interface ready"

        self.current_robot_state = self.robot.get_current_state()

    def go_to_joint_state(self, joint_goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        # joint_goal = group.get_current_joint_values()
        # joint_goal[0] = 0
        # joint_goal[1] = -pi/4
        # joint_goal[2] = 0
        # joint_goal[3] = -pi/2
        # joint_goal[4] = 0
        # joint_goal[5] = pi/3
        # joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        plan = group.plan()
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        # group.stop()
        self.execute_plan(plan)

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        # current_joints = self.group.get_current_joint_values()
        # return all_close(joint_goal, current_joints, 0.01)

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        group.execute(plan, wait=True)

    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state = self.joint_state
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = "panda_link0"
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        # q = []
        if res.error_code.val == res.error_code.SUCCESS:
            print '###################'
            print 'res.solution.joint_state', res.solution.joint_state.position
            print '666666666^^^^^^'
            return res.solution.joint_state.position
        else:
            print 'IK Failed'


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()

    Ttrans = tf.transformations.translation_matrix((-0.3, 0.2, 0.9))
    Rtrans = tf.transformations.rotation_matrix(1.57, (0, 1, 0))
    T = numpy.dot(Ttrans, Rtrans)
    # mf = movefranka()
    posi = ma.IK(T)
    # posi = list([posi[0], posi[1], posi[2], posi[3], posi[4], posi[5], posi[6]])
    posi = list([0., posi[1], posi[2], posi[3], posi[4], posi[5], posi[6]])

    ma.go_to_joint_state(posi)
    rospy.sleep(2)
    print '############# current robot state ####### \n', ma.current_robot_state.joint_state.position
    print '$$$$$$$$$'
    rospy.spin()
