import rospy
from franka_gripper.msg import MoveAction
import moveit_commander
import sys
import actionlib
from franka_gripper.msg import MoveActionGoal     # imported to populate goal with the correct syntax


def move_action(w, s):
    client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
    client.wait_for_server()
    goal = MoveActionGoal
    goal.width = w
    goal.speed = s
    client.send_goal(goal)
    print "#### moving gripper to set width=", goal.width, "set speed=", goal.speed
    move_done = client.wait_for_result()


def go_to_joint_state(joint_goal):
    group = moveit_commander.MoveGroupCommander("panda_arm")
    print group.get_current_joint_values()
    group.set_max_velocity_scaling_factor(0.15)  # scaling down velocity
    group.set_max_acceleration_scaling_factor(0.10)  # scaling down velocity
    group.go(joint_goal, wait=True)
    group.stop()


def main():
    # robot = Robot()
    # robot.go_home()
    # rospy.spin()
    moveit_commander.MoveGroupCommander("hand")
    # gripper = GripperClient()
    move_action(0.01, 0.01)
    rospy.sleep(0.3)
    move_action(0.02, 0.01)
    rospy.sleep(0.3)
    move_action(0.03, 0.01)
    rospy.sleep(0.3)
    move_action(0.01, 0.01)


if __name__ == '__main__':
    rospy.init_node('Move_Robot')
    pose_needed = [0.01822114529473803, -0.03469482581261627, -0.25800041619936626, -2.003761352664521, 0.01522025576979748, 1.9771611754452738, 0.6377279893875695]
    moveit_commander.roscpp_initialize(sys.argv)
    go_to_joint_state(pose_needed)
    try:
        main()
    except Exception as e:
        print e
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)