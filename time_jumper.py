#!/usr/bin/env python

import rospy
import rosbag
from tf2_msgs.msg import TFMessage
import sensor_msgs.msg


def cb_1(msg):
    global first_stamp, now
    for i, tf in enumerate(msg.transforms):
        print i
        if first_stamp is None:
            first_stamp = tf.header.stamp
        tf.header.stamp -= first_stamp
        tf.header.stamp += now
    pub_1.publish(msg)


rospy.init_node('hoge')
first_stamp = None
print 'hi'
rospy.sleep(1)
now = rospy.Time.now()

pub_1 = rospy.Publisher('/tf', TFMessage, queue_size=1)
while not rospy.is_shutdown():
    try:
        sub_1 = rospy.Subscriber('/tf', TFMessage, cb_1)
        # print 'there'
    except rospy.ROSInterruptException:
        pass

#########################################################
# METHOD 2:
##########################################################
bag = rosbag.Bag('/home/automato/Ash/scripts/ceiling_cam_qhd_points.bag')
rospy.init_node('rosbag_filter', anonymous=True)
pubber = rospy.Publisher("/ceiling_camera/qhd/points", sensor_msgs.msg.PointCloud2, queue_size=1)
time_now = rospy.Time.now()
time_prev = rospy.Time()
first_stamp = None

# i = 0
while not rospy.is_shutdown():
    bag = rosbag.Bag('/home/automato/Ash/scripts/ceiling_cam_qhd_points.bag')
    for topic, msg, t in bag.read_messages(topics = '/ceiling_camera/qhd/points'):
        # msg = copy.deepcopy(org_msg)
        # if time_prev.secs == 0:
        #   d_time = rospy.Duration()
        #   time_prev = copy.deepcopy(msg.header.stamp)
        #   msg.header.stamp = copy.deepcopy(time_now)
        # else:
        #   d_time = msg.header.stamp - time_prev
        #   time_now += d_time
        #   time_prev = copy.deepcopy(msg.header.stamp)
        #   msg.header.stamp = copy.deepcopy(time_now)
        # # rospy.loginfo(msg.header.stamp.secs)
        # rospy.sleep(d_time)
        if first_stamp is None:
            first_stamp = msg.header.stamp
        msg.header.stamp -= first_stamp
        msg.header.stamp += time_now
    # pub_1.publish(msg)
        try:
            pubber.publish(msg)
        except:
            break
        else:
            pass
    bag.close()

