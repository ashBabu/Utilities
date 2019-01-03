import rospy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

# global first_stamp, now
# first_stamp = None
# now = rospy.Time.now()


class latch_pointCloud(object):

    def __init__(self):
        rospy.init_node('latch_pointCloud', anonymous=True)
        rospy.Subscriber('/ceiling_camera/qhd/points_old', PointCloud2, self.cb_1)
        self.pub_1 = rospy.Publisher('/ceiling_camera/qhd/points', PointCloud2, queue_size=1, latch=True)
        self.first_stamp = None
        self.now = rospy.Time.now()
        # print 'hi'

    def cb_1(self, msg):
        # global first_stamp, now
        print msg.height
        print 'hi'
        self.pub_1.publish(msg)
        print 'hiA'

        ###########  Time Jumper #################
        # for i, tf in enumerate(msg.transforms):
        #     print i
        #     if self.first_stamp is None:
        #         self.first_stamp = tf.header.stamp
        #     tf.header.stamp -= self.first_stamp
        #     tf.header.stamp += self.now
        # self.pub_1.publish(msg)
