import rosbag
import tf
import rospy
import sensor_msgs.msg
import copy

# listener = tf.TransformListener()
bag = rosbag.Bag('/home/ash/filtered_cloud1.bag')
# for topic, msg, t in bag.read_messages(topics = '/tf'):
rospy.init_node('rosbag_filter', anonymous=True)
pubber = rospy.Publisher("/move_group/filtered_cloud", sensor_msgs.msg.PointCloud2, queue_size=5)
time_now = rospy.get_rostime()
time_prev = rospy.Time()
# i = 0
while not rospy.is_shutdown():
	# i += 1
	# rospy.loginfo(i)
	bag = rosbag.Bag('/home/ash/filtered_cloud1.bag')
	for topic, org_msg, t in bag.read_messages(topics = '/move_group/filtered_cloud'):
		msg = copy.deepcopy(org_msg)
		if time_prev.secs == 0:
			d_time = rospy.Duration()
			time_prev = copy.deepcopy(msg.header.stamp)
			msg.header.stamp = copy.deepcopy(time_now)
		else:
			d_time = msg.header.stamp - time_prev
			time_now += d_time
			time_prev = copy.deepcopy(msg.header.stamp)
			msg.header.stamp = copy.deepcopy(time_now)
		# rospy.loginfo(msg.header.stamp.secs)
		rospy.sleep(d_time)
		try:
			pubber.publish(msg)
		except:
			break
		else:
			pass
	bag.close()
