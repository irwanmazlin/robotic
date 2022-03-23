import rospy
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import ObjectCount
import sort
import message_filters
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

bridge = CvBridge()

bbox_checkin = 0


# def count(msg):


# 	if(msg.count > 0):
# 		detectioncallback(msg, bbox_msg)


	
# 	else:
# 		something()

	


# 	# print("message", msg.count)


# def something():


def detectioncallback(msg, bbox_msg):
	# global detections
	# global trackers
	# global track

	detections = []
	trackers = []
	track = []
	

	# print(bbox_msg.bounding_boxes)


	# print("info bbox",  bbox_msg.bounding_boxes)

	for box in bbox_msg.bounding_boxes:
		# print(box.Class)
		label = box.Class
		conf = box.probability

		detections.append(np.array([box.xmin, box.ymin, box.xmax, box.ymax, conf]))

	detections = np.array(detections)
	print(detections)
	# declare tracker
	bbox_checkin = 1

	trackers = tracker.update(detections)
	trackers = np.array(trackers, dtype= 'int')
	track = trackers

	# rb = BoundingBox()
	# print(rb)

	somethingcount = count(trackers)

	# print(bbox_checkin)
	img = bridge.imgmsg_to_cv2(msg, "bgr8")
	print(bbox_checkin)



	for i in track:
		print(i)

		# rb = BoundingBox()
		# print(rb)

	
	cv2.imshow("tracking", img)
	cv2.waitKey(3)



# def imagecallback(msg):
	
# 	# r = BoundingBox()
# 	# print(r)



# 	# for tracker in track:

# 	# 	print("number of tracked inside loop : " + str(len(track)))


	

# 	# 	cv2.rectangle(img, (tracker[0], tracker[1]), (tracker[2], tracker[3]), (0, 255, 255), 1)

# 	# 	cv2.putText(img, str(tracker[4]), (tracker[2], tracker[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), lineType=cv2.LINE_AA)

# 	# print(str(track[0][4]))
# 	print("number of tracked object : " + str(len(track)))
# 	cv2.imshow("tracking", img)
# 	cv2.waitKey(3)


def main():
	global tracker


	while not rospy.is_shutdown():
		rospy.init_node("node track started", anonymous = True)


		tracker = sort.Sort()

		img_sub = message_filters.Subscriber('/camera/color/image_raw', Image)

		det_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)


		obj_count = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, count)

		publishtrack = rospy.Publisher('/tracker/image_result',
                                   Image,
                                   queue_size=1)
		sync = message_filters.ApproximateTimeSynchronizer([img_sub, det_sub], queue_size=10,slop=100000000)
		sync.registerCallback(detectioncallback)

		rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass