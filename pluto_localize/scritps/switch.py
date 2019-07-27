#!/usr/bin/env python
import rospy
import roslib
from geometry_msgs.msg import Point
import tf
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Float64

class get_pose():

	def __init__(self):
		rospy.init_node('get_pose',anonymous=False)
		self.aruco_marker = {}
		self.cam_pose = Point()
		self.posepub = rospy.Publisher('/statespecs/pose', Point, queue_size=10)

		rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.aruco_data)	# Subscribing to topic
		rospy.Subscriber('setspecs/id',MarkerArray,self.get_aruco_id)	# Subscribing to topic

		self.id = 12
		self.count = 0

	# Callback for aruco marker information
	def aruco_data(self, msg):
		for i in range(0,len(msg.markers)):
			aruco_id = msg.markers[i].id
			pose_x = -round(msg.markers[i].pose.pose.position.x,3)*30
			pose_y = -round(msg.markers[i].pose.pose.position.y,3)*30
			pose_z = -round(msg.markers[i].pose.pose.position.z,3)*30
			self.aruco_marker[aruco_id] = [pose_x,pose_y,pose_z]

		try:
			self.cam_pose.x = self.aruco_marker[self.id][0]
			self.cam_pose.y = self.aruco_marker[self.id][1]
			self.cam_pose.z = self.aruco_marker[self.id][2]
			self.posepub.publish(self.cam_pose)

		except:
			print "Next target is not visible"
			""" Somehow call aruco_map to get the next target """

		# print "\n"
		# print "ArUco_ID: ",self.id, "\r"
		# self.changeid()

		#Callback to get the current aruco ID to localize
	def get_aruco_id(self, msg):
		self.id = msg.data

	def changeid(self):
		if (self.count >900):
			self.count = 0
		if (self.count == 300):
			self.id = 256
		elif (self.count == 600):
			self.id = 320
		elif (self.count == 900):
			self.id = 0

		self.count = self.count + 1


if __name__=="__main__":

	marker = get_pose()
	while not rospy.is_shutdown():
		rospy.spin()
