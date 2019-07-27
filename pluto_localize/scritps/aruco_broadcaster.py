#!/usr/bin/env python
import rospy

 # Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from aruco_msgs.msg import MarkerArray


class get_pose():

    def __init__(self):
        rospy.init_node('tf_pose',anonymous=False)
        self.aruco_marker = {}

        rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.handle_aruco_pose)    # Subscribing to topic
        rospy.Subscriber('setspecs/id',MarkerArray,self.get_aruco_id)    # Subscribing to topic

        self.id = 0
        self.count = 0

    # Callback for aruco marker information
    def aruco_data(self, msg):

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

    """
    This function will get the aruco information of all marker
    and then it will store them in a dictionary with ID as key
    Once the dictionary is ready it will get the appropriate ID
    and publish its coordinates to the tf2 after appropriate
    scaling
    """
    def handle_aruco_pose(self, msg):
        for i in range(0,len(msg.markers)):
            aruco_id = msg.markers[i].id
            pose_x = -round(msg.markers[i].pose.pose.position.x,3)    # Negative sign to reverse the direction of vector
            pose_y = -round(msg.markers[i].pose.pose.position.y,3)    # Aruco gives vector from camera to marker, we reverse it
            pose_z = -round(msg.markers[i].pose.pose.position.z,3)    # To get vector from marker to camera

            self.aruco_marker[aruco_id] = [pose_x,pose_y,pose_z]     # Dictionary is a ready after this loop

        # Declare a tf2 broadcaster and an object
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        # Assigning the values to the object of geometry_msgs
        t.header.stamp = rospy.Time.now()
        # Setting the parent frame, we have coordinates from aruco to drone, thus aruco is the parent frame
        t.header.frame_id = "aruco" + str(self.id)   # Name is aruco + its ID, for example for ID 0, frame name would be aruco0
        t.child_frame_id = "drone"  # child is always drone
        # Setting translation values
        """
        Axis will be flipped here as x axis in aruco is y in Vrep
        z axis in aruco is x in Vrep
        y axis in aruco is z in Vrep
        """
        t.transform.translation.x = self.aruco_marker[self.id][2]/1.9    # 1.9 is scale in Z axis in Vrep to aruco conversion for some reason
        t.transform.translation.y = self.aruco_marker[self.id][0]   # x axis
        t.transform.translation.z = self.aruco_marker[self.id][1]   # y axis
        """
        No need to change this for every ID since it will remain same
        as long as the wall on which markers are doesn't change
        If we decide to implement that,
        then these will have to be handled properly
        """
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = msg.markers[0].pose.pose.orientation.x
        t.transform.rotation.y = msg.markers[0].pose.pose.orientation.y
        t.transform.rotation.z = msg.markers[0].pose.pose.orientation.z
        t.transform.rotation.w = msg.markers[0].pose.pose.orientation.w
        br.sendTransform(t)



if __name__=="__main__":
    marker = get_pose()
    while not rospy.is_shutdown():
        rospy.spin()
