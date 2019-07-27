#!/usr/bin/env python
import rospy
import roslib
from aruco_msgs.msg import MarkerArray

"Import a cusomized graph designed to store aruco marker and paths in between them"
from maps import Graph    # For python3 from .aruco_maps imort Graph

# Make a graph to store the information on Aruco markers
graph = Graph()


# Class containing the functions needed to map the aruco markers to the graph
class Map():

    def __init__(self):
        rospy.init_node("aruco_mapping")

        # Dictionary for storing currently visible aruco markers
        self.aruco_marker = {}

        # Subscribers
        rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.get_markers)  # Subscriber to get the marker information from aruco library

    # Callback function to fetch the current camera poses with respect to different markers
    def get_markers(self, msg):
        for i in range(0,len(msg.markers)):
            aruco_id = msg.markers[i].id

            pose_x = round(msg.markers[i].pose.pose.position.x,3)*10
            pose_y = round(msg.markers[i].pose.pose.position.y,3)*10
            pose_z = round(msg.markers[i].pose.pose.position.z,3)*10
            self.aruco_marker[aruco_id] = [pose_x,pose_y,pose_z]
            self.create_graph()

            """
            Now all the currently visible aruco markers are stored in self.aruco_marker_publisher
            Now we iterate over them in order to get a bipartite graph (every pair of node)
            Now we have three cases
                - If both the aruco ids are same (self loop)
                - If a edge already exists
                for these two cases we break from the inner loop
            For the third case we add an edge
            """
    def create_graph(self):
        for i in self.aruco_marker:
            for j in self.aruco_marker:
                if (i == j):
                    break
                if (graph.isconnected(i, j)):
                    break
                else:
                    print "Adding edge between Aruco marker: ", i, " and Aruco marker : ", j
                    graph.add_edge(i, j, self.get_relative_vector(i, j), 0)

    # Function to calculate the vector between two aruco markers given, their poses wrt to Camera
    def get_relative_vector(self, x, y):
        relative_vector = []    # Array to store the relatve vector
        for i, j in zip(self.aruco_marker[x], self.aruco_marker[y]):
            relative_vector.append(i-j)    # Vector Addition Law
        return relative_vector


if __name__=="__main__":
    while not rospy.is_shutdown():
        mp = Map()
        rospy.spin()
