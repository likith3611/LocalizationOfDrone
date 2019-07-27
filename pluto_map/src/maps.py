from collections import defaultdict

# Class for definition of the graph
class Graph():
    def __init__(self):
        """
        self.edges is a dict of all possible paths between all aruco markers
        e.g. {'320': ['0', '1', '5', '7'], ...}
        self.weights has all the real world vectors between any two aruco markers,
        with the two aruco IDs as a tuple as the key
        e.g. {('320', '256'): (7,3,-2)}
        """
        self.edges = defaultdict(list)
        self.orientation = defaultdict(list)    # Used to store relative orientations
        self.weights = {}


    def add_edge(self, from_node, to_node, weight, orient):
        # Note: assumes edges are bi-directional, as drone can go anywhere
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.weights[(from_node, to_node)] = weight
        self.weights[(to_node, from_node)] = [ -x for x in weight]
        self.orientation[from_node].append((to_node,orient))
        print self.weights

    # Function to check if a path is known between two aruco IDs x and y
    def isconnected (self, x, y):
        if (x,y) in self.weights:
            return True
        else:
            return False;

        # Function to fetch the path (Vector) from one aruco ID x to another(y)
    def get_path(self, x, y):
        if (x,y) in self.weights:
            return self.weights[(x,y)]
        else:
            print ("Path to that particular Aruco Marker Unknown")

