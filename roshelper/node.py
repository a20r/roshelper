
import rospy
import types
import threading
from partialnode import PartialNode


class Node(PartialNode):

    def __init__(self, node_name, **kwargs):
        PartialNode.__init__(self)
        self.node_name = node_name
        self.kwargs = kwargs

    def start(self, spin=False):
        rospy.init_node(self.node_name, **self.kwargs)
        return self._start(spin)
