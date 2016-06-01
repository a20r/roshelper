#!/usr/bin/env python

import roshelper
import rospy
from std_msgs.msg import String


n = roshelper.Node("test_node", __name__, anonymous=False)


@n.start_node(word="balls")
class TestNode(object):

    def __init__(self, word):
        self.word = word

    @n.publisher("/test_node_string", String, queue_size=1)
    def str_pub(self, word):
        rospy.loginfo("Pub --> {}".format(word))
        st = String()
        st.data = word[::-1]
        return st

    @n.subscriber("/test_node_string", String, queue_size=1)
    def str_sub(self, word):
        rospy.loginfo("Sub --> {}".format(word))

    @n.main_loop()
    def run(self):
        self.str_pub(self.word)
