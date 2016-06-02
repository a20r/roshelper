
import roshelper
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64


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

    @n.publisher(Int64, queue_size=1)
    def int_pub(self, num):
        rospy.loginfo("Int Pub --> {}".format(num))
        msg = Int64()
        msg.data = num
        return msg

    @n.subscriber("/test_node_string", String, queue_size=1)
    def str_sub(self, word):
        rospy.loginfo("Sub --> {}".format(word))

    @n.subscriber("/test_node_int", Int64, queue_size=1)
    @n.subscriber("/another_test_node_int", Int64, queue_size=1)
    def int_sub(self, num):
        rospy.loginfo("Int Sub --> {}".format(num))

    @n.main_loop()
    def run(self):
        self.str_pub(self.word)
        self.int_pub(3).publish("/test_node_int")
        self.int_pub(10).publish("/another_test_node_int")
