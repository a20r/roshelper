
import roshelper
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64


n = roshelper.Node("test_node", __name__, anonymous=False)


@n.publisher("/test_node_string", String, queue_size=1)
def str_pub(word):
    rospy.loginfo("Pub --> {}".format(word))
    st = String()
    st.data = word[::-1]
    return st


@n.multi_publisher(Int64, queue_size=1)
def int_pub(num):
    rospy.loginfo("Int Pub --> {}".format(num))
    msg = Int64()
    msg.data = num
    return msg


@n.subscriber("/test_node_string", String, queue_size=1)
def str_sub(word):
    rospy.loginfo("Sub --> {}".format(word))


@n.subscriber("/test_node_int", Int64, queue_size=1)
@n.subscriber("/another_test_node_int", Int64, queue_size=1)
def int_sub(num):
    rospy.loginfo("Int Sub --> {}".format(num))


@n.start_node()
def run():
    str_pub("balls")
    int_pub(3).publish("/test_node_int")
    int_pub(10).publish("/another_test_node_int")
