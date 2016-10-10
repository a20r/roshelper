
import roshelper
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


n = roshelper.Node("test_node", anonymous=False)


@n.publisher("/test_node_string", String)
def str_pub(word):
    rospy.loginfo("Pub --> {}".format(word))
    st = String()
    st.data = word[::-1]
    return st


@n.publisher(Int64)
def int_pub(num):
    rospy.loginfo("Int Pub --> {}".format(num))
    msg = Int64()
    msg.data = num
    return msg


@n.subscriber("/test_node_string", String)
def str_sub(word):
    rospy.loginfo("Sub --> {}".format(word))


@n.subscriber("/test_node_int", Int64)
@n.subscriber("/another_test_node_int", Int64)
def int_sub(num, topic):
    rospy.loginfo("Int Sub ({}) --> {}".format(topic, num))


@n.entry_point(frequency=30)
def run():
    str_pub("balls")
    int_pub(3).publish("/test_node_int")
    int_pub(10).publish("/another_test_node_int")


@n.service("set_bool", SetBool)
def set_bool(set_bool_request):
    assert isinstance(set_bool_request, SetBoolRequest)
    return SetBoolResponse(True, "Boolean set. Inverted value: {}".format(not set_bool_request.data))


if __name__ == "__main__":
    n.start(spin=True)
