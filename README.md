# roshelper

A helper library for rospy. `roshelper` provides a variety of helper function
and decorators to make developing ROS nodes in Python much easier.

# Using standard rospy
```python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64


str_pub = rospy.Publisher("/test_node_string", String, queue_size=1)
int_pub_1 = rospy.Publisher("/test_node_int", Int64, queue_size=1)
int_pub_2 = rospy.Publisher("/another_test_node_int", Int64, queue_size=1)


def get_int_sub(topic_name):
    def int_sub(num):
        rospy.loginfo("Int Sub ({}) --> {}".format(topic_name, num))


def str_sub_callback(word):
    rospy.loginfo("Sub --> {}".format(word))


str_sub = rospy.Subscriber("/test_node_string", String, str_sub_callback,
                           queue_size=1)

int_sub_1 = rospy.Subscriber(
    "/test_node_int", Int64,
    get_int_sub("/test_node_int"), queue_size=1)


int_sub_2 = rospy.Subscriber(
    "/another_test_node_int", Int64,
    get_int_sub("/another_test_node_int"), queue_size=1)


def publish_int(num, pub):
    rospy.loginfo("Int Pub --> {}".format(num))
    msg = Int64()
    msg.data = num
    pub.publish(msg)


def publish_str(word):
    rospy.loginfo("Pub --> {}".format(word))
    st = String()
    st.data = word[::-1]
    str_pub.publish(st)


if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=False)
    freq = rospy.get_param("frequency", 30)
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        publish_str("balls")
        publish_int(3, int_pub_1)
        publish_int(3, int_pub_2)
        rate.sleep()
```

# Using roshelper
```
import roshelper
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64


n = roshelper.Node("test_node", __name__, anonymous=False)


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


@n.start_node(frequency="frequency", default_frequency=30)
def run():
    str_pub("balls")
    int_pub(3).publish("/test_node_int")
    int_pub(10).publish("/another_test_node_int")
```
