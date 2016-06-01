
# import roshelper
import rospy
from std_msgs.msg import String


# @roshelper.node("test_node", anonymous=False)
class TestNode(object):

    # @roshelper.publisher("/test_node_string", String, queue_size=1)
    def str_pub(self, word):
        rospy.loginfo("Pub --> {}".format(word))
        st = String()
        st.data = word[::-1]
        return st

    # @roshelper.subscriber("/test_node_string", String, queue_size=1)
    def str_sub(self, word):
        rospy.loginfo("Sub --> {}".format(word))

    def run(self):
        self.str_pub("hello")


if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=False)
    tn = TestNode()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        tn.run()
        print "here"
        rate.sleep()
