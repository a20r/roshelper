
import roshelper
import rospy
from std_msgs.msg import String


n = roshelper.Node("talker_and_listener", anonymous=False)


@n.publisher("/chatter", String, queue_size=1)
def talker():
    hello_str = "hello world %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    return hello_str


@n.subscriber("/chatter", String)
def listener(word):
    rospy.loginfo(rospy.get_caller_id() + " : I heard %s" % word.data)


@n.entry_point(frequency=30)
def main():
    talker()


if __name__ == "__main__":
    n.start(spin=True)
