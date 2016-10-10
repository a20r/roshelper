
import roshelper
import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

message = "hello world"

n = roshelper.Node("talker_and_listener", anonymous=False)


@n.publisher("/chatter", String, queue_size=1)
def talker():
    hello_str = message + " %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    return hello_str


@n.subscriber("/chatter", String)
def listener(word):
    rospy.loginfo(rospy.get_caller_id() + " : I heard %s" % word.data)


@n.entry_point(frequency=30)
def main():
    talker()


@n.service("use_uppercase", SetBool)
def use_uppercase(request):
    assert isinstance(request, SetBoolRequest)
    global message  # Ugh, globals, this better be a class, but that would be slightly less simple code
    if request.data:
        message = message.upper()
    else:
        message = message.lower()
    return SetBoolResponse(True, "Boolean set. New message value: {}".format(message))


if __name__ == "__main__":
    n.start(spin=True)
