
import rospy


class MultiPublisher(object):

    def __init__(self, msg, msg_type, topics, **kwargs):
        self.msg = msg
        self.msg_type = msg_type
        self.topics = topics
        self.kwargs = kwargs

    def publish(self, topic):
        if not topic in self.topics:
            args = [topic, self.msg_type]
            self.topics[topic] = rospy.Publisher(*args, **self.kwargs)
        self.topics[topic].publish(self.msg)
        return self.msg

    def msg(self):
        return self.msg
