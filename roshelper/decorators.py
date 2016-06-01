
import rospy
import time


node_dict = dict()


class HelperDecorator(object):

    def __init__(self, topic_name, msg_type, **kwargs):
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.kwargs = kwargs
        self.sub = None
        self.pub = None
        self.func = None


class subscriber(HelperDecorator):

    def __call__(self, func):
        setattr(self, func.func_name, func)
        args = [self.topic_name, self.msg_type, self.func]
        self.sub = rospy.Subscriber(*args, **self.kwargs)
        return func


class publisher(HelperDecorator):

    def __call__(self, func):
        args = [self.topic_name, self.msg_type, func]
        self.pub = rospy.Publisher(*args, **self.kwargs)
        self.func = func

        def __inner(*args, **kwargs):
            msg = self.func(*args, **kwargs)
            self.pub.publish(msg)

        return __inner


class node(object):

    def __init__(self, node_name, **kwargs):
        self.node_name = node_name
        self.kwargs = kwargs

    def __call__(self, cl):
        rospy.init_node(self.node_name, **self.kwargs)
        return cl
