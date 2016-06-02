
import rospy
import types


class MultiPublisherHelper(object):
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


class Node(object):

    def __init__(self, node_name, pyname, **kwargs):
        self.node_name = node_name
        self.kwargs = kwargs
        self.is_main = pyname == "__main__"
        self.parents = dict()
        self.m_loop = None

    def subscriber(self, topic_name, msg_type, **kwargs):
        def __decorator(func):
            def __inner(msg):
                if "self" in func.func_code.co_varnames:
                    func(self.parents[func.func_name], msg)
                else:
                    func(msg)
            args = [topic_name, msg_type, __inner]
            rospy.Subscriber(*args, **kwargs)
            return func
        return __decorator

    def publisher(self, topic_name, msg_type, **kwargs):
        def __decorator(func):
            args = [topic_name, msg_type]
            pub = rospy.Publisher(*args, **kwargs)

            def __inner(*args, **kwargs):
                msg = func(*args, **kwargs)
                pub.publish(msg)
            return __inner
        return __decorator

    def multi_publisher(self, msg_type, **kwargs):
        kw = kwargs
        topics = dict()

        def __decorator(func):

            def __inner(*args, **kwargs):
                msg = func(*args, **kwargs)
                return MultiPublisherHelper(msg, msg_type, topics, **kw)

            return __inner
        return __decorator

    def start_class(self, cl, *ar, **kw):
        rospy.init_node(self.node_name, **self.kwargs)
        freq = rospy.get_param("frequency", 3)
        rate = rospy.Rate(freq)
        nd = cl(*ar, **kw)
        for v in dir(cl):
            self.parents[v] = nd
        if not self.m_loop is None:
            while not rospy.is_shutdown():
                slf = self.parents[self.m_loop.func_name]
                args = [slf] + self.m_loop_args
                self.m_loop(*args, **self.m_loop_kwargs)
                rate.sleep()
        else:
            rospy.spin()
        return cl

    def start_func(self, cl, *ar, **kw):
        rospy.init_node(self.node_name, **self.kwargs)
        freq = rospy.get_param("frequency", 3)
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            cl(*ar, **kw)
            rate.sleep()
        return cl

    def start_node(self, *args, **kwargs):
        ar = args
        kw = kwargs

        def __inner(cl):
            if self.is_main:
                is_func = isinstance(cl, types.FunctionType)
                is_class = isinstance(cl, types.TypeType)
                if is_class:
                    self.start_class(cl, *ar, **kw)
                elif is_func:
                    self.start_func(cl, *ar, **kw)
            return cl
        return __inner

    def main_loop(self, *args, **kwargs):
        def __inner(func):
            self.m_loop = func
            self.m_loop_args = list(args)
            self.m_loop_kwargs = kwargs
            return func
        return __inner
