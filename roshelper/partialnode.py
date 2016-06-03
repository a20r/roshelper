
import rospy
import types
import threading
from multipublisher import MultiPublisher


def start_partial_nodes(node_name, *partials, **kwargs):
    rospy.init_node(node_name, **kwargs)
    for p in partials:
        p._start(False)


class PartialNode(object):

    def __init__(self):
        self.m_loop = None
        self.thread = None
        self.cl = None
        self.subscribers = list()
        self.subscribers_init = list()

    def subscriber(self, topic_name, msg_type, **kwargs):
        if not "queue_size" in kwargs:
            kwargs["queue_size"] = 1

        def __decorator(func):
            def __inner(msg):
                if "self" in func.func_code.co_varnames:
                    return self.__class_subscriber(func, msg, topic_name)
                else:
                    return self.__function_subscriber(func, msg, topic_name)
            args = [topic_name, msg_type, __inner]
            self.subscribers.append((args, kwargs))
            return func
        return __decorator

    def publisher(self, *upper_args, **kwargs):
        if not "queue_size" in kwargs:
            kwargs["queue_size"] = 1
        if isinstance(upper_args[0], str):
            topic_name, msg_type = upper_args

            def __decorator(func):
                args = [topic_name, msg_type]
                pub = rospy.Publisher(*args, **kwargs)

                def __inner(*args, **kwargs):
                    msg = func(*args, **kwargs)
                    pub.publish(msg)
                    return msg
                return __inner
            return __decorator
        elif isinstance(upper_args[0], types.TypeType):
            return self.__multi_publisher(upper_args[0], **kwargs)

    def entry_point(self, *args, **kwargs):
        self.cl_args = args
        self.cl_kwargs = kwargs

        def __inner(cl):
            self.cl = cl
            return cl
        return __inner

    def main_loop(self, *args, **kwargs):
        def __inner(func):
            self.m_loop = func
            self.m_loop_args = list(args)
            self.m_loop_kwargs = kwargs
            return func
        return __inner

    def __multi_publisher(self, msg_type, **kwargs):
        kw = kwargs
        topics = dict()

        def __decorator(func):

            def __inner(*args, **kwargs):
                msg = func(*args, **kwargs)
                return MultiPublisher(msg, msg_type, topics, **kw)

            return __inner
        return __decorator

    def __class_subscriber(self, func, msg, topic_name):
        if func.func_code.co_argcount == 2:
            return func(self.slf, msg)
        elif func.func_code.co_argcount == 3:
            return func(self.slf, msg, topic_name)

    def __function_subscriber(self, func, msg, topic_name):
        if func.func_code.co_argcount == 1:
            return func(msg)
        elif func.func_code.co_argcount == 2:
            return func(msg, topic_name)

    def _start(self, spin):
        for args, kwargs in self.subscribers:
            self.subscribers_init.append(rospy.Subscriber(*args, **kwargs))
        is_func = isinstance(self.cl, types.FunctionType)
        is_class = isinstance(self.cl, types.TypeType)
        if is_class:
            targ = self.__start_class
        elif is_func:
            targ = self.__start_func
        self.thread = threading.Thread(target=targ,
                                       args=(self.cl,) + self.cl_args,
                                       kwargs=self.cl_kwargs)
        self.thread.daemon = True
        self.thread.start()
        if spin:
            rospy.spin()
        return self

    def __start_class(self, cl, *ar, **kw):
        vrs = cl.__init__.func_code.co_varnames
        class_args = list()
        for v in vrs:
            if not v == "self":
                arg_tuple = kw.get(v)
                is_tuple = isinstance(arg_tuple, types.TupleType)
                if arg_tuple is None or len(arg_tuple) == 0:
                    default_var = None
                    scope = "~"
                elif not is_tuple or len(arg_tuple) == 1:
                    default_var = arg_tuple[0]
                    scope = "~"
                else:
                    scope, default_var = arg_tuple
                arg = rospy.get_param(scope + v, default_var)
                class_args.append(arg)
        rate = self.__get_rate(self.m_loop_kwargs)
        nd = cl(*class_args)
        self.slf = nd
        if not self.m_loop is None:
            while not rospy.is_shutdown():
                args = [self.slf] + self.m_loop_args
                self.m_loop(*args, **self.m_loop_kwargs)
                rate.sleep()
        return cl

    def __start_func(self, cl, *ar, **kw):
        rate = self.__get_rate(kw)
        while not rospy.is_shutdown():
            cl(*ar, **kw)
            rate.sleep()
        return cl

    def __get_rate(self, kw):
        if "frequency" in kw:
            freq = kw["frequency"]
            del kw["frequency"]
        else:
            freq = "frequency"
        if isinstance(freq, str):
            def_freq = kw.get("default_frequency", 30)
            freq = rospy.get_param(freq, def_freq)
        if "default_frequency" in kw:
            del kw["default_frequency"]
        return rospy.Rate(freq)
