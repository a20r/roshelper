# RosHelper

A Python library that makes using `rospy` a bit easier. `roshelper` provides a
variety of helper function and decorators that makes developing ROS nodes in
Python much easier.

Contents:
- Quick Start
- How Do I Get It?
- Where Do I Go From Here?

## Quick Start

Below is a reimplementation of the [beginners
tutorial](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber)
using `roshelper`. 

```python
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
    pstr = rospy.get_caller_id() + " : I heard %s" % word.data
    rospy.loginfo(p_str)


@n.entry_point(frequency=30)
def main():
    talker()


if __name__ == "__main__":
    n.start(spin=True)
```

### Breakdown

First the necessary libraries are imported and a `roshelper` node is
instantiated.

```python
import roshelper
import rospy
from std_msgs.msg import String


n = roshelper.Node("talker_and_listener", anonymous=False)
```

Now using the node, we can create a publisher. This function will publish the
string, "hello world", with the current time to the "/chatter" topic. The
`publisher` decorator is used to capture the return value from the `talker`
function and publish it to the "/chatter" topic.

```python
@n.publisher("/chatter", String, queue_size=1)
def talker():
    hello_str = "hello world %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    return hello_str
```

Similarly, we can use the `subscriber` decorator to subscribe to the "/chatter"
topic. Whenever a message is received on the "/chatter" topic, the `listener`
function is called with the message. In this example `listener` is logging the
string heard over the "/chatter" topic along with the caller id.

```python
@n.subscriber("/chatter", String)
def listener(word):
    pstr = rospy.get_caller_id() + " : I heard %s" % word.data
    rospy.loginfo(p_str)
```

In order to run the node that you have been implementing, we need to define an
entry point. This entry point will be the function that runs on a continuous
loop at a given frequency. The `entry_point` decorator attached to the `main`
function, will call `main` in a loop with a frequency of 30 Hz. In this
example, `main` is just using `talker` to publish to the "/chatter" topic.

```python
@n.entry_point(frequency=30)
def main():
    talker()
```

At last, you are ready to run your node. The `start` method will just run a new
thread initializing the ROS node and calling the function you marked as the
entry point. The `spin=True` flag is used to tell `roshelper` that you would
like continue running the node on the main thread. This is equivalent to
calling `rospy.spin()` after calling `start`.

```python
if __name__ == "__main__":
    n.start(spin=True)
```

If none of this just made sense, then you should go read more about
[ROS](http://ros.org), [rospy](http://wiki.ros.org/rospy), and / or [Python
decorators](https://realpython.com/blog/python/primer-on-python-decorators/)

## How Do I Get It?

Glad you asked. First you need to [install
ROS](http://wiki.ros.org/indigo/Installation/Ubuntu). Once you have done that,
just install `roshelper` with pip.

```bash
$ pip install roshelper
```

## Where Do I Go From Here?

Please look at the contents in the sidebar for detailed explanations of how the
different components of `roshelper` work. Also, look these
[examples](https://github.com/wallarelvo/roshelper/tree/master/examples).
