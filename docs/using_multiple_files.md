# Using Multiple Files

`roshelper` provides support of using multiple files to design and organize ROS
nodes. In order to design nodes over multiple files, instead of instantiating a
`Node` object, you can specify a `PartialNode` object.

## Example

```python
### subs.py ###

import roshelper
import rospy
from std_msgs.msg import Int64

n = roshelper.PartialNode()

@n.subscriber("/sum")
def subscribe_sum():
    rospy.loginfo("Num: {}".format(num))


### pubs.py ###

import roshelper
from std_msgs.msg import Int64

n = roshelper.PartialNode()

@n.publisher("/sum", Int64)
def publish_sum(a, b, c):
    num = Int64()
    num.data = a + b + c
    return num


### run.py ###

import roshelper
import rospy
import subs
import pubs

n = roshelper.PartialNode()

@n.entry_point(frequency=30)
def run():
    msg = subs.publish(1, 2, 3)
    rospy.loginfo("Publishing: {}".format(msg))

if __name__ == "__main__":
    roshelper.start_partial_nodes("summer", subs.n, pubs.n, n)
    rospy.spin()
```

In this example, three files are used to design a ROS node. The subs.py file
defines a subscriber, the pubs.py file defines a publisher, and the run.py file
declares the entry point of the node. Also, in run.py, `start_partial_nodes` is
called with the name of the node, and all the partial nodes that will be joined
into one ROS node. This allows you to design ROS nodes over multiple files to
produce well organized code.

## Important Note

The `start_partial_nodes` function also takes keyword arguments such as
`anonymous` which is used by the underlying rospy `init_node`.
