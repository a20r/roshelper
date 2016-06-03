# roshelper

A Python library that makes using `rospy` less painful. `roshelper` provides a
variety of helper function and decorators that makes developing ROS nodes in
Python much easier.

```python

import roshelper


n = roshelper.Node("talker_and_listener", anonymous=False)


@n.publisher("/chatter", String, queue_size=1)
def talker(word):
    pass

```
