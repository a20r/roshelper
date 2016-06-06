# Subscribers

The `subscriber` decorator allows you to automatically subscribe and specify a
callback for a given ROS topic. You can also use a single function to subscribe
to multiple topics, something that was not trivial in `rospy`.

## Subscribing To A Single Topic

By adding the `subscriber` decorator to a function, you are able to specify
that function as the callback for a topic. This is shown below.

```python
import roshelper
import rospy
from std_msgs.msg import Int64

n = roshelper.Node("summer_listener")

@n.subscriber("/sum", Int64)
def subscribe_sum(num):
    rospy.loginfo("Num: {}".format(num))

...
```

In this simple example, the `subscribe_sum` function is called whenever a
message is received on the "/sum" topic. This function will then log the number
on the info console.

## Subscribing to Multiple Topics

You can subscribe to multiple topics by simply adding multiple `subscriber`
decorators to the same function. This is shown below.

```python
import roshelper
import rospy
from std_msgs.msg import Int64

n = roshelper.Node("summer_listener")

@n.subscriber("/smaller_sum", Int64)
@n.subscriber("/larger_sum", Int64)
def subscribe_sum(num):
    rospy.loginfo("Num: {}".format(num))

...
```

In this example, `subscribe_sum` will be called whenever the "/smaller_sum" or
"/larger_sum" topics receive a message. However in this example, you are not
sure of who the `subscribe_sum` function is receiving the message from. By
adding another argument to the `subscribe_sum` function, the `subscriber`
decorator will also pass in the topic on which the message is being sent. This
is shown below. 

```python
import roshelper
import rospy
from std_msgs.msg import Int64

n = roshelper.Node("summer_listener")

@n.subscriber("/smaller_sum", Int64)
@n.subscriber("/larger_sum", Int64)
def subscribe_sum(num, topic_name):
    rospy.loginfo("Num ({}): {}".format(topic_name, num))

...
```

Now, `subscribe_sum` will be able to receive messages from multiple topics, but
will also be able to determine which topic the message was received over.
