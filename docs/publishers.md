# Publishers

The `publisher` decorator allows you to call functions that automatically
publish to specified topics. The decorator allows you to either publish to a
single topic specified in the decorator or to publish to multiple topics using
a *multi-publisher* object.

## Single Topic Publishers

By specifying the topic name as the first argument in the decorator, the return
value of the decorating function will be automatically published to the
specified topic whenever said function is called. A simple example is shown
below.

```python
import roshelper
from std_msgs.msg import Int64

n = roshelper.Node("summer_publisher")

@n.publisher("/sum", Int64)
def publish_sum(a, b, c):
    num = Int64()
    num.data = a + b + c
    return num

...

# publishes to "/sum" and `val == num`
val = publish_sum(1, 2, 3)

...
```

This function computes the sum of three numbers, marshals it into a ROS
message, and returns the message. The return value will be automatically
published to the "/sum" topic whenever `publish_sum` is called. The return
value of `publish_sum` will also be the message.

## Multi-Publishers

The `publisher` decorator also allows you to publish to an arbitrary set of
topics. This works the similarly to example above, except the topic is not
specified in the decorator arguments. Instead when you call the decorating
function, you will be returned a `MultiPublisher` object that allows you to
call the `publish` method to specify the topic at the time of the function
call.

```python
import roshelper
from std_msgs.msg import Int64

n = roshelper.Node("summer_publisher")

@n.publisher(Int64)
def publish_sum(a, b, c):
    num = Int64()
    num.data = a + b + c
    return num

...

# publishes to "/smaller_sum" and `small_sum == num`
small_sum = publish_sum(1, 2, 3).publish("/smaller_sum")

# publishes to "/larger_sum" and `large_sum == num`
large_sum = publish_sum(10, 20, 30).publish("/larger_sum")

...
```

In the example above, `publish_sum` is a function that will return an `Int64`.
This `Int64` produced will then be used to publish to a specified topic in the
`publish` method. The return value from `publish` is the message you have
constructed. The `MultiPublisher` object also has a method, `msg()`, that
allows you to retrieve the produced message (i.e. the return value from the
decorating function) without publishing it.

## Important Note

All return values from an function being decorated by `publisher` must return
the ROS message type specified in the decorator. The `publisher` decorator also
takes keyword arguments such as `queue_size` which are used the underlying
rospy publisher.
