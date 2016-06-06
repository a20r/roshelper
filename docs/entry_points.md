# Entry Points

The `entry_point` decorator specifies the function that will loop continuously
at a given frequency while the node is running. The `entry_point` decorator is
also able to receive the frequency parameter as ROS parameter by specifying a
string instead of a number for the frequency. Below is the simplest way to use
the decorator.

## Simple Example

```python
import roshelper
import rospy

n = roshelper.Node("summer_publisher")

@n.entry_point(frequency=30)
def run():
    msg = publish_sum(1, 2, 3)
    rospy.loginfo("Publishing: {}".format(msg))


if __name__ == "__main__":
    n.start(spin=True)
```

In the example above, `publish_sum` is the function described in the
[publishers description](#docs/publishers). The `run` function is called at 30
Hz as specified in the decorator's arguments. Once `n.start(spin=True)` is
called, a thread with a loop calling `run` will be called at the specified
frequency. Using `spin=True` in the start function is just telling `rospy` to
spin and is equivalent to calling `rospy.spin()` after calling `start`.

## ROS Parameter Frequency

Instead of setting `frequency` to a number, you can specify a ROS parameter to
be automatically looked up and used as the frequency. This is shown below.

```python
@n.entry_point(frequency="~frequency")
def run():
    ...
```

A default argument can also be provided in case the parameter has not been
provided in `rosrun` or `roslaunch`.

```python 
@n.entry_point(frequency="~frequency", default_frequency=30)
def run():
    ...
```

## Important Note

In order to run a node using `roshelper`, you must call `n.start`. This will
instantiate the publishers and subscribers and will call start the thread
running the entry point loop. Also note that the `start` function also takes
keyword arguments such as `anonymous` which is used by the underlying rospy
`init_node`.
