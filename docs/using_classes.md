# Using Classes

`roshelper` supports the use of classes to organize ROS nodes in order to
maintain state and share information between functions.

## Example

```python
import roshelper
from std_msgs.msg import Int64
from collections import defautldict

n = roshelper.Node("interactive_summer")

@n.entry_point()
class InteractiveSummer(object):

    def __init__(self):
        self.nums = defaultdict(int)

    @n.publisher("/sum", Int64)
    def publish_sum(self, a, b, c):
        num = Int64()
        num.data = a + b + c
        return num

    @n.subscriber("/num_1", Int64)
    @n.subscriber("/num_2", Int64)
    @n.subscriber("/num_3", Int64)
    def subscribe_nums(self, num, topic_name):
        self.nums[topic_name] = num

    @n.main_loop(frequency=30)
    def run(self):
        a = self.nums["/num_1"]
        b = self.nums["/num_2"]
        c = self.nums["/num_3"]
        self.publish_sum(a, b, c)

if __name__ == "__main__":
    n.start(spin=True)
```

In this example, we start a node that subscribes to topics, "/num_1", "/num_2",
and "/num_3". The subscriber stores these three numbers in a dictionary and at
every iteration of the main loop, it publishes the sum of these three numbers.
Unlike working with functions, we need to specify the class as the entry point,
but we also use the `main_loop` decorator to specify which function in the
class will be called in a loop within a separate thread (exactly how
`entry_point` is used for functions).

## Using the Parameter Server

Using a class structure, we can automatically lookup parameters from the
Parameter Server using the constructor of the class. This is shown with a
similar example below.

```python
import roshelper
from std_msgs.msg import Int64
from collections import defautldict

n = roshelper.Node("interactive_summer")

@n.entry_point(exp_a=1, exp_b=1, exp_c=1)
class InteractiveSummer(object):

    def __init__(self, exp_a, exp_b, exp_c):
        self.nums = defaultdict(int)
        self.exp_a = exp_a
        self.exp_b = exp_b
        self.exp_c = exp_c

    @n.publisher("/sum", Int64)
    def publish_sum(self, a, b, c):
        num = Int64()
        num.data = a ** self.exp_a + b ** self.exp_b \
            + c ** self.exp_c
        return num

    @n.subscriber("/num_1", Int64)
    @n.subscriber("/num_2", Int64)
    @n.subscriber("/num_3", Int64)
    def subscribe_nums(self, num, topic_name):
        self.nums[topic_name] = num

    @n.main_loop(frequency=30)
    def run(self):
        a = self.nums["/num_1"]
        b = self.nums["/num_2"]
        c = self.nums["/num_3"]
        self.publish_sum(a, b, c)

if __name__ == "__main__":
    n.start(spin=True)
```

In this example, the class we have defined now takes three parameters. These
parameters are automatically collected from the parameter server using the
names "exp_a", "exp_b", and "exp_c". In the `entry_point` decorator, these
parameters have been given default values in the case that they have not yet
been defined in the parameter server. This is simply just some sugar you can
use. In the constructor you could still manually query the parameter server for
whatever else you like.

### Scope

In order to specify the scope of a parameter being looked up, you need to tell
the `entry_point` decorator where to look.

```python
@n.entry_point(exp_a=("/", 1), exp_b=("~", 1), exp_c=("/", 1))
class InteractiveSummer(object):
```

Each keyword argument in the `entry_point` decorator can be assigned to a tuple
where the first value is the scope and the second is the default value. If no
scope is provided, "~" is used as the default scope.
