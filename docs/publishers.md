# Publishers

The publisher decorator allows you to call functions that automatically publish
to specified topics. The decorator allows you to publish to a predefined
topic as shown below.

```python
@n.publisher("/topic", SomeMsgType)
def foo(arg1, arg2, ...):
    ...
    return baz

# publishes to "/topic" and `a == baz`
a = foo(1, "hello", ...)
```

The decorator also allows you to publish to an arbitrary set of topics as shown
below.

```python
@n.publisher(SomeMsgType)
def bar(arg1, arg2, ...):
    ...
    return baz

# publishes to "/topic_1" and `b.msg() == baz`
b = bar(1, "hello", ...).publish("/topic_1")

# publishes to "/topic_2"
bar(2, "there", ...).publish("/topic_2")
```

In both cases, the return value of the function you are decorating gets
published to a topic you either predefined or specified at the time of
the `publish` call. The only difference between these two cases, is that when
you call `a = foo(...)`, `a` will be whatever `foo` would normally return, as
opposed to when you call `b = bar(...)`, `b` will be a special multi-publisher
object that allows you to publish the message to any topic you like with the
correct message type. The multi-publisher does have a method, `msg()`, that
allows you to retrieve the return result from `bar`.
