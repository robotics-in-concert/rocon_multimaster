# Rocon Tests

## Overview

A multimaster unit testing framework. Think of of rostest applied at a higher level again. Syntax, calling and execution are almost identical with the main difference being that you use a multimaster launcher (see notes for **rocon_launch**) instead of a
regular ros launcher.

* **rocon_test**: a command line executor for multimaster tests.
* **add_rocon_test**: a cmake macro for including them in catkin build runs.

## Example

### Launchers

This example can be found in the sources here. A typical rocon launcher must be provided to rocon_test.

```
<concert>
  <launch package="rocon_test" name="talker.launch" port="11311"/>
  <launch package="rocon_test" name="listener.launch" port="11312"/>
</concert>
```

Each individual launcher is then either just a regular ros launcher, or one with tests included. e.g. 
for the above, we have tests in the listener:

```
<launch>
  <test test-name="listener" pkg="rocon_test" type="listener.py"/>
  <node name="latched_talker" pkg="rocon_test" type="talker.py" output="screen"/>
</launch>
```

### Executing

```
> rocon_test rocon_test chatter.test --screen --text_mode
```

Text mode gives the full rocon_test output (setup, teardown notifications etc). 

If your build directory is `/opt/rocon/build`, then you can find the results of the test in 
`/opt/rocon/build/test_results/rocon_test/rostest-test_chatter.xml` and the output from the test
itself in `/opt/rocon/build/test_results/rocon_test/rosunit-listener.xml`.

Alternatively you can use catkin_make or yujin_make to run the tests, but
with one catch - they won't run in parallel yet without getting roscore
conflicts, so make sure you call:

```
> yujin_make --run_tests -j1
```

Other examples can be found in rocon_gateway_tests.