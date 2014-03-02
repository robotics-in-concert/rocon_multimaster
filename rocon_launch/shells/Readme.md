It isn't perfect. If -- options come before the rocon launch arguments it is foobar because
the roslaunch search engine expects those arguments at 0 and 1. It could be done by stripping
out the cwords, but that would require patching roslaunch. See notes in the link below:

http://stackoverflow.com/questions/3578584/bash-how-to-delete-elements-from-an-array-based-on-a-pattern