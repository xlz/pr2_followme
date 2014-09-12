pr2_followme
============

A PR2 app designed for RoboCup@Home Follow-Me Challenge of RoboCup@Home. See [my demo video](http://www.youtube.com/watch?v=qooZtjGAYaQ&t=786).

This has NOT been tested for redistribution, but hopefully it should be trivial if I have not done so.

This is designed to run with a ROS hydro chroot because it needs PCL 1.7, but PR2 is supported with ROS groovy.

This depends on [a speech engine](https://github.com/xlz/rospcsdk) (also by me), which requires Windows VM running with QEMU/KVM. It needs rebuilt kernel support for KVM.

I used [a hack](http://answers.ros.org/question/28950/what-is-the-best-way-to-follow-a-moving-target/?answer=168951#post-id-168951) in the nav stack. It works, but I don't feel it's the right way of doing it.

TODO
----

* Clean up by a proper ROS packaging format
* Add the starting script as the main entry point
* Add direction of setting up ROS hydro chroot
* Find out possible PCL patch used here
* Examine configurations and document parameter changes
* Document the VM network setup for rospcsdk
* Provide the patch of KVM kernel config
* There seems to be some bug in rospcsdk during long uptime and restarts. Need investigation.
* List dependencies; and other documentation
