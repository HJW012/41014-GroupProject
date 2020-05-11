# 41014-GroupProject
41014 Sensors and Control - Group Project


Hi gents,

I have moved the ar_tag_demo package folder to this repository so you guys can see how it runs.

A few points before it will work properly. You guys will need to clone and build the ar_track_alvar ROS package

http://wiki.ros.org/ar_track_alvar#ar_track_alvar.2BAC8-post-fuerte.Generating_AR_tags

To run the package you will need to initialise a turtlebot environment first then roslaunch the main.launch

To look at the outputs you can rostopic echo the ar_pose_marker topic or you can run RVIZ, set the camera as the main reference (instead of map) and add the TF. You should then be able to move both the turtlebot and/or the marker around gazebo and it should update.

I'm trying to work on only needing to run a single launch file, but it seems that because it is an XML file the launch files may not be called sequentially and it results in some weird things (the turtlebot is rendered as the tag bundle and stuff like that). Maybe it might be worth looking into calling launch files from MATLAB but I have no idea if that is even possible.

Any issues with running it, let me know. I'm hoping you will have everything and it should work fine.
