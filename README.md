virtual_obstacles
=================

This repository holds the virtual_obstcales packages for [ROS][1] [Turtlebots][2].
For more information see `package.xml` file. For license see `LICENSE`
and the sourcecode.

This package needs more documentation and more work to be generic useful. I'll
try to add more documentation and modify the source to be more flexible, but don't
hold your breath. If you think it might be useful just ask and I'll try to help.

getting started
---------------

The moving_objects_msg_generator.py generates moving_object
messages from Turtlebots position and planned routes and publishes them.
The moving_objects.cpp is a Costmap Plugin and adds a obstacle layer to
move_base so the route planner avoids the track of other robots and other
robots even if they can't be seen by sensors.

To get it work this package has to be in your catkin workspace and compile
without error on every robot and your coordniating ROS PC, then the `move_base`
Package should be able to load this plugin.

#### Prerequirements

  * every robot in its own namespace. (/Turtlebo1/, /Turtlebot2/, ...)
  * working tf transformation between frame `/map` and all robots frames
    `/TurtlebotX/base_footprint`
  * all ROS PCs use the same `/map` topic

#### Config change on every robot

To load the plugin you have to load different parameter sets in move_base.

So on every robots config you need to change the move_base node config like this:

    <launch>
    ...
      <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
        ...
        <!-- we comment out the turtlebot local_costmap params and use our own -->
        <!--<rosparam file="$(find turtlebot_navigation)/param/local_costmap_params.yaml" command="load" />-->
        <rosparam file="$(find virtual_obstacles)/param/local_costmap_params.yaml" command="load" />
        ...
        <!-- do the same for global_costmap params, here we our move_base plugin -->
        <!--<rosparam file="$(find turtlebot_navigation)/param/global_costmap_params.yaml" command="load" />-->
        <rosparam file="$(find virtual_obstacles)/param/global_costmap_params.yaml" command="load" />
        ...
      </node>
    </launch>

**Note:** parameter in the yaml file have to be adjusted to your map and robots config.

**Note:** config of plugins parameters is possible over dynamic reconfigure.

**Note:** if `own_unique_name` is empty (default) namespace is used

**Note:** robot footprint is hardcoded to a radius of 0.2 m

#### Config change on coordinating workstation (i.e. roscore PC)
On a workstation PC you have to run `moving_objects_generator` by adding

    <launch>
      ...
      <node name="moving_objects_msg_generator" pkg="virtual_obstacles" type="moving_objects_msg_generator.py" args="" />
      ...
    </launch>

to your config or by simply running `roslaunch virtual_obstacles moving_objects_generator`.

**Note:** you may have to edit some strings in `virtual_obstacles/src/moving_objects_msg_generator.py`
to match your namespaces and `tf frame_id's` to work.

**Note:** `robot_names` and `unique_name` in general means the name of the robots namespace.

#### In the end ....

In the end you should have a node `moving_objects_msg_generator` with one
publisher and *number of robots* subscriber. And on every single robot a
`move_base` node with loaded `virtual_obstacles` plug-in.


[1]: http://www.ros.org
[2]: http://wiki.ros.org/Robots/TurtleBot
