# hector_navigation_node
Using hector_exploration_planner, hector_path_follower and hector_nav_msgs we can navigate, using RViz 2D nav goal

## How to use?

Basic launch file:

```xml
<node pkg="hector_navigation_node" type="hector_navigation_node" name="hector_navigation_node" output="screen">
	<rosparam file="$(find hector_exploration_node)/config/costmap.yaml" command="load" />
</node>
```

When you launch, you can set 2D nav goals in rviz, that are in published into `/move_base_simple/goal` topic.

The node publishes a `/nav_path` topic.