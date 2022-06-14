# `marathon_waypoint` ROS package

Similar functionality as [Autoware's `waypoint_maker`](https://github.com/Autoware-AI/core_planning/tree/master/waypoint_maker).
## Build

```
catkin build marathon_waypoint
```

# ROS publications / subscriptions

```mermaid
flowchart LR

A(waypoint_loader) --> |autoware_msgs/LaneArray| B[based/lane_waypoints_raw]
```


```mermaid
flowchart LR

A[current_pose] -->|geometry_msgs/PoseStamped| B(waypoint_saver)
B --> |visualization_msgs/MarkerArray| C[waypoint_saver_marker]
```

## Run

```
rosrun marathon_waypoint waypoint_loader _multi_lane_csv:="/mnt/c/bag/temp/ok.csv"
```

```
rosrun marathon_waypoint waypoint_saver _save_filename:=del.csv _pose_topic:=current_pose
```
## Launch

``` xml
<launch>
    <node pkg="marathon_waypoint" type="waypoint_loader" name="waypoint_loader1" output="screen">
        <param name="multi_lane_csv" type="string" value="/mnt/c/bag/temp/ok.csv" />
    </node>
</launch>


<launch>
    <node pkg="marathon_waypoint" type="waypoint_saver" name="waypoint_saver1" output="screen">
        <param name="save_filename" type="string" value="/mnt/c/bag/temp/del.csv" />
        <param name="pose_topic" type="string" value="current_pose" />
    </node>
</launch>
```