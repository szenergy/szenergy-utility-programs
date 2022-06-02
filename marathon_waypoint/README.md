
Similar functionality as [Autoware's `waypoint_maker`](https://github.com/Autoware-AI/core_planning/tree/master/waypoint_maker) but without Autoware messages and depenency.

```
catkin build marathon_waypoint
```

```
rosrun marathon_waypoint waypoint_loader _multi_lane_csv:="/mnt/c/bag/temp/ok.csv"
```

```
rosrun marathon_waypoint waypoint_saver
```