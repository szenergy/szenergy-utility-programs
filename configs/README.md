
# Saved config files from rviz / plotjuggler / rqtmultiplot etc.

It may comes handy to start e.g. rviz with the following commands:
```
roscd configs
rosrun rviz rviz -d rviz/14leaf_ouster.rviz 
```
or in one command (this time the tab-tab method does not help):
```
rosrun rviz rviz -d `rospack find configs`/rviz/14leaf_ouster.rviz
```

or include it in launch files:
```xml
<launch>
  <node type="rviz" name="rviz" pkg="rviz" args="-d $(find configs)/rviz/14leaf_ouster.rviz" />
</launch>
```