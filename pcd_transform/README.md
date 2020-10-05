This node translate a previously saved PCD map with a 4x4 translation matrix and save it with a new name. It works with dynamic parameters.

# How to use it

First write in a terminal:

`rosrun pcd_transform rosrun dynamic_rec_node`

Then in an other termial open the dynamic parameters with:

`rosrun rqt_reconfigure rqt_reconfigure`

The following gui will appear:

![pcd_transform](https://user-images.githubusercontent.com/51919446/95057233-25c01980-06f6-11eb-83a1-24873365f043.png)

**The translation parameters are in meters.The rotatoion parameters are in deg!** 

Set the parameters then run the following command:

`rosrun pcd_transform transform_node`

If the transformation is succesfull you will see this in the terminal:

![pcdtrans](https://user-images.githubusercontent.com/51919446/95058000-30c77980-06f7-11eb-8732-9baf3db8b382.png)
