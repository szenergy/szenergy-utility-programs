import rosbag
#from tf.transformations import quaternion_from_euler
#quaternion_from_euler(0, 0, 0)
#import geometry_msgs.msg as geomsg

# -1.25 -0.63 -1.1 0.0 0.0 0.0 lexus3/gps  lexus3/base_link
# -1.25 -0.63 -1.1 0.0 0.0 0.0 lexus1/gps  lexus1/base_link
#  0.0   0.0  0.2  0.0 0.0 0.0 base_link   duro_gps

#bag_name = "04_merged.bag"
bag_name = "sync_merged.bag"
dir_name = "/mnt/c/bag/zala_leaf_es_lexus_kozos/"
file_prefix = "tf_"
first_run_a = True
first_run_b = True
first_run_c = True
first_run_d = True
print("Opening %s%s --> %s%s" % (dir_name, bag_name, file_prefix, bag_name))
with rosbag.Bag(dir_name + file_prefix + bag_name, "w") as outbag:
    for topic, msg, t in rosbag.Bag(dir_name + bag_name).read_messages():
        if topic == "/tf":
            if len(msg.transforms) > 1:
                print("Warn: tf message has more than one transform %d" % len(msg.transforms)) # normally this shouldn't be executed
            if msg.transforms[0].header.frame_id == "base_link" and  msg.transforms[0].child_frame_id == "duro_gps":
                msg.transforms[0].transform.rotation.x = 0.0
                msg.transforms[0].transform.rotation.y = 0.0
                msg.transforms[0].transform.rotation.z = 0.0
                msg.transforms[0].transform.rotation.w = 1.0
                if first_run_a:
                    print(msg)
                    first_run_a = False
            if msg.transforms[0].header.frame_id == "lexus1/gps" and  msg.transforms[0].child_frame_id == "lexus1/base_link":
                msg.transforms[0].transform.translation.x = -1.25
                msg.transforms[0].transform.translation.y = -0.63
                msg.transforms[0].transform.translation.z = -1.1
                if first_run_b:
                    print(msg)
                    first_run_b = False
            """
            if msg.transforms[0].header.frame_id == "map" and  msg.transforms[0].child_frame_id == "lexus1/gps":
                if first_run_c:
                    print(msg)
                    first_run_c = False        
            if msg.transforms[0].header.frame_id == "map" and  msg.transforms[0].child_frame_id == "gps":
                if first_run_d:
                    print(msg)
                    first_run_d = False                         
            """        
        outbag.write(topic, msg, t)
print(dir_name + file_prefix + bag_name + " .... ok")


