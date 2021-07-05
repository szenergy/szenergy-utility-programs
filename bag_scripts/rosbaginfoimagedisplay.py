#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rosbag
from cv_bridge import CvBridge
import os, re
import matplotlib.pyplot as plt

class xyz:  
    def __init__(self):  
        self.x = []  
        self.y = [] 
        self.z = []
        self.t = []

pose_curr = xyz()

rx = re.compile(r'\.(bag)')
rosbags = list(filter(rx.search, os.listdir('.')))

print("Choose a rosbag:")
i = 0
for rosbag1 in rosbags:
    print(("[%d]: %s")% (i, rosbag1))
    i += 1

inp = input()
myrosbag = rosbags[int(inp)]

bag = rosbag.Bag(myrosbag, "r")
bridge = CvBridge()
count = 0
cv_img = None
for topic, msg, t in bag.read_messages(topics=['/zed_node/left/image_rect_color/compressed', '/current_pose']):
    if topic == '/current_pose':
        pose_curr.x.append(msg.pose.position.x)
        pose_curr.y.append(msg.pose.position.y)
    if topic == '/zed_node/left/image_rect_color/compressed':
        if count == 0:
            cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
            count += 1

px = 1/plt.rcParams['figure.dpi']  # pixel in inches
plt.figure(1, figsize=(1800*px, 600*px))
plt.subplot(131)
if cv_img is not None:
    plt.imshow(cv_img)
    plt.axis('off')
plt.subplot(132)
plt.plot(pose_curr.x, pose_curr.y, '.')
plt.axis("equal")
plt.grid(True)

plt.subplot(133)
topics = bag.get_type_and_topic_info()[1].keys()
freq = bag.get_type_and_topic_info()[1]
importants = ['/current_pose', '/gps/duro/current_pose', '/gps/nova/current_pose', '/left_os1/os1_cloud_node/points', '/right_os1/os1_cloud_node/points', '/scan', '/tf', '/vehicle_status', '/velodyne_left/velodyne_points', '/velodyne_right/velodyne_points', '/zed_node/left/camera_info', '/zed_node/left/image_rect_color/compressed']
i = 0
for important in importants:
    try:
        #print("%s %.0f hz" % (important, freq[important][3]))
        plt.axis('off')
        plt.text(0, i, s = "%s = %.0f hz" % (important, freq[important][3]), fontsize=12)
        i -= 1
    except:
        print("missing:", important)
plt.ylim([-12, 0])
#plt.savefig(myrosbag[:-3]+"png", bbox_inches='tight')
plt.show()
bag.close()

