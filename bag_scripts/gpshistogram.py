#!/usr/bin/env python
# a simple histogram based on 2 gps pose diftances

#%%
import os, re, rosbag, math
import matplotlib.pyplot as plt
#%%
class xyz:  
    def __init__(self):  
        self.x = []  
        self.y = [] 
        self.z = []
        self.t = []

#%%
rx = re.compile(r'\.(bag)')
rosbags = list(filter(rx.search, os.listdir('.')))
print(os.listdir('.'))
print("Choose a rosbag number:")
i = 0
for rosbag1 in rosbags:
    print(("  [%d]: %s")% (i, rosbag1))
    i += 1
inp = input()

#%%

gps_p = xyz()
gps_a = xyz()
dist = []
myrosbag = rosbags[inp]
print("Working on: %s" % (myrosbag))
bag = rosbag.Bag(myrosbag, "r")
for topic, msg, t in bag.read_messages(topics=['/gps/current_pose', '/gps/current_pose_alt']):
    if topic == '/gps/current_pose':
        gps_p.x.append(msg.pose.position.x)
        gps_p.y.append(msg.pose.position.y)
    if topic == '/gps/current_pose_alt':
        gps_a.x.append(msg.pose.position.x)
        gps_a.y.append(msg.pose.position.y)
        try:
            if gps_p.x[-1] > 0:
                dist.append(math.sqrt((gps_p.x[-1] - msg.pose.position.x)**2 + (gps_p.y[-1] - msg.pose.position.y)**2))
        except:
            None


#%%

plt.hist(dist)
plt.show()

bag.close()

# %%
