#!/usr/bin/env python
# a simple histogram based on 2 gps pose diftances

#%%
import os, re, rosbag, math
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion # ros tf
#%%
class xyz:  
    def __init__(self):  
        self.x = []  
        self.y = [] 
        self.z = []
        self.t = []
        self.ori = []

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
gps_d = xyz()
gps_n = xyz()
dist = []
myrosbag = rosbags[inp]
print("Working on: %s" % (myrosbag))
bag = rosbag.Bag(myrosbag, "r")
count = 0
for topic, msg, t in bag.read_messages(topics=['/current_pose', '/current_pose_alt', '/gps/duro/current_pose', '/gps/nova/current_pose']):
    if topic == '/current_pose':
        gps_p.x.append(msg.pose.position.x)
        gps_p.y.append(msg.pose.position.y)
        gps_p.t.append(t.to_sec())
        _, _, yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        gps_p.ori.append(yaw)
    elif topic == '/current_pose_alt':
        gps_a.x.append(msg.pose.position.x)
        gps_a.y.append(msg.pose.position.y)
        gps_a.t.append(t.to_sec())
        _, _, yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        gps_a.ori.append(yaw)
        if len(gps_p.x): # if not empty
            ## 20 Hz = 0.05 second
            time_margin = 0.018
            if abs(gps_p.t[-1] - t.to_sec()) < time_margin:
                dist.append(math.sqrt((gps_p.x[-1] - msg.pose.position.x)**2 + (gps_p.y[-1] - msg.pose.position.y)**2))
                #if abs(gps_p.ori[-1] - gps_a.ori[-1]) / math.pi * 180 < 90:
                #    dist.append((gps_p.ori[-1] - gps_a.ori[-1]) / math.pi * 180  )
                #print("smaller than ", time_margin,  " sec diff")
                #dist.append(abs(gps_p.t[-1] - t.to_sec()))
            else:    
                #print("larger than ", time_margin,  " sec diff, dropped")
                count += 1  
    elif topic == '/gps/duro/current_pose':
        gps_d.x.append(msg.pose.position.x)
        gps_d.y.append(msg.pose.position.y)
        gps_d.t.append(t.to_sec())
        _, _, yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        gps_d.ori.append(yaw )        
    elif topic == '/gps/nova/current_pose':
        gps_n.x.append(msg.pose.position.x)
        gps_n.y.append(msg.pose.position.y)
        gps_n.t.append(t.to_sec())
        _, _, yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        gps_n.ori.append(yaw ) #/ math.pi * 180     

print("dropped %d of total %d time_margin: %.3f" % (count, len(gps_a.x), time_margin))

#%%

plt.subplot(221)
plt.plot(gps_d.x, gps_d.y, 'y.', alpha=0.1, label="duro")
plt.plot(gps_n.x, gps_n.y, 'g.', alpha=0.1, label="nova")
plt.plot(gps_a.x, gps_a.y, 'y.', alpha=0.9, label="duro base_link")
plt.plot(gps_p.x, gps_p.y, 'g.', alpha=0.9, label="nova base_link")
plt.legend()
plt.axis("equal")
plt.grid(True)

plt.subplot(222)
plt.hist(dist, bins=80, rwidth=0.85)

plt.subplot(223)
plt.plot(gps_d.t, gps_d.ori, 'y', alpha=0.3, label="duro")
plt.plot(gps_n.t, gps_n.ori, 'g', alpha=0.3, label="nova")
plt.plot(gps_a.t, gps_a.ori, 'y', alpha=0.9, label="duro base_link")
plt.plot(gps_p.t, gps_p.ori, 'g', alpha=0.9, label="nova base_link")
plt.grid(True)


plt.subplot(224)
plt.plot(gps_d.x, gps_d.y, 'y.', alpha=0.4)
for i in range(len(gps_d.x)):
    plt.arrow(x=gps_d.x[i], y=gps_d.y[i], dx=2*math.cos(gps_d.ori[i]), dy=2*math.sin(gps_d.ori[i]))
plt.plot(gps_n.x, gps_n.y, 'g.', alpha=0.4)
for i in range(len(gps_n.x)):
    plt.arrow(x=gps_n.x[i], y=gps_n.y[i], dx=2*math.cos(gps_n.ori[i]), dy=2*math.sin(gps_n.ori[i]))
plt.axis("equal")

plt.grid(True)


plt.show()
bag.close()


# %%
