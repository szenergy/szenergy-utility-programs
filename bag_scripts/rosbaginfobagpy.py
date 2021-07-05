## pip3 install -U bagpy
## pip3 install importlib-resources
## pip3 install opencv-python

import os, re, bagpy, cv2
import pandas as pd
import matplotlib.pyplot as plt

rx = re.compile(r'\.(bag)')
rosbags = list(filter(rx.search, os.listdir('.')))

print("Choose a rosbag:")
i = 0
for rosbag1 in rosbags:
    print(("[%d]: %s")% (i, rosbag1))
    i += 1

inp = input()
if inp.isnumeric():
    print("Opening: ", rosbags[int(inp)])
    myrosbag = rosbags[int(inp)]
elif inp == "a" or inp == "A":
    print("All")
else:
    exit()


b = bagpy.bagreader(myrosbag)

## get the list of topics
#print(b.topic_table)
#
df_pose = pd.read_csv(b.message_by_topic('/current_pose'))

plt.scatter(x = 'pose.position.x', y = 'pose.position.y', data  = df_pose, s= 1)

"""
df_zed = pd.read_csv(b.message_by_topic('/zed_node/left/image_rect_color/compressed'))
"""

plt.show()
