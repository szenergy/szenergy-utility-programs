#!/usr/bin/env python
# rosbag info image display / save
# displays some info (eg image, topics, freq, path) of rosbags in the same directory as this script

from cv_bridge import CvBridge
import os, re, rosbag, cv2
import matplotlib.pyplot as plt
class xyz:  
    def __init__(self):  
        self.x = []  
        self.y = [] 
        self.z = []
        self.t = []

def create_rosbag_plot(rosbags, x, save_or_disp):
    pose_curr = xyz()
    myrosbag = rosbags[x]
    print("Working on: %s" % (myrosbag))
    bag = rosbag.Bag(myrosbag, "r")
    bridge = CvBridge()
    count = 0
    cv_img = None
    for topic, msg, t in bag.read_messages(topics=['/zed_node/left/image_rect_color/compressed', '/zed_node/rgb/image_rect_color', '/current_pose']):
        if topic == '/current_pose':
            pose_curr.x.append(msg.pose.position.x)
            pose_curr.y.append(msg.pose.position.y)
        if topic == '/zed_node/left/image_rect_color/compressed':
            if count == 0:
                cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
                count += 1
        if topic == '/zed_node/rgb/image_rect_color':
            if count == 0:
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                count += 1

    px = 1/plt.rcParams['figure.dpi']  # pixel in inches
    plt.figure(x, figsize=(1800*px, 600*px))
    plt.subplot(131)
    if cv_img is not None:
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
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
            print("  missing: %s" % (important))
    plt.ylim([-12, 0])
    if save_or_disp == "display":
        plt.show()
    else: # save
        plt.savefig(myrosbag[:-3]+"png", bbox_inches='tight')
    bag.close()

rx = re.compile(r'\.(bag)')
rosbags = list(filter(rx.search, os.listdir('.')))

print("Choose a rosbag number or -1 if all rosbag needs to saved as an image:")
i = 0
for rosbag1 in rosbags:
    print(("  [%d]: %s")% (i, rosbag1))
    i += 1
inp = input()

if inp < 0: # if -1 save as image
    print("Saving images...")
    for x in range(len(rosbags)):
        create_rosbag_plot(rosbags, x, "save")
else: # display rosbag info with matplotlib
    create_rosbag_plot(rosbags, inp, "save")
