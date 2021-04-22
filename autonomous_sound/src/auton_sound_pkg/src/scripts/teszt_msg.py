import rospy
import autoware_msgs.msg as aw_msgs

def main():
    rospy.init_node("teszt_m", anonymous=True)
    t_pub = rospy.Publisher("/vehicle_status", aw_msgs.VehicleStatus, queue_size=10)
    t_msg = aw_msgs.VehicleStatus()
    r = rospy.Rate(5)
    i = 0
    while(not rospy.is_shutdown()):
        """
        int32 MODE_MANUAL=0
        int32 MODE_AUTO=1
        """
        if i > 20:
            t_msg.drivemode = 0
        else:
            t_msg.drivemode = 1
        if i > 40:
            i = 0
        t_msg.header.stamp = rospy.Time.now()
        t_pub.publish(t_msg)
        i += 1
        r.sleep()

if __name__=="__main__":
    main()
