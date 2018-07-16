#!/usr/bin/env python
import rospy
from velodyne_msgs.msg import VelodyneScanUnified

prev_time = 0
prev_az = 0

def callback(data):
    global prev_time
    global prev_az

    num_packets = len(data.packets)
    for i in range(0, num_packets):
        lidar_data = data.packets[i].data
        t1 = ord(lidar_data[1200])
        t2 = ord(lidar_data[1201])
        t3 = ord(lidar_data[1202])
        t4 = ord(lidar_data[1203])
        current_time = (t4 << 24) + (t3 << 16) + (t2 << 8) + t1
        tdiff = current_time - prev_time

        az1 = ord(lidar_data[2])
        az2 = ord(lidar_data[3])
        current_az = (az2 << 8) + az1
        azdiff = current_az - prev_az

        if tdiff > 173:
            print tdiff, azdiff, current_az
        prev_time = current_time
        prev_az = current_az

    #     status_type = ord(lidar_data[1204])
    #     if status_type == 71:
    #         status_value = ord(lidar_data[1205])
    #         print status_value,
    #
    # print '\n'
    # print num_packets

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("/sensor/velodyne64/VelodyneScan", VelodyneScanUnified, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
