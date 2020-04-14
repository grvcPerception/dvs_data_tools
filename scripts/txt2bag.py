#!/usr/bin/env python

import time, os, csv, sys
import rospy
import numpy as np
import rosbag
from numpy import genfromtxt

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dvs_msgs.msg import EventArray
from dvs_msgs.msg import Event

#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

def getEventsMsg(events, width, height, tZero, index_reference, time_reference, t_ROS):
    msg = EventArray()
    msg.header.frame_id = "vide2e"
    msg.header.stamp = t_ROS
    msg.width = width
    msg.height = height

    index = index_reference 

    if index < len(events):
        while(events[index][0]<time_reference ):
            e = Event() 
            e.ts = rospy.Time(tZero+events[index][0])
            e.x = events[index][1]
            e.y = events[index][2]
            if  events[index][3] == 1:
                e.polarity = True
            else :
                e.polarity = False

            index+=1
            msg.events.append(e)

    return msg, index

def image2imMsg(im_ , time, format):
    im_msg  = CvBridge().cv2_to_imgmsg(im_,format)
    im_msg.header.frame_id = "vide2e"
    im_msg.header.stamp = time
    im_msg.height = im_.shape[0] 
    im_msg.width = im_.shape[1]
    im_msg.step = im_.shape[1]*im_.shape[2]
    im_msg.encoding = "bgr8"

    return im_msg


def main():
    rospy.init_node('events_txt2bag_node')

    # leading_zeros_ = 8
    # video_rate = 30
    # path_to_events_ = "/home/juan/Github/rpg_vid2e/esim_py/juan/data/images/runing/events.txt"
    # path_to_image_timestamps_ = "/home/juan/Github/rpg_vid2e/esim_py/juan/data/images/runing/timestamps.txt"
    # path_to_images_ = "/home/juan/Github/rpg_vid2e/esim_py/juan/data/images/runing/imgs/"

    path_to_events_ = rospy.get_param("~path_to_events")
    path_to_image_timestamps_ = rospy.get_param("~path_to_image_timestamps")
    path_to_images_ = rospy.get_param("~path_to_images")
    path_to_save_bag_ = rospy.get_param("~path_to_save_bag")
    leading_zeros_ = rospy.get_param("~leading_zeros")
    video_rate = rospy.get_param("~video_rate")

    video_t = 1.0/video_rate
    
    events = genfromtxt(path_to_events_, delimiter=',')
    hf_image_timestamps = genfromtxt(path_to_image_timestamps_, delimiter=',')

    image_total_time = hf_image_timestamps[len(hf_image_timestamps)-1]
    n_images_to_publish = int(round(image_total_time/video_t))
    image_step = int(len(hf_image_timestamps)/n_images_to_publish)

    time_now_ros = rospy.get_rostime()
    t_base = time_now_ros.to_sec()

    # print(t_base)
    # print(len(hf_image_timestamps))
    # print(image_total_time)
    # print(n_images_to_publish)
    # print(image_step)

    date_time = time.gmtime()
    date_id = str(date_time.tm_year)+"-"+str(date_time.tm_mon)+"-"+str(date_time.tm_mday)+"_"+str(date_time.tm_hour)+"_"+str(date_time.tm_min)+"_"+str(date_time.tm_sec)

    #path_to_save_bag_ = os.path.abspath(os.getcwd())
    #bag = rosbag.Bag(path_to_save_bag_+'/'+date_id+'.bag', 'w')
    
    bag = rosbag.Bag(path_to_save_bag_+date_id+'.bag', 'w')
    print("Bag will be safed as : "+path_to_save_bag_+date_id+".bag")

    image_index = 0
    index_reference = 0
    width = 640
    height = 352

    for ii in range(0, n_images_to_publish):
        # Prepare the timestamp for the bag
        t = rospy.Time(t_base+hf_image_timestamps[image_index])

        events_msg, index_reference = getEventsMsg(events, width, height, t_base, index_reference, hf_image_timestamps[image_index], t)
        # add event message to bag file
        bag.write('dvs/events', events_msg, t)
     
        # Read image
        im_ = cv2.imread(path_to_images_+str(image_index).zfill(leading_zeros_)+".png") 
        #cv2.imshow("images", im_)
        #cv2.waitKey(video_rate)
        
        # Convert image to ros image msgs
        im_msg =  image2imMsg(im_ , t, "bgr8")
        # add  image message to bag file
        bag.write('dvs/image_raw', im_msg, t)

        image_index += 8
        
    bag.close()

if __name__ == "__main__":
    main() 