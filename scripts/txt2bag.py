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

def getEventsMsg(events, width, height, tZero, index_reference, time_reference):
    msg = EventArray()
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

    msg.header.frame_id = ""
    msg.header.seq = getEventsMsg.counter
    if len(msg.events) > 0:
        msg.header.stamp = msg.events[-1].ts
        getEventsMsg.counter += 1

    return msg, index

getEventsMsg.counter = 0

def image2imMsg(im_ , time):
    if(im_.shape[2]>1):
        im_msg = CvBridge().cv2_to_imgmsg(cv2.cvtColor(im_, cv2.COLOR_BGR2GRAY),"mono8")
    else:
        im_msg = CvBridge().cv2_to_imgmsg(im_,"mono8")
        

    im_msg.height = im_.shape[0] 
    im_msg.width = im_.shape[1]
    im_msg.is_bigendian = 0
    im_msg.encoding = "mono8"
    im_msg.step = im_.shape[1]
    
    im_msg.header.frame_id = ""
    im_msg.header.stamp = time
    im_msg.header.seq = image2imMsg.counter

    image2imMsg.counter += 1
    return im_msg

image2imMsg.counter = 0


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
    n_images_in_folder = rospy.get_param("~n_images_in_folder")

    video_t = 1.0/video_rate
    
    print("Loading events")
    events = genfromtxt(path_to_events_, delimiter=',')
    print("Loading images timestamps")
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

    # loading image parameters
    img = cv2.imread(path_to_images_+str(0).zfill(leading_zeros_)+".png") 
   
    date_time = time.gmtime()
    date_id = str(date_time.tm_year)+"-"+str(date_time.tm_mon)+"-"+str(date_time.tm_mday)+"_"+str(date_time.tm_hour)+"_"+str(date_time.tm_min)+"_"+str(date_time.tm_sec)

    #path_to_save_bag_ = os.path.abspath(os.getcwd())
    #bag = rosbag.Bag(path_to_save_bag_+'/'+date_id+'.bag', 'w')
    
    bag = rosbag.Bag(path_to_save_bag_+date_id+'.bag', 'w')
    print("Bag will be safed as : "+path_to_save_bag_+date_id+".bag")

    image_index = 0
    index_reference = 0

    # Step counter
    next_step = video_t

    for ii in range(0, n_images_in_folder):
        if hf_image_timestamps[ii] > next_step:
            t = rospy.Time(t_base+hf_image_timestamps[ii])
            # Read image
            im_ = cv2.imread(path_to_images_+str(ii).zfill(leading_zeros_)+".png") 
            # cv2.imshow("images", im_)
            # cv2.waitKey(video_rate)
            
            # Convert image to ros image msgs
            im_msg =  image2imMsg(im_ , t)
            # add  image message to bag file
            bag.write('dvs/image_raw', im_msg, t)

            height, width, channels = im_.shape 
            # Publish events only if they were triggered before the image
            if (hf_image_timestamps[ii]>=events[0][0]):
                events_msg, index_reference = getEventsMsg(events, width, height, t_base, index_reference, hf_image_timestamps[ii])
                # Only if the message has at least one event
                if len(events_msg.events)>0:
                    t_last_event = events_msg.events[-1].ts
                    # add event message to bag file
                    bag.write('dvs/events', events_msg, t_last_event)
        
            #image_index += image_step

            next_step += video_t

    bag.close()

    # for ii in range(0, n_images_to_publish):
    #     if image_index<len(hf_image_timestamps):
    #         # Prepare the timestamp for the bag
    #         t = rospy.Time(t_base+hf_image_timestamps[image_index])
    #         # Read image
    #         im_ = cv2.imread(path_to_images_+str(image_index).zfill(leading_zeros_)+".png") 
    #         # cv2.imshow("images", im_)
    #         # cv2.waitKey(video_rate)
            
    #         # Convert image to ros image msgs
    #         im_msg =  image2imMsg(im_ , t)
    #         # add  image message to bag file
    #         bag.write('dvs/image_raw', im_msg, t)

    #         height, width, channels = im_.shape 
    #         # Publish events only if they were triggered before the image
    #         if (hf_image_timestamps[image_index]>=events[0][0]):
    #             events_msg, index_reference = getEventsMsg(events, width, height, t_base, index_reference, hf_image_timestamps[image_index])
    #             # Only if the message has at least one event
    #             if len(events_msg.events)>0:
    #                 t_last_event = events_msg.events[-1].ts
    #                 # add event message to bag file
    #                 bag.write('dvs/events', events_msg, t_last_event)
        
    #         image_index += image_step
        
    # bag.close()

if __name__ == "__main__":
    main() 