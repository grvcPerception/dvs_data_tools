#!/usr/bin/env python

import time, os, csv
import rospy
import numpy as np
from dvs_msgs.msg import EventArray
from dvs_msgs.msg import Event
import cv2
import math

def inEdge(x, y):
    return y<115 and x>179 and math.sqrt(pow(y-115,2) + pow(x-179,2)) > 182 or y>135 and x>245 and math.sqrt(pow(y-135,2) + pow(x-245,2)) > 131 or y>135 and x>260 and math.sqrt(pow(y-135,2) + pow(x-260,2)) > 124 or y<115 and x<160 and math.sqrt(pow(y-115,2) + pow(x-160,2)) > 165 or y>115 and x<160 and math.sqrt(pow(y-115,2) + pow(x-160,2)) > 178 or y>115 and x<105 and math.sqrt(pow(y-115,2) + pow(x-105,2)) > 144

def draw_pixel(im, x, y, polarity):
    if polarity == 1:
        im[y,x] = (255,0,0)
    else:
        im[y,x] = (0,0,255)
    return im

def EventsCallback(msg):
    #print('Hola')
    # initialize time reference
    if EventsCallback.msgCounter == 0:
        EventsCallback.tZero = msg.events[0].ts.secs + msg.events[0].ts.nsecs*0.000000001

    for i in range(0, len(msg.events)):
        if not inEdge(msg.events[i].x, msg.events[i].y):
            timestamp = msg.events[i].ts.secs + msg.events[i].ts.nsecs*0.000000001
            localTime = timestamp - EventsCallback.tZero

            if localTime > 0.82337 and localTime < 2.08 and EventsCallback.imuDataIndex <= len(imu_data):
                #print('data')
                # Increment the event counter
                EventsCallback.eventCounter += 1
                # Initialize the first image
                if EventsCallback.firstImg == True:
                    EventsCallback.event_img = np.zeros((260,346,3), np.uint8)
                    EventsCallback.imgTs = localTime
                    EventsCallback.firstImg = False

                #print(str(localTime)+'  '+str(imu_data[EventsCallback.imuDataIndex][0])+'  '+str(EventsCallback.imuDataIndex))
                # Update the imu data using the event time_stamp
                while localTime > float(imu_data[EventsCallback.imuDataIndex][0]) and EventsCallback.imuDataIndex<= len(imu_data):
                    EventsCallback.imuDataIndex += 1

                # Save the image if it's the case
                if (localTime - EventsCallback.imgTs > EventsCallback.time_img or EventsCallback.prevCurveLimit != EventsCallback.curveLimit):
                    data_row = []
                    data_row.append(localTime)
                    data_row.append(EventsCallback.eventCounter)

                    if EventsCallback.curveLimit == True:
                        cv2.imwrite(path_border_folder+'/'+str(EventsCallback.imgCounter).zfill(8)+'.png', EventsCallback.event_img)
                        data_row.append(1)
                    else :
                        cv2.imwrite(path_center_folder+'/'+str(EventsCallback.imgCounter).zfill(8)+'.png', EventsCallback.event_img)
                        data_row.append(0)
                    EventsCallback.imgCounter +=1

                    cv2.imshow('Test', EventsCallback.event_img)
                    cv2.waitKey(10)
                    # Clear the image and update the image time
                    EventsCallback.event_img = np.zeros((msg.height,msg.width,3), np.uint8)
                    EventsCallback.imgTs = localTime

                    writer_events.writerow(data_row)
                    EventsCallback.eventCounter = 0

                #print(str(imu_data[EventsCallback.imuDataIndex][6])+' '+str(EventsCallback.imuDataIndex))
                if EventsCallback.min_th < float(imu_data[EventsCallback.imuDataIndex][6]) and float(imu_data[EventsCallback.imuDataIndex][6]) < EventsCallback.max_th:
                    EventsCallback.prevCurveLimit = EventsCallback.curveLimit
                    EventsCallback.curveLimit = False
                    #if EventsCallback.prevCurveLimit != EventsCallback.curveLimit:
                    #print('Center :'+str(imu_data[EventsCallback.imuDataIndex][6])+' '+str(EventsCallback.imuDataIndex))

                else:
                    EventsCallback.prevCurveLimit = EventsCallback.curveLimit
                    EventsCallback.curveLimit = True
                    #if EventsCallback.prevCurveLimit != EventsCallback.curveLimit:
                    #print('Peak :'+str(imu_data[EventsCallback.imuDataIndex][6])+' '+str(EventsCallback.imuDataIndex))
                EventsCallback.event_img = draw_pixel(EventsCallback.event_img, msg.events[i].x,msg.events[i].y,msg.events[i].polarity)

    EventsCallback.msgCounter += 1

EventsCallback.tZero = 0
EventsCallback.eventCounter = 0
EventsCallback.msgCounter = 0
EventsCallback.imuDataIndex = 0
EventsCallback.imgCounter = 0
EventsCallback.imgTs = 0.0
EventsCallback.curveLimit = False
EventsCallback.prevCurveLimit = False
EventsCallback.firstImg = True
EventsCallback.event_img = np.zeros((260,346,3), np.uint8)
EventsCallback.time_img = 0.001
EventsCallback.min_th = -30
EventsCallback.max_th = 3.5



def main():

    rospy.init_node('save_events')
    global writer_events, writer_event_time, writer_event_polarity, imu_data, path_border_folder, path_center_folder

    imu_data = []
    fileImuData = '/home/juan/catkin_ws/test_dvs/imu_vn.csv'
    with open(fileImuData, mode='r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
                imu_data.append(row)

    print(len(imu_data))
    # Get local time of the computer
    date_time = time.gmtime()
    date_id = str(date_time.tm_year)+"-"+str(date_time.tm_mon)+"-"+str(date_time.tm_mday)+"_"+str(date_time.tm_hour)+"_"+str(date_time.tm_min)+"_"+str(date_time.tm_sec)
    # # Current working directory
    cwd = os.getcwd()
    if not os.path.exists(cwd+'/'+date_id):
        ref_folder = cwd+'/'+date_id
        os.makedirs(ref_folder)
        path_border_folder = ref_folder+'/border'
        os.makedirs(path_border_folder)
        path_center_folder = ref_folder+'/center'
        os.makedirs(path_center_folder)
        print("The text files will be saved in:"+ref_folder)
        writer_events = csv.writer(open(ref_folder+"/NEvents.csv", 'w'), delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

    subscriber_events = rospy.Subscriber('/dvs/events', EventArray, EventsCallback, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()
