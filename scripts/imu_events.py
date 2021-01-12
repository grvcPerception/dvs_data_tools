#!/usr/bin/env python

import time, os, csv
import rospy
import numpy as np
from dvs_msgs.msg import EventArray
from dvs_msgs.msg import Event
import cv2

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
        timestamp = msg.events[i].ts.secs + msg.events[i].ts.nsecs*0.000000001
        localTime = timestamp - EventsCallback.tZero

        if localTime > 0.82337 and localTime < 2.08 and EventsCallback.imuDataIndex <= len(imu_data):
            #print('data')
            # Initialize the first image
            if EventsCallback.firstImg == True:
                EventsCallback.event_img = np.zeros((260,346,3), np.uint8)
                EventsCallback.imgTs = localTime
                EventsCallback.firstImg = False

            # Update the imu data using the event time_stamp
            while localTime > imu_data[EventsCallback.imuDataIndex][0] and EventsCallback.imuDataIndex<= len(imu_data):
                EventsCallback.imuDataIndex += 1
                print('updating list index')

            print(str(localTime)+'  '+str(imu_data[EventsCallback.imuDataIndex][0])+'  '+str(EventsCallback.imuDataIndex))
            # Save the image if it's the case
            if (localTime - EventsCallback.imgTs > EventsCallback.time_img or EventsCallback.prevCurveLimit != EventsCallback.curveLimit):
                # if EventsCallback.curveLimit == True:
                #     cv2.imwrite(path_border_folder+str(EventsCallback.imgCounter).zfill(8), EventsCallback.event_img)
                # else :
                #     cv2.imwrite(path_center_folder+str(EventsCallback.imgCounter).zfill(8), EventsCallback.event_img)
                EventsCallback.imgCounter +=1

                cv2.imshow('Test', EventsCallback.event_img)
                cv2.waitKey(10)
                # Clear the image and update the image time
                EventsCallback.event_img = np.zeros((msg.height,msg.width,3), np.uint8)
                EventsCallback.imgTs = localTime
                #print('Holaaaaa')

            #print(str(imu_data[EventsCallback.imuDataIndex][6])+' '+str(EventsCallback.imuDataIndex))
            if EventsCallback.min_th < imu_data[EventsCallback.imuDataIndex][6] and imu_data[EventsCallback.imuDataIndex][6] < EventsCallback.max_th:
                EventsCallback.prevCurveLimit = EventsCallback.curveLimit
                EventsCallback.curveLimit = False
                #if EventsCallback.prevCurveLimit != EventsCallback.curveLimit:
                print('Center')
            else:
                EventsCallback.prevCurveLimit = EventsCallback.curveLimit
                EventsCallback.curveLimit = True
                #if EventsCallback.prevCurveLimit != EventsCallback.curveLimit:
                #print('Peak')

            EventsCallback.event_img = draw_pixel(EventsCallback.event_img, msg.events[i].x,msg.events[i].y,msg.events[i].polarity)

    EventsCallback.msgCounter += 1

EventsCallback.tZero = 0
EventsCallback.msgCounter = 0
EventsCallback.imuDataIndex = 0
EventsCallback.imgCounter = 0
EventsCallback.imgTs = 0.0
EventsCallback.curveLimit = False
EventsCallback.prevCurveLimit = False
EventsCallback.firstImg = True
EventsCallback.event_img = np.zeros((260,346,3), np.uint8)
EventsCallback.time_img = 0.1
EventsCallback.min_th = -30
EventsCallback.max_th = 3.5



def main():

    rospy.init_node('save_events')
    global writer_events, writer_event_time, writer_event_polarity, imu_data, path_border_folder, path_center_folder

    path_border_folder = ''
    path_center_folder = ''

    imu_data = []
    fileImuData = '/home/juan/catkin_ws/test/imu_vn.csv'
    with open(fileImuData, mode='r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
                imu_data.append(row)

    print(len(imu_data))
    # Get the global time
    # date_time = time.gmtime()
    # date_id = str(date_time.tm_year)+"-"+str(date_time.tm_mon)+"-"+str(date_time.tm_mday)+"_"+str(date_time.tm_hour)+"_"+str(date_time.tm_min)+"_"+str(date_time.tm_sec)
    # # Current working directory
    # cwd = os.getcwd()
    # print("The text files will be saved in:"+cwd)

    #writer_events = csv.writer(open(cwd+"/NEvents_"+date_id+".csv", 'w'), delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    subscriber_events = rospy.Subscriber('/dvs/events', EventArray, EventsCallback, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()
