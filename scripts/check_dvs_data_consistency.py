#!/usr/bin/env python

import time, os, csv
import rospy
import numpy as np
from dvs_msgs.msg import EventArray
from dvs_msgs.msg import Event
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import math


def inEdge(x, y):
        return y<115 and x>179 and math.sqrt(pow(y-115,2) + pow(x-179,2)) > 182 or y>135 and x>245 and math.sqrt(pow(y-135,2) + pow(x-245,2)) > 131 or y>135 and x>260 and math.sqrt(pow(y-135,2) + pow(x-260,2)) > 124 or y<115 and x<160 and math.sqrt(pow(y-115,2) + pow(x-160,2)) > 165 or y>115 and x<160 and math.sqrt(pow(y-115,2) + pow(x-160,2)) > 178 or y>115 and x<105 and math.sqrt(pow(y-115,2) + pow(x-105,2)) > 144
        #y<115 and x>179 and math.sqrt((y-115)**2 + (x-179)**2) > 182 or y>135 and x>245 and math.sqrt((y-135)**2 + (x-245)**2) > 131 or y>135 and x>260 and math.sqrt((y-135)**2 + (x-260)**2) > 124 or y<115 and x<160 and math.sqrt((y-115)**2 + (x-160)**2) > 165 or y>115 and x<160 and math.sqrt((y-115)**2 + (x-160)**2) > 178 or y>115 and x<105 and math.sqrt((y-115)**2 + (x-105)**2) > 144:
def EventsCallback(msg):
    print(str(msg.header.stamp))
    #print('Hola')
    # initialize time reference
    if EventsCallback.msgCounter == 0:
        EventsCallback.tZero = msg.events[0].ts.secs + msg.events[0].ts.nsecs*0.000000001
        print('tZero initialized')

    for i in range(0, len(msg.events)):
       timestamp = msg.events[i].ts.secs + msg.events[i].ts.nsecs*0.000000001
       #data_row.append(timestamp)
       #data_row.append(msg.events[i].x)
       #data_row.append(msg.events[i].y)
       #if msg.events[i].polarity == True :
       #   data_row.append(1)
       #else :
       #   data_row.append(0)
       # Write a row with the event (timestamp, x, y, polarity)
       localeTime = timestamp - EventsCallback.tZero
       if localeTime < EventsCallback.timeStep*EventsCallback.stepCounter :
           if not inEdge(msg.events[i].x, msg.events[i].y):
               EventsCallback.eventCounter += 1
       else:
          #print(localeTime)
          data_row = []
          data_row.append(localeTime)
          data_row.append(EventsCallback.eventCounter)
          writer_events.writerow(data_row)

          EventsCallback.eventCounter = 0
          if not inEdge(msg.events[i].x, msg.events[i].y):
              EventsCallback.stepCounter += 1

    EventsCallback.msgCounter += 1

EventsCallback.tZero = 0
EventsCallback.eventCounter = 0
EventsCallback.timeStep = 0.001
EventsCallback.stepCounter = 1
EventsCallback.msgCounter = 0


def ImageCallback(msg):
    print(str(msg.header.stamp))
    #print('Hola')
    # initialize time reference
    if ImageCallback.msgCounter == 0:
        ImageCallback.tZero = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
        print('tZero initialized')

    timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
    localeTime = timestamp - ImageCallback.tZero

    data_row = []
    data_row.append(localeTime)
    data_row.append(ImageCallback.msgCounter)
    writer_events.writerow(data_row)

    ImageCallback.msgCounter += 1

ImageCallback.msgCounter = 0
ImageCallback.tZero = 0

def ImuCallback(msg):
    print(str(msg.header.stamp))
    #print('Hola')
    # initialize time reference
    if ImuCallback.msgCounter == 0:
        ImuCallback.tZero = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
        print('tZero initialized')

    timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
    localeTime = timestamp - ImuCallback.tZero

    data_row = []
    data_row.append(localeTime)
    data_row.append(ImuCallback.msgCounter)
    writer_events.writerow(data_row)

    ImuCallback.msgCounter += 1

ImuCallback.msgCounter = 0
ImuCallback.tZero = 0

def EventBatchCallback(msg):
    print(str(msg.header.stamp))
    #print('Hola')
    # initialize time reference
    if EventBatchCallback.msgCounter == 0:
        EventBatchCallback.tZero = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
        print('tZero initialized')

    timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
    localeTime = timestamp - EventBatchCallback.tZero

    data_row = []
    data_row.append(localeTime)
    data_row.append(EventBatchCallback.msgCounter)
    writer_events.writerow(data_row)

    EventBatchCallback.msgCounter += 1

EventBatchCallback.msgCounter = 0
EventBatchCallback.tZero = 0



def main():

    rospy.init_node('save_events')
    global writer_events, writer_event_time, writer_event_polarity
    # Get the global time
    date_time = time.gmtime()
    date_id = str(date_time.tm_year)+"-"+str(date_time.tm_mon)+"-"+str(date_time.tm_mday)+"_"+str(date_time.tm_hour)+"_"+str(date_time.tm_min)+"_"+str(date_time.tm_sec)
    # Current working directory
    cwd = os.getcwd()
    print("The text files will be saved in:"+cwd)

    writer_events = csv.writer(open(cwd+"/NEvents_"+date_id+".csv", 'w'), delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

    subscriber_events =rospy.Subscriber('/dvs/events', EventArray, EventsCallback, queue_size=1)
    #subscriber_events =rospy.Subscriber('/dvs/events', EventArray, EventBatchCallback, queue_size=1)
    #subscriber_aps =rospy.Subscriber('/dvs/image_raw', Image, ImageCallback, queue_size=1)
    #subscriber_imu =rospy.Subscriber('/dvs/imu', Imu, ImuCallback, queue_size=1)

    rospy.spin()


if __name__ == "__main__":
    main()
