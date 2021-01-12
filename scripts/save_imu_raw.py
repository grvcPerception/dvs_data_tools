#!/usr/bin/env python

import time, os, csv
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from dvs_msgs.msg import EventArray
from dvs_msgs.msg import Event

def imuCallback(msg):
    if tZeroEvents > 0.0:
        if imuCallback.msgCounter == 0:
            tZeroImu = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
            if tZeroImu >= tZeroEvents :
                imuCallback.tZero = tZeroEvents
                print('tZeroImu initialized (events)')
            else :
                imuCallback.tZero = tZeroImu
                print('tZeroImu initialized (Imu)')
        timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
        localTime = timestamp - imuCallback.tZero

        data_row = []
        data_row.append(localTime)
        data_row.append(msg.angular_velocity.x)
        data_row.append(msg.angular_velocity.y)
        data_row.append(msg.angular_velocity.z)
        data_row.append(msg.linear_acceleration.x)
        data_row.append(msg.linear_acceleration.y)
        data_row.append(msg.linear_acceleration.z)
        writer_events.writerow(data_row)

        imuCallback.msgCounter += 1

imuCallback.tZero = 0
imuCallback.msgCounter = 0

def EventsCallback(msg):
    if EventsCallback.msgCounter == 0:
        global tZeroEvents
        tZeroEvents = msg.events[0].ts.secs + msg.events[0].ts.nsecs*0.000000001
        print('tZeroEvents initialized ')
        EventsCallback.msgCounter += 1

EventsCallback.msgCounter = 0

def main():

    rospy.init_node('save_imu')
    global writer_events, writer_event_time, writer_event_polarity, tZeroEvents
    tZeroEvents = -1.0
    # Get the global time
    date_time = time.gmtime()
    date_id = str(date_time.tm_year)+"-"+str(date_time.tm_mon)+"-"+str(date_time.tm_mday)+"_"+str(date_time.tm_hour)+"_"+str(date_time.tm_min)+"_"+str(date_time.tm_sec)
    # Current working directory
    cwd = os.getcwd()
    print("The text files will be saved in:"+cwd)

    writer_events = csv.writer(open(cwd+"/imu_"+date_id+".csv", 'w'), delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

    subscriber_imu = rospy.Subscriber('/vn/imu', Imu, imuCallback, queue_size=1)
    subscriber_events = rospy.Subscriber('/dvs/events', EventArray, EventsCallback, queue_size=1)

    rospy.spin()


if __name__ == "__main__":
    main()
