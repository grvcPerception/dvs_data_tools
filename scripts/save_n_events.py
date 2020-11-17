#!/usr/bin/env python

import time, os, csv
import rospy
import numpy as np
from dvs_msgs.msg import EventArray
from dvs_msgs.msg import Event

def EventsCallback(msg):
    #print('Hola')
    # initialize time reference
    if EventsCallback.msgCounter == 0:
        EventsCallback.tZero = msg.events[0].ts.secs + msg.events[0].ts.nsecs*0.000000001

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
           if msg.events[i].x >= 41 and msg.events[i].x <= 245:
             EventsCallback.eventCounter += 1
       else:
          print(localeTime)
          data_row = []
          data_row.append(localeTime)
          data_row.append(EventsCallback.eventCounter)
          writer_events.writerow(data_row)

          EventsCallback.eventCounter = 0
          EventsCallback.stepCounter += 1
          
    EventsCallback.msgCounter += 1

EventsCallback.tZero = 0
EventsCallback.eventCounter = 0
EventsCallback.timeStep = 0.025
EventsCallback.stepCounter = 1
EventsCallback.msgCounter = 0



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
    
    subscriber=rospy.Subscriber('/dvs/events', EventArray, EventsCallback, queue_size=1)

    rospy.spin()
   

if __name__ == "__main__":
    main() 
