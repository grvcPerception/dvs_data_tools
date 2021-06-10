#!/usr/bin/env python

import time, os, csv
import rospy
import numpy as np
from dvs_msgs.msg import EventArray
from dvs_msgs.msg import Event

def EventsCallback(msg):

    for i in range(0, len(msg.events)):
       data_row = []
       timestamp = msg.events[i].ts.secs + msg.events[i].ts.nsecs*0.000000001
       data_row.append(timestamp)
       data_row.append(msg.events[i].x)
       data_row.append(msg.events[i].y)
       if msg.events[i].polarity == True :
          data_row.append(1)
       else :
          data_row.append(0)
       # Write a row with the event (timestamp, x, y, polarity)
       writer_events.writerow(data_row)


def main():

    rospy.init_node('save_events')
    global writer_events, writer_event_time, writer_event_polarity
    # Get the global time
    date_time = time.gmtime()
    date_id = str(date_time.tm_year)+"-"+str(date_time.tm_mon)+"-"+str(date_time.tm_mday)+"_"+str(date_time.tm_hour)+"_"+str(date_time.tm_min)+"_"+str(date_time.tm_sec)
    # Current working directory
    cwd = os.getcwd()
    print("The text files will be saved in:"+cwd)

    writer_events = csv.writer(open(cwd+"/Events_"+date_id+".txt", 'w'), delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    
    subscriber=rospy.Subscriber('/dvs/events', EventArray, EventsCallback, queue_size=1)

    rospy.spin()
   

if __name__ == "__main__":
    main() 
