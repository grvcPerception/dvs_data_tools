# dvs_data_tools
Software tools to handle the information from DVS sensors. 

The `eventFrame_node` node renders the event information into frames of events.  A simple example is described in `events2frames.launch`
There are four options of visualization
  1. Defaul. Rendering events at the fequency of the event publisher (tipically `/dvs/events`)
  2. Time. Rendering events within a specific time window
  3. Number of Events. Events are redendered in groups of N events
  4. Experimental
  

 
 The `save_events_raw.py` script saves the events provided by the publisher (tipically `/dvs/events`) in a csv file.
