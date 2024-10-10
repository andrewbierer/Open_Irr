# Open_Irr
USDA-ARS Appalachian Fruit Research Station

This repository contains the documentation for the Open_Irr soil water tension management system.

NOTE: This is the beta branch and functionality is NOT guaranteed - in fact, as features are worked on it is likely that this branch will NOT function correctly.
As features are completed and validated, they will be pushed to the main branch.

Upcoming Features:

1) Event Scheduling:
 -  In the new version, users will be able to schedule events (sensor measurements, irrigation events, permission and deny windows for irrigation events) on a recurring or non recurring basis. This has mainly involved creating new data structures for aspects of events and functions for their handling. The RTC is checked frequently during small wake/sleep intervals and detects if there are events that are scheduled to occur before the next wake. If so, the event is added to a queue and the device remains active to avoid sleeping and missing events; if the queue is empty (i.e., there are no events between now and the next wake cycle) the device resumes the small wake/sleep cycles for power conservation.
   
 - Practically, this means that users can pre-schedule aspects of irrigation events. Here are a couple examples.
   (i) FULLY TIMER BASED IRRIGATION
       Similar functionality you might expect from standard type irrigation controllers.

       A user needs to irrigate a field from 4am-8am and 7pm-11pm, beginning on May 1st of 2025 and ending on October 1st of 2025. They may create a new recurring irrigation event to achieve this. Moreover, more than 1 recurring event can be pre-scheduled at a time. A user may want to pre-schedule timer based irrigation events corresponding to the time of the year as an example. 
   
   (ii) PERMIT & DENY WINDOWS FOR FIELD ACCESS, WATER SUPPLY AVAILABILITY ETC.
       A user identifies the need to gain field access later in the month while irrigation events are typically occuring. They may schedule, in advance, a "deny" window so that irrigation events will not occur when work crews will be in the field.
   
       A user's water supply line may only be pressurized for a certain time period during the day. The user may establish a "permit" window within which irrigation events may proceed. e.g., from 6am to 8am daily.
   
2) Non-Blocking Irrigation
- In the previous version, only a singular irrigation group (output pin) could be handled at one time. This means that if the need for irrigation was indicated by sensor measurements for more than one irrigation group the device would only be able to irrgiate one of the groups at a time. Integration of non-blocking code functionality allows users to schedule multiple events to occur at the same time. This is accomplished using event time trackers and principles of state-machine functionality.
  
- For example, there could be an irrigation event for groups 1 and 4 occuring, simultaneously, if called for.


3) Expansion of Valve Logic Support
- Previously, only 100% duty cycle DC valves that are "normally closed -> no water flow when receiving signal logic LOW" were compatible. This means the output signal from the device is HIGH until the irrigation event was completed. This was suitable for small installations where water requirements were low (lots of small time duration irrigation events) and the valves in use simplistic. The implementation of this feature involves structured data for pre-defined valve models stored in header files, writing the contents of the header file to a datafile on the SDcard in JSON format, appending the datafile when a new user-defined valve model is entered, and a menu feature for users to configure a new valve model. A new function was developed to evaluate the logic of any valve using the information in the valve model struct and output an integer, corresponding to the routine logic necessary, which is stored in non-volatile EEPROM memory for access. 
  
- Support for Latching and Non-Latching valves will be completed to facilitate greater potential for end-user adoption in different production systems.
- Support for "normally open -> no water flow when receiving signal logic HIGH" valves will also expand the compatible valves.
- Common valve models will be stored in header files for users to select from and the correct logic will be automatically applied.
- Users will be able to enter specifications for additional valve models to save and select in the future.

Quality of Life Menu Functions
- Users will be able to print the last datafile line from the SD card to check the last data record written; similarly, an option to print the last irrigation date & time for each irrigation group will be added.

Basic Logic Diagram For Device Operation
![Open_Irr_Operation_Diagram drawio](https://github.com/user-attachments/assets/cd99bccb-9705-467a-9cc2-d6b5bf07cfd4)

Once the device wakes up, it checks for upcoming tasks. This could be either sceduled events or measurement events. If there are measurement events, the device proceeds to sensor based irrigation. This is nonblocking. The details as to when such measurement events occur are specified by the user. If the user has specified other events such as scheduled irrigation or radio events, the device performs these events. In the case of irrigation, since this is a nonblocking event, the device can perform other tasks at the same time. Once an event is done, it is removed from the queue. Once the queue is empty, the device can go back to the Awake state and decide whether or not to go to sleep.


###Continue Standard README below###
