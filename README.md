# Open_Irr
USDA-ARS Appalachian Fruit Research Station

This repository contains the documentation for the Open_Irr soil water tension management system.

Upcoming Features:
Time Based Event Scheduling:
 - Users will be able to schedule new events on a recurring or non recurring basis. Currently, the plan is to have radio and irrigation events, but there are many possibilities for extension to more general events. This has mainly involved creating new structs for events and reading and storing data on the SD card using the JSON format and Arduino JSON. We also utilize the RTC clock to recognize when events are occurring or about to occur.,

Non-Blocking Irrigation
- In the previous version, only one event could happen at a time. This means that even if there were 2 irrigation events from 10 AM - 12 PM on a certain day for different zones (say zone 1 and 4), the device would only be able to irrgiate one zone. Soon, users will be able to schedule multiple events to occur at the same time. Going back to the same example, there could be an irrigation event for zone 1 and zone 4 both occuring from 10 AM to 12 PM on a certain day. This feature is mainly accomplished by using states and state machines as well as nonblocking timers.

Better Valve Support
- Users will be able to add new valve types and there are plans to support latching and non latching valve types. In addition, there will be a set of predefined valve configurations which the user can utilize. The implementation of this feature mainly involves structs for valves, some new logic regarding closing and opening normally open or normally closed valves, and storing JSON files on the SD card to store user and predefined valves.

New Menu Functions
- Users will be able to print the last file of the SD card and the last irrigation date/time. The first fe![Open_Irr_Operation_Diagram drawio]
ature will likely be implemented using the basic SD library functions along with basic C++ data structures. The second feature is mainly just printing out information that the device already has.

Basic Diagram For Device Operation

![Open_Irr_Operation_Diagram drawio](https://github.com/user-attachments/assets/cd99bccb-9705-467a-9cc2-d6b5bf07cfd4)


