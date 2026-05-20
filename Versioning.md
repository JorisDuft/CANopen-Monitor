# Single Thread
## CANopenMonitor_single-thread.c 
- latest version (based on 0.3.1)
- Schortly testet (about same as version 0.3.1)
  - Data load 1 MBit/s all logged and printet on TUI --> 3 - 10 % messages lost
- New features:
  - Filtering is done with CANsocket

## CANopenMonitor_v0.3.1.c
- Testing:
  - Data load 1 MBit/s all logged and printed on TUI --> 3 - 10 % messages lost
  - Data load 1 MBit/s 50% logged and printed on TUI --> 0 % messages lost
- New features:
  - Logging
  - Improved CANopen detection
  - Object filter massively improved
  - Configuration (config file) expanded
  - Performance slightly better
 
# Multi Thread
## CANopenMonitor_multi-thread.c
- latest version (same as 0.2.1)
- Testing
  - Data load 1 MBit/s all logged and printed on TUI --> 13 - 18 % messages lost
  - Data load 1 MBit/s 50% logged and printed on TUI --> 0 % messages lost
