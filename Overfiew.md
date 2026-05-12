# CANopenMonitor

CANopen Monitor is a Linux-based monitoring and diagnostic tool designed for CANopen networks. It provides terminal user interface for analyzing CANopen communication. It works by connecting a CANable V2.0, probably works with other CAN-to-USB-converter too but not tested.

Features:
 - node table for monitoring active participants on the CANopen-Bus 
 - including node status and activity information
 - Emergency and SDO Abort messages are separated from regular CAN traffic decoded automatically and displayed in a readable format to simplify debugging and fault analysis
 - The monitor supports configurable filtering based on Communication Objects and Node-IDs
 - System behavior and interface settings can be customized through an external configuration file

## Table of Contents
- [Requierements](#Requirements)
- [Installation / Build](#installation/build)
- [Configuration](#configuration)


## Requierements
 - CANopen Monitor is designed to run exclusively on Linux systems.
 - Before using the application, ensure that the `gs_usb` driver for SocketCAN-compatible USB CAN adapters is installed and working correctly on the host system.
 - Docker is required for building and running the application environment.
 - Recommended: Download CAN-utils to test and setup SocketCAN. (gets installed in dockercontainer)


## installation/build
1) Setup Requirements
2) Creat a Workspace directory containing the followiung files:
    - `CANopenMonitor_conf.txt`
    - `DockF_debian_CANopenMonitor_thin`
    - CANopen Monitor source code (only need one of both)
      - `CANopenMonitor_single-thread.c` (got bit more performance)
      - `CANopenMonitor_multi-thread.c`
3) Build Dockerfile:  `docker build -f DockF_debian_CANopenMonitor_thin -t canopen-monitor-thin .`
4) Run Image:         `docker run -it --rm --network=host --privileged -v <workspace>:/workspace canopen-monitor-thin:latest bash`
6) Build file         `gcc -o CANopenMonitor CANopenMonitor_single-thread.c -lpthread -lrt -lcdk -lncurses`
7) Setup SocketCAN:   `sudo ip link set can0 type can bitrate 1000000`
                      `sudo ip link set can0 up` 
8) Start Programm:    `./CANopenMonitor`


## Configuration
The configuration file allows various program settings to be adjusted.

### Hartbeat timeout
The heartbeat timeout setting defines the maximum allowed time between periodic heartbeat messages in milliseconds. If no heartbeat is received within this timeout period, the corresponding node is considered inactive.

### Node removal timeout
The node removal timeout defines how long an inactive node remains visible in the node table. After this period expires, the node is automatically removed from the participant list.

### Enable Logging
Logging can be enabled or disabled through the logging configuration setting. A value of 1 enables logging, while a value of 0 disables it.

### Node-ID filter
The Node-ID filter allows filtering CANopen traffic by specific node IDs. Node IDs are configured as hexadecimal values in the range from 01 to 7F (decimal 1–127). Using FF disables the filter and allows messages from all nodes. Multiple node IDs can be specified to display only selected participants.

### COB Filter
The communication object filter allows filtering by CANopen communication object type. Examples include SDO, PDO, and NMT messages. Using all disables the filter and displays all communication object types. Multiple communication object types can be combined in the filter configuration.
