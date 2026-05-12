#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <cdk/cdk.h>
#include <mqueue.h>
#include <fcntl.h>
#include <time.h>

#define CAN_INTERFACE "can0"
#define CAN_QUEUE "/can_queue"

#define GUI_UPDATE_INTERVAL_MS 200                                          // Update interval for GUI in milliseconds

typedef struct 
{
    time_t timeout_ms;
} thread_args_t;

/*-------------------------------------------------------------------------------
 * Function: get_time_ms
 * ------------------------------------------------------------------------------
 * returns the current time in milliseconds
 */
long get_time_ms() 
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/*-------------------------------------------------------------------------------
 * Function: get_nodeStatus
 * ------------------------------------------------------------------------------
 * converts the status byte of a heartbeat message into a human-readable string
 */
const char* get_nodeStatus(uint8_t state) {
    switch (state) {
        case 0:   return "INITIALIZING";
        case 4:   return "STOPPED";
        case 5:   return "OPERATIONAL";
        case 127: return "PRE-OPERATIONAL";
        default:  return "UNKNOWN";
    }
}

/*-------------------------------------------------------------------------------
 * Thread: Readout
 * ------------------------------------------------------------------------------
 * opens queue for writing
 * opens socket and binds to CAN interface (can0, 1Mbit/s, 11-bit IDs)
 * opens message queue for writing
 * loop:
 *      - read CAN messages
 *      - format message as string
 *      - send message to GUI thread via message queue
 */
void* thread_Readout (void* arg) 
{
    int s;                                                                  // Socket for CAN communication
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    mqd_t mq = mq_open(CAN_QUEUE, O_WRONLY);                                // open message queue for writing
    if (mq == (mqd_t)-1) {
        perror("[CAN Thread] mq_open failed");
        return NULL;
    }

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);                                  // open socket for CAN communication
    if (s < 0) { perror("[CAN Thread] socket failed"); return NULL; }       // check if socket opened successfully

    strcpy(ifr.ifr_name, "can0");                                           // specify CAN interface (can0)
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("[CAN Thread] ioctl failed"); return NULL; } // get interface index for CAN interface

    addr.can_family = AF_CAN;                                               // set address family to CAN
    addr.can_ifindex = ifr.ifr_ifindex;                                     // set interface index for CAN communication       
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("[CAN Thread] bind failed"); return NULL; }   // bind socket to CAN interface

    //printf("[Readout Thread] gestartet\n");

    while (1)                                                               // loop to read CAN messages and send to GUI thread
    {
        if (read(s, &frame, sizeof(frame)) > 0)                             // read CAN message from socket
        {
            char msg[128];
            int cob_id = frame.can_id & 0x7FF;

            snprintf(msg, sizeof(msg),                                      // format message as string with ID, data length and data bytes
                "ID: 0x%03X; Dlen: %d; Data: %02X %02X %02X %02X %02X %02X %02X %02X;",
                frame.can_id & 0x7FF,
                frame.can_dlc,
                frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

            mq_send(mq, msg, strlen(msg) + 1, 0);                           // send formatted message to GUI thread via message queue
        }
    }
    return NULL;
}


/*-------------------------------------------------------------------------------
 * GUI-Thread
 * ------------------------------------------------------------------------------
 * Create struct for node status (active, status byte, last message time)
 * open message queue for reading and create buffer for messages
 * initialise GUI with CDK (Create Windows, set layout)
 * Loop: 
 *      - read messages from queue 
 *      - filter for heartbeat and emergency messages
 *      - update node list with heartbeat status
 *      - update GUI accordingly
 */
void* thread_gui(void* arg) 
{    
    const time_t timeout_Heartbeat_ms = *((time_t*)arg);                    // get timeout for heartbeat messages from argument
    typedef struct {                                                        // sturct for node status
        bool active;
        uint8_t status;
        time_t last_msg;
    } CANNode;

    CANNode node_list[128] = {0};                                           // Index 0-127 für NodeID 1-127
    static time_t time_last_update = 0;                                     // variable Last GUI update time

    mqd_t mq_can  = mq_open(CAN_QUEUE,  O_RDONLY | O_NONBLOCK);             // can message queue for reading
    if (mq_can == (mqd_t)-1) perror("[GUI] CAN mq_open failed");            // check if queue opened successfully
    
    char buffer[256];                                                       // buffer for reading messages from queue
    char buf_hb[256];
    char buf_em[256];
    char buf_dat[256];



    WINDOW *cursesWin = initscr();                                          // initialize curses window
    CDKSCREEN *cdkscreen = initCDKScreen(cursesWin);
    initCDKColor();
    curs_set(0);

    
    int width = COLS - 2;                                                   // GUI Layout settings 
    int x = 1;
    int heightNode = 17; 
    int heightEm   = 15;
    int heightData  = LINES - heightNode - heightEm - 6;
    if (heightData < 3) heightData = 3;

    CDKSWINDOW *nodeWin = newCDKSwindow(cdkscreen, x, 1, heightNode, width, "Node-ID: | Status:", 100, TRUE, FALSE);
    CDKSWINDOW *emWin   = newCDKSwindow(cdkscreen, x, heightNode + 2, heightEm, width, "Emergency-Messages:", 100, TRUE, FALSE);
    CDKSWINDOW *dataWin  = newCDKSwindow(cdkscreen, x, heightNode + heightEm + 3, heightData, width, "Data:", 500, TRUE, FALSE);

    refreshCDKScreen(cdkscreen);

    while (1) 
    {
        char temp_buf[512];
        ssize_t bytes;
        bool updated = false;

        while ((bytes = mq_receive(mq_can, temp_buf, sizeof(temp_buf), NULL)) > 0) {    // read messages from queue until queue is empty
            temp_buf[bytes] = '\0'; 
            updated = true;

            int cob_id;
            unsigned int data[8];

            if (sscanf(temp_buf,
                    "ID: 0x%X; Dlen: %*d; Data: %2X %2X %2X %2X %2X %2X %2X %2X",
                    &cob_id,
                    &data[0], &data[1], &data[2], &data[3],
                    &data[4], &data[5], &data[6], &data[7]) >= 1)
            {
                // HEARTBEAT (0x700 + NodeID)
                if ((cob_id & 0x700) == 0x700)
                {
                    int node_id = cob_id & 0x7F;
                    uint8_t status = (uint8_t)data[0];

                    node_list[node_id].active = true;
                    node_list[node_id].status = status;
                    node_list[node_id].last_msg = get_time_ms();
                }

                // EMERGENCY (0x080 + NodeID)
                else if ((cob_id & 0x780) == 0x080)
                {
                    int node_id = cob_id & 0x7F;
                    
                    char em_msg[256];
                    snprintf(em_msg, sizeof(em_msg),
                            "Node %d (Emergency): %s",
                            node_id,
                            temp_buf);

                    addCDKSwindow(emWin, em_msg, BOTTOM);
                }

                // all other messages
                else
                {
                    addCDKSwindow(dataWin, temp_buf, BOTTOM);
                }
            }
        }

        if (bytes == -1 && errno != EAGAIN) perror("mq_receive error");     // check for errors other than empty queue
        
        if (get_time_ms() - time_last_update > GUI_UPDATE_INTERVAL_MS)      // update GUI every #define GUI_UPDATE_INTERVAL_MS xxx ms
        {
            cleanCDKSwindow(nodeWin);
            for (int i = 1; i < 128; i++) 
            {
                if (node_list[i].active) 
                {
                    if (get_time_ms() - node_list[i].last_msg > timeout_Heartbeat_ms)    // check if last Hartbeat is older than timeout
                    {
                        node_list[i].active = 0;                            // Make node inactive if timeout exceeded
                    } 
                    else                                                    // Node is acive and heartbeat is recent, display in GUI with status
                    {          
                        char row[128];
                        const char* state_name = get_nodeStatus(node_list[i].status);

                        snprintf(row, sizeof(row), "Node %3d | %-15s (0x%02X)", 
                                i, state_name, node_list[i].status);
                        
                        addCDKSwindow(nodeWin, row, BOTTOM);
                    }
                }
            }
            time_last_update = get_time_ms();
            refreshCDKScreen(cdkscreen);
        }
    }

    endCDK();
    return NULL;
}

/*-------------------------------------------------------------------------------
 * MAIN
 * ------------------------------------------------------------------------------
 * Readout of settings-file Initialise Programm with parameters
 * initialise queue
 * starts Reader- and GUI-Thread
 */
int main() 
{

    time_t timeout_Heartbeat_ms = 2000; // timeout in milliseconds for heartbeat messages

    pthread_t t1, t2;
    
    mq_unlink(CAN_QUEUE);

    struct mq_attr attr;
    attr.mq_flags   = 0;
    attr.mq_maxmsg  = 80;   // Maximal 10 Nachrichten in der Warteschlange
    attr.mq_msgsize = 256;  // Puffergrösse für eine Nachricht
    attr.mq_curmsgs = 0;

    mqd_t mq_can  = mq_open(CAN_QUEUE,  O_CREAT | O_RDWR | O_NONBLOCK, 0644, &attr);
    
    if (mq_can == (mqd_t)-1) perror("mq_can open failed");

    thread_args_t gui_args;
    gui_args.timeout_ms = timeout_Heartbeat_ms;

    pthread_create(&t1, NULL, thread_Readout, NULL);
    pthread_create(&t2, NULL, thread_gui, &gui_args);

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);

    mq_close(mq_can); 
    mq_unlink(CAN_QUEUE);

    return 0;
}
