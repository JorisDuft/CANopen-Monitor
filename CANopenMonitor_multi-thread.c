#define _GNU_SOURCE

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
#include <strings.h>
#include <cdk/cdk.h>
#include <mqueue.h>
#include <fcntl.h>
#include <time.h>
#include <stdbool.h>
#include <errno.h>

#define CAN_INTERFACE "can0"
#define CAN_QUEUE "/can_queue"

#define GUI_UPDATE_INTERVAL_MS 200                                          // Update interval for GUI in milliseconds

#define OBJ_SDO  (1 << 0)                                                   // object filter bitmask values for SDO-, PDO-, NMT- messages (extendable)
#define OBJ_PDO  (1 << 1)
#define OBJ_NMT  (1 << 2)

typedef struct 
{
    uint8_t cob;
    uint8_t id;
    uint8_t len;
    uint8_t data[8];
    uint64_t timestamp;
} CANopenMessage;


typedef struct 
{
    time_t timeout_ms;
    time_t remove_timeout_ms;

    bool log_enabled;

    // Node Filter
    int nodeID_arr[127];                                                    // Array to store Node IDs for filtering (1-127)
    int nodeID_count;                                                       // Count of Node IDs in the filter
    bool node_filter_enabled;

    // Object Filter (Bitmaske)
    int object_mask;                                                        // Bitmask filter for communication-object (SDO, PDO, NMT)  
    bool object_filter_enabled;
} thread_args_t;

/*-------------------------------------------------------------------------------
 * Function: get_time_ms
 * ------------------------------------------------------------------------------
 * returns the current time in milliseconds
 */
long get_time_ms() 
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

void format_time(char* out, size_t size, uint64_t t_ms)
{
    if (t_ms == 0)
    {
        snprintf(out, size, "-");
        return;
    }

    time_t sec = t_ms / 1000;
    struct tm *tm_info = localtime(&sec);
    tm_info->tm_hour = tm_info->tm_hour + 2;
    strftime(out, size, "%H:%M:%S", tm_info);
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
 * Function: get_emergencyErrorDescription
 * ------------------------------------------------------------------------------
 * converts the error code of an emergency message into a human-readable string
 */
const char* get_emergencyErrorDescription(uint16_t error_code) {
    const char* error_desc;
       switch (error_code & 0xF000)
    {
        case 0x0000: error_desc = "No Error / Reset"; break;
        case 0x1000: error_desc = "Generic Error"; break;
        case 0x2000: error_desc = "Current Error"; break;
        case 0x3000: error_desc = "Voltage Error"; break;
        case 0x4000: error_desc = "Temperature Error"; break;
        case 0x5000: error_desc = "Hardware Error"; break;
        case 0x6000: error_desc = "Software Error"; break;
        case 0x8000: error_desc = "Communication Error"; break;
        case 0xF000: error_desc = "Manufacturer Specific"; break;
        default:      error_desc = "Unknown Error"; break;
    }
    return error_desc;
}

/*-------------------------------------------------------------------------------
 * Function: get_sdoAbortDescription
 * ------------------------------------------------------------------------------
 * converts the SDO abort code into a human-readable string
 */
const char* get_sdoAbortDescription(uint32_t code)
{
    switch (code)
    {
        case 0x05030000: return "Toggle bit not altered";
        case 0x05040000: return "SDO timeout";
        case 0x05040001: return "Invalid command";
        case 0x05040002: return "Invalid block size";
        case 0x05040003: return "Invalid sequence number";
        case 0x05040004: return "CRC error";
        case 0x05040005: return "Out of memory";
        case 0x06010000: return "Unsupported access";
        case 0x06010001: return "Read of write-only";
        case 0x06010002: return "Write of read-only";
        case 0x06020000: return "Object does not exist";
        case 0x06040041: return "Object cannot be mapped";
        case 0x06040042: return "PDO length exceeded";
        case 0x06040043: return "Parameter incompatibility";
        case 0x06040047: return "Device incompatibility";
        case 0x06060000: return "Hardware error";
        case 0x06070010: return "Data type mismatch";
        case 0x06070012: return "Data too long";
        case 0x06070013: return "Data too short";
        case 0x06090011: return "Subindex does not exist";
        case 0x06090030: return "Invalid value";
        case 0x06090031: return "Value too high";
        case 0x06090032: return "Value too low";
        case 0x06090036: return "Max < Min";
        case 0x060A0023: return "No SDO resource";
        case 0x08000000: return "General error";
        case 0x08000020: return "Data transfer error";
        case 0x08000021: return "Local control error";
        case 0x08000022: return "Wrong device state";
        case 0x08000023: return "Object dictionary error";
        case 0x08000024: return "No data available";
        default: return "Unknown SDO Abort";
    }
}

bool nodeID_filter(int node_id, thread_args_t* args)
{
    if (!args->node_filter_enabled)
    {
        return true;
    }

    for (int i = 0; i < args->nodeID_count; i++)
    {
        if (args->nodeID_arr[i] == node_id)
        {
            return true;
        }
    }
    return false;
}

bool commObj_filter(int cob, thread_args_t* args)
{
    if (!args->object_filter_enabled)
    {
        return true;
    }

    // COB 0x0 (0) = NMT Network Management
    if (cob == 0x0)
    {
        return (args->object_mask & OBJ_NMT);
    }

    // COB 0x3 bis 0xA (3-10) = PDO (TPDO & RPDO)
    // Bereich 0x180 bis 0x57F
    if (cob >= 0x3 && cob <= 0xA)
    {
        return (args->object_mask & OBJ_PDO);
    }

    // COB 0xB und 0xC (11-12) = SDO (Transmit & Receive)
    // Bereich 0x580 bis 0x67F
    if (cob == 0xB || cob == 0xC)
    {
        return (args->object_mask & OBJ_SDO);
    }

    // COB 0xE (14) = Heartbeat / Node Guarding
    if (cob == 0xE)
    {
        return (args->object_mask & OBJ_NMT);
    }

    // Für alle anderen (SYNC, TIME, EMCY etc.)
    return false; 
}


void parse_node_filter(thread_args_t* cfg, char* value)
{
    if (strstr(value, "FF")) {
        cfg->node_filter_enabled = false;
        return;
    }

    cfg->node_filter_enabled = true;

    char* token = strtok(value, "[, ]");
    while (token)
    {
        int id = (int)strtol(token, NULL, 16);
        if (id >= 1 && id <= 127)
        {
            cfg->nodeID_arr[cfg->nodeID_count++] = id;
        }
        token = strtok(NULL, "[, ]");
    }
}

void parse_object_filter(thread_args_t* cfg, char* value)
{
    cfg->object_mask = 0;

    if (strcasestr(value, "all"))
    {
        cfg->object_filter_enabled = false;
        return;
    }

    cfg->object_filter_enabled = true;

    if (strcasestr(value, "sdo")) cfg->object_mask |= OBJ_SDO;
    if (strcasestr(value, "pdo")) cfg->object_mask |= OBJ_PDO;
    if (strcasestr(value, "nmt")) cfg->object_mask |= OBJ_NMT;
}

/*-------------------------------------------------------------------------------
 * Function: load_config
 * ------------------------------------------------------------------------------
 */
void load_config(thread_args_t* gui_args)
{
    FILE* file = fopen("CANopenMonitor_conf.txt", "r");
    if (!file) {
        perror("Config file open failed");
        return;
    }

    char line[256];

    while (fgets(line, sizeof(line), file))
    {
        char key[128] = {0};
        char value[128] = {0};

        if (sscanf(line, " %127[^= ] = %127[^;]", key, value) == 2)
        {
            if (strcmp(key, "timeout_heartbeat_ms") == 0)
            {
                long tmp = atol(value);
                if (tmp > 1)
                    gui_args->timeout_ms = (time_t)tmp;
            }
            else if (strcmp(key, "timeout_remove_ms") == 0)
            {
                long tmp = atol(value);
                if (tmp > 1)
                    gui_args->remove_timeout_ms = (time_t)tmp;
            }
            else if (strcmp(key, "nodeid_filter") == 0)
            {
                parse_node_filter(gui_args, value);
            }
            else if (strcmp(key, "object_filter") == 0)
            {
                parse_object_filter(gui_args, value);
            }
            else if (strcmp(key, "log_enable") == 0)
            {
                gui_args->log_enabled = (bool)atoi(value);
            }
        }
    }

    fclose(file);
}

/*-------------------------------------------------------------------------------
 * Thread: CANopenReader
 * ------------------------------------------------------------------------------
 */
void* thread_CANopenReader (void* arg) 
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    CANopenMessage msg;

    mqd_t mq = mq_open(CAN_QUEUE, O_WRONLY);
    if (mq == (mqd_t)-1) {
        perror("[CAN Thread] mq_open failed");
        return NULL;
    }

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("[CAN Thread] socket failed"); return NULL; }

    strcpy(ifr.ifr_name, "can0");
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("[CAN Thread] ioctl failed"); return NULL; }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("[CAN Thread] bind failed"); return NULL; }

    while (1)
    {
        if (read(s, &frame, sizeof(frame)) > 0)
        {
            msg.timestamp = get_time_ms();
            int cob_id = frame.can_id & 0x7FF;
            msg.cob = cob_id >> 7;
            msg.id = cob_id & 0x7F;
            msg.len = frame.can_dlc;
            for (int i = 0; i < msg.len; i++) {
                msg.data[i] = frame.data[i];
            }

            mq_send(mq, (const char*)&msg, sizeof(CANopenMessage), 0);
        }
    }
    return NULL;
}

/*-------------------------------------------------------------------------------
 * GUI-Thread
 * ------------------------------------------------------------------------------
 */
void* thread_gui(void* arg) 
{    
    thread_args_t* gui_args = (thread_args_t*)arg;
    const time_t timeout_Heartbeat_ms = gui_args->timeout_ms;
    const time_t timeout_remove_ms = gui_args->remove_timeout_ms;

    int *node_filter_arr = gui_args->nodeID_arr;
    int node_filter_count = gui_args->nodeID_count;
    bool node_filter_enabled = gui_args->node_filter_enabled;

    int object_mask = gui_args->object_mask;
    bool object_filter_enabled = gui_args->object_filter_enabled;
    bool node_log_enabled = gui_args->log_enabled;

    

    typedef struct 
    {
        bool active;
        bool visible;
        uint8_t status;

        uint64_t time_lastHB;
        uint64_t time_lastError;

        char error_buf[64];
        const char* error_desc;
    } CANNode;

    CANopenMessage msg;
    ssize_t bytes;
    bool updated = false;
    bool state_changed = false;

    CANNode node_list[128] = {0};
    static time_t time_last_update = 0;

    mqd_t mq_can  = mq_open(CAN_QUEUE,  O_RDONLY | O_NONBLOCK);
    if (mq_can == (mqd_t)-1) perror("[GUI] CAN mq_open failed");

    // Log init
    FILE *logfile = NULL;
    if(node_log_enabled)
    {
        char filename[128];
        //char time_str[16];
        time_t time_now = time(NULL);
        struct tm *t = localtime(&time_now); 
        t->tm_hour = t->tm_hour + 2;
        //format_time(time_str, sizeof(time_str), get_time_ms());

        strftime(filename, sizeof(filename),
            "CANopen_log_%Y-%m-%d_%H-%M-%S.txt", t);

        logfile = fopen(filename, "a");
        if (!logfile) {
            perror("Fehler beim Öffnen der Logdatei");
            printf("Logging error File open failed\n");
            exit(1);
        }
    }

    // GUI INIT
    const char* state_color;
    const char* emcy_color;
    WINDOW *cursesWin = initscr();
    CDKSCREEN *cdkscreen = initCDKScreen(cursesWin);
    initCDKColor();
    curs_set(0);

    int width  = COLS - 2;
    int spacer = 2;
    int hNode = 16;
    int hEm   = 12;
    int hData = LINES - hNode - hEm - (spacer * 2) - 5; 
    if (hData < 3) hData = 3;
    int yNode = 1;

    CDKSWINDOW *nodeWin = newCDKSwindow(cdkscreen, 1, yNode, hNode, width,
        "</B>NodeID | Status          | Error                               | LastError  | LastHB</B>", 100, TRUE, FALSE);

    int yEm = yNode + hNode + spacer; 
    CDKSWINDOW *emWin = newCDKSwindow(cdkscreen, 1, yEm, hEm, width, 
        "</B>Emergency-Messages:</B>", 100, TRUE, FALSE);

    int yData = yEm + hEm + spacer;
    CDKSWINDOW *dataWin = newCDKSwindow(cdkscreen, 1, yData, hData, width, 
        "</B>Data:</B>", 500, TRUE, FALSE);

    refreshCDKScreen(cdkscreen);
    /*
    char test_msg[256];

    snprintf(test_msg, sizeof(test_msg),
            "timeout_heartbeat_ms = %ld ms",
            (long)timeout_Heartbeat_ms);
    addCDKSwindow(dataWin, test_msg, BOTTOM);

    snprintf(test_msg, sizeof(test_msg),
            "node_filter_enabled = %s, count = %d",
            node_filter_enabled ? "true" : "false",
            node_filter_count);
    addCDKSwindow(dataWin, test_msg, BOTTOM);

    snprintf(test_msg, sizeof(test_msg),
            "object_filter_enabled = %s, object_mask = 0x%X",
            object_filter_enabled ? "true" : "false",
            object_mask);
    addCDKSwindow(dataWin, test_msg, BOTTOM);

    refreshCDKScreen(cdkscreen);
    */
    while (1) 
    {
        updated = false;

        while ((bytes = mq_receive(mq_can, (char*)&msg, sizeof(CANopenMessage), NULL)) > 0)
        {
            if (bytes == -1 && errno != EAGAIN) perror("mq_receive error");
            unsigned int data[8];

            // Hartbeat Messages
            if (msg.cob == 0xE)
            {
                node_list[msg.id].visible = true;
                node_list[msg.id].active = true;
                node_list[msg.id].status = msg.data[0];
                node_list[msg.id].time_lastHB = msg.timestamp;
                updated = true;
                if(node_log_enabled)
                {
                    char time_str[16];
                    format_time(time_str, sizeof(time_str), msg.timestamp);
                    fprintf(logfile, "Node %03X: %02X | %s\n", (msg.id | (msg.cob<<7)), msg.data[0], time_str);
                }
            }

            // Emergency Messages
            else if (msg.cob == 0x1)
            {
                char em_msg[256];
                char time_str[16];
                char raw_data[64];

                snprintf(raw_data, sizeof(raw_data),
                    "%02X %02X %02X %02X %02X %02X %02X %02X",
                    msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                    msg.data[4], msg.data[5], msg.data[6], msg.data[7]);

                format_time(time_str, sizeof(time_str), msg.timestamp);
                
                uint16_t error_code = ((uint16_t)msg.data[1] << 8) | msg.data[0];
                const char* error_desc = get_emergencyErrorDescription(error_code);

                snprintf(node_list[msg.id].error_buf, sizeof(node_list[msg.id].error_buf),
                        "EMCY: %s", error_desc);
                node_list[msg.id].error_desc = node_list[msg.id].error_buf;
                node_list[msg.id].time_lastError = msg.timestamp;

                snprintf(em_msg, sizeof(em_msg),
                        "Node %d: EMCY: %-30.30s | %s | %s",
                        msg.id, error_desc, raw_data, time_str);

                addCDKSwindow(emWin, em_msg, BOTTOM);
                updated = true;
                if(node_log_enabled)
                {
                    fprintf(logfile, "Node %03X: %s | %s\n", (msg.id | (msg.cob<<7)), raw_data, time_str);
                }
            }

            // SDO Abort Messages
            else if (msg.cob == 0xB)
            {
                if (msg.data[0] == 0x80)
                {
                    char buffer[256];
                    char time_str[16];
                    char raw_data[64];

                    snprintf(raw_data, sizeof(raw_data),
                        "%02X %02X %02X %02X %02X %02X %02X %02X",
                        msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                        msg.data[4], msg.data[5], msg.data[6], msg.data[7]);

                    format_time(time_str, sizeof(time_str), msg.timestamp);

                    uint32_t abort_code = ((uint32_t)msg.data[7] << 24) |
                                          ((uint32_t)msg.data[6] << 16) |
                                          ((uint32_t)msg.data[5] << 8)  |
                                          ((uint32_t)msg.data[4]);

                    const char* desc = get_sdoAbortDescription(abort_code);

                    snprintf(node_list[msg.id].error_buf, sizeof(node_list[msg.id].error_buf),
                            "SDO Abort: %s", desc);
                    node_list[msg.id].error_desc = node_list[msg.id].error_buf;
                    node_list[msg.id].time_lastError = msg.timestamp;

                    snprintf(buffer, sizeof(buffer),
                        "Node %d: SDO Abort: %-25.25s | %s | %s", 
                        msg.id, desc, raw_data, time_str);
                    addCDKSwindow(emWin, buffer, BOTTOM);
                    updated = true;
                    if(node_log_enabled)
                    {
                        fprintf(logfile, "Node %03X: %s | %s\n", (msg.id | (msg.cob<<7)), raw_data, time_str);
                    }
                }
            }

            // All other Data Messages
            else
            {    
                if (!nodeID_filter(msg.id, gui_args) || !commObj_filter(msg.cob, gui_args))    { continue; }
                else
                {
                    char buffer[256];
                    char raw_data[64];
                    char time_str[16];

                    snprintf(raw_data, sizeof(raw_data),
                        "%02X %02X %02X %02X %02X %02X %02X %02X",
                        msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                        msg.data[4], msg.data[5], msg.data[6], msg.data[7]);

                    format_time(time_str, sizeof(time_str), msg.timestamp);
            
                    snprintf(buffer, sizeof(buffer),
                        "Node %d: %-35.35s | %s",
                        msg.id,
                        raw_data,
                        time_str);
                    
                    addCDKSwindow(dataWin, buffer, BOTTOM);
                    updated = true;
                    if(node_log_enabled)
                    {
                        fprintf(logfile, "Node %03X: %s | %s\n", (msg.id | (msg.cob<<7)), raw_data, time_str);
                    }
                }
            }
        }

        if(get_time_ms() - time_last_update > 1000) //minimaler Update-Intervall --> aktualisiert auch die Heartbeat-Statusanzeige, wenn kein neues CAN-Message ankommt
        {
            updated = true;
            state_changed = true;
        }

        if ((updated || state_changed) && get_time_ms() - time_last_update > GUI_UPDATE_INTERVAL_MS)
        {
            cleanCDKSwindow(nodeWin);

            for (int i = 1; i < 128; i++) 
            {
                if (node_list[i].visible) 
                {
                    uint64_t age = get_time_ms() - node_list[i].time_lastHB;

                    if (age > timeout_remove_ms)
                    {
                        node_list[i].visible = false;
                        node_list[i].active = false;
                        state_changed = true;
                        continue;
                    }

                    if (age > timeout_Heartbeat_ms)
                    {
                        node_list[i].active = false;
                        state_changed = true;
                    }

                    char row[128];
                    char hb_time[16];
                    char err_time[16];

                    format_time(hb_time, sizeof(hb_time), node_list[i].time_lastHB);
                    format_time(err_time, sizeof(err_time), node_list[i].time_lastError);

                    const char* state_name;

                    if (node_list[i].active)
                    {
                        state_name = get_nodeStatus(node_list[i].status);
                        state_color = "24";
                    }
                    else
                    {
                        state_name = "INACTIVE";
                        state_color = "02";
                    }

                    const char* error = node_list[i].error_desc ? node_list[i].error_desc : "EMCY: No Error / Reset";

                    if(strcmp(error, "EMCY: No Error / Reset") == 0)
                    {
                        emcy_color = "24";   // grün
                    }
                    else
                    {
                        emcy_color = "02";   // rot
                    }

                    snprintf(row, sizeof(row),
                        "%-6d | </%s>%-15.15s<!%s> | </%s>%-35.35s<!%s> | %-10s | %-10s",
                        i,
                        state_color,
                        state_name,
                        state_color,
                        emcy_color,
                        error,
                        emcy_color,
                        err_time,
                        hb_time
                    );
                        
                    addCDKSwindow(nodeWin, row, BOTTOM);
                }
            }

            time_last_update = get_time_ms();
            refreshCDKScreen(cdkscreen);
            if(node_log_enabled)
            {
                fflush(logfile);
            }
            state_changed = false;
        }

        usleep(10000);
    }

    endCDK();
    if (logfile)
    {
        fclose(logfile);
    }
    return NULL;
}

/*-------------------------------------------------------------------------------
 * MAIN
 * ------------------------------------------------------------------------------
 */
int main() 
{
    thread_args_t gui_args;
    memset(&gui_args, 0, sizeof(gui_args));

    gui_args.timeout_ms = 1000;
    gui_args.remove_timeout_ms = 5000;

    gui_args.node_filter_enabled = false;
    gui_args.object_filter_enabled = false;
    gui_args.object_mask = OBJ_SDO | OBJ_PDO | OBJ_NMT;

    load_config(&gui_args);

    pthread_t t1, t2;
    
    mq_unlink(CAN_QUEUE);

    struct mq_attr attr;
    attr.mq_flags   = 0;
    attr.mq_maxmsg  = 80;
    attr.mq_msgsize = sizeof(CANopenMessage);
    attr.mq_curmsgs = 0;

    mqd_t mq_can  = mq_open(CAN_QUEUE,  O_CREAT | O_RDWR | O_NONBLOCK, 0644, &attr);
    
    if (mq_can == (mqd_t)-1) perror("mq_can open failed");

    pthread_create(&t1, NULL, thread_CANopenReader, NULL);
    pthread_create(&t2, NULL, thread_gui, &gui_args);

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);

    mq_close(mq_can); 
    mq_unlink(CAN_QUEUE);

    return 0;
}
