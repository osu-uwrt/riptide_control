#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include "send_thruster_cmd.h"

#define CAN_INTERFACE_NAME "can0"
#define CAN_ID_ESC_BOARD0 2
#define CAN_ID_ESC_BOARD1 3

// Note these have been stolen from titan_firmware/lib/titan_canmore/.../protocol.h
// Please be sure these match the headers in that file
#define CANMORE_TITAN_CHAN_THRUSTER_CMDS 0
#define CANMORE_CRC_LENGTH 18
#define CANMORE_NOC_LENGTH 4
#define CANMORE_DIRECTION_LENGTH 1
#define CANMORE_TYPE_LENGTH 1
#define CANMORE_CLIENT_ID_LENGTH 5
#define CANMORE_STD_NOC_OFFSET 0
#define CANMORE_STD_DIRECTION_OFFSET (CANMORE_STD_NOC_OFFSET + CANMORE_NOC_LENGTH)
#define CANMORE_STD_TYPE_OFFSET (CANMORE_STD_DIRECTION_OFFSET + CANMORE_DIRECTION_LENGTH)
#define CANMORE_STD_CLIENT_ID_OFFSET (CANMORE_STD_TYPE_OFFSET + CANMORE_TYPE_LENGTH)
#define CANMORE_DIRECTION_AGENT_TO_CLIENT 1
#define CANMORE_TYPE_UTIL 1

#define CANMORE_CALC_STD_ID(client_id, type, direction, noc)                                                           \
    ((((client_id) & ((1u << CANMORE_CLIENT_ID_LENGTH) - 1)) << CANMORE_STD_CLIENT_ID_OFFSET) |                        \
     (((type) & ((1u << CANMORE_TYPE_LENGTH) - 1u)) << CANMORE_STD_TYPE_OFFSET) |                                      \
     (((direction) & ((1u << CANMORE_DIRECTION_LENGTH) - 1u)) << CANMORE_STD_DIRECTION_OFFSET) |                       \
     (((noc) & ((1u << CANMORE_NOC_LENGTH) - 1u)) << CANMORE_STD_NOC_OFFSET))

#define CANMORE_CALC_THRUSTER_CMD_ID(client_id) \
    CANMORE_CALC_STD_ID(client_id, CANMORE_TYPE_UTIL, CANMORE_DIRECTION_AGENT_TO_CLIENT, CANMORE_TITAN_CHAN_THRUSTER_CMDS)




static int socket_fd = -1;

static void report_init_error(const char* func_name, int err_val) {
    fprintf(stderr, "[send_thruster_cmd_canbus] Failed to initialize! Function '%s' failed with error: %s (%d)\n",
            func_name, strerror(err_val), err_val);
}


static void report_tx_error(const char* func_name, int err_val) {
    fprintf(stderr, "[send_thruster_cmd_canbus] Failed to transmit! Function '%s' failed with error: %s (%d)\n",
            func_name, strerror(err_val), err_val);
}

static int init_can_socket(void) {
    int last_err = 0;

    // Close an existing socket if open
    if (socket_fd >= 0) {
        int last_fd = socket_fd;
        socket_fd = -1;
        if (close(last_fd)) {
            last_err = errno;
            report_init_error("close(socket_fd)", last_err);
            goto end;
        }
    }

    // Get CAN bus interface index
    int if_index = if_nametoindex(CAN_INTERFACE_NAME);
    if (if_index == 0) {
        last_err = errno;
        report_init_error("if_nametoindex(\"" CAN_INTERFACE_NAME "\")", last_err);
        goto end;
    }

    // Open socket
    socket_fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (socket_fd < 0) {
        last_err = errno;
        report_init_error("socket(...)", last_err);
        goto end;
    }

    // Bind to requested interface
    struct sockaddr_can addr = {0};
    addr.can_family = AF_CAN;
    addr.can_ifindex = if_index;

    if (bind(socket_fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        last_err = errno;
        report_init_error("bind(...)", last_err);
        goto end;
    }

end:
    if (last_err != 0 && socket_fd >= 0) {
        close(socket_fd);
        socket_fd = -1;
    }

    return last_err;
}

static void populate_can_frame(struct can_frame *frame, int client_id, int16_t cmds[4]) {
    frame->can_dlc = 8;
    frame->can_id = CANMORE_CALC_THRUSTER_CMD_ID(client_id);

    for (int i = 0; i < 4; i++) {
        size_t idx = i * 2;
        uint16_t cmd_network = (uint16_t) cmds[i];
        frame->data[idx] = cmd_network >> 8;
        frame->data[idx + 1] = cmd_network & 0xFF;
    }
}

int send_thruster_cmd_canbus(int16_t cmds[8]) {
    if(socket_fd == -2)
    {
        return 0;
    }
    
    if (socket_fd == -1) {
        printf("Initializing CAN socket\n");
        int err = init_can_socket();
        if (err) 
        {
            socket_fd = -2;
            return err;
        }
    }

    // Send commands to board 0

    struct can_frame frame;
    populate_can_frame(&frame, CAN_ID_ESC_BOARD0, cmds);

    if (write(socket_fd, &frame, sizeof(frame)) != sizeof(frame)) {
        int last_err = errno;
        report_tx_error("write(BOARD0, ...)", last_err);
        return last_err;
    }

    // Send commands to board 1
    populate_can_frame(&frame, CAN_ID_ESC_BOARD1, &cmds[4]);

    if (write(socket_fd, &frame, sizeof(frame)) != sizeof(frame)) {
        int last_err = errno;
        report_tx_error("write(BOARD1, ...)", last_err);
        return last_err;
    }

    return 0;
}

#ifdef ENABLE_THRUSTER_TEST

// Thruster Test Script Below this Line

void stop_handler () {
    int16_t cmd[8] = {0};
    if (socket_fd >= 0) {
        send_thruster_cmd_canbus(cmd);
    }
    const char msg[] = " - INTERRUPT! Stopping Thrusters\n";
    write(STDOUT_FILENO, msg, sizeof(msg) - 1);
    _exit(0);
}

#include <signal.h>

int main(void) {
    // Do nothing
    signal(SIGINT, stop_handler);

    int16_t cmd[8] = {0};

    while(1) {
        for (int i = 0; i < 8; i++) {
            printf("Testing Thruster %d...\n", i);
            cmd[i] = 300;
            for (int j = 0; j < 100; j++) {
                if (send_thruster_cmd_canbus(cmd)) return 1;
                usleep(30 * 1000); //100 * 30000 = 3000000us = 3s
            }
            cmd[i] = 0;
        }
    }

    printf("Done!\n");
    return send_thruster_cmd_canbus(cmd) != 0;
}

#endif //ENABLE_THRUSTER_TEST
