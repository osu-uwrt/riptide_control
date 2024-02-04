#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C"
#endif 
int send_thruster_cmd_canbus(int16_t cmds[8]);
