#ifndef VFC_HARDWARE_TELEMETRY_H_
#define VFC_HARDWARE_TELEMETRY_H_


#define TELEM_BUFFER_SIZE 1024

int telem_init(void);
int telem_log_info(char*, uint32_t);

#endif