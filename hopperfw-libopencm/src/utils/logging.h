#ifndef _VFC_LOGGING_H_
#define _VFC_LOGGING_H_



//void log_printf(char* source, char* function, char* msg, LogLevel level);

void log_info(char* source, char* function, char* msg);
void log_success(char* source, char* function, char* msg);
void log_warn(char* source, char* function, char* msg);
void log_error(char* source, char* function, char* msg);
void log_debug(char* source, char* function, char* msg);

#endif