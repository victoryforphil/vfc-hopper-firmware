/*
 * Logging.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: Victo
 */



void log_info(char* str){
	SEGGER_SYSVIEW_Print(str);
}
