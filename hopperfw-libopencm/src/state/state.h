#ifndef _VFC_STATE_H_
#define _VFC_STATE_H_

#include "actions/actions.h"

typedef struct VFC_State_t{
    float pos_x;
    float pos_y;
    float height;
    float acc_x;
    float acc_y;
    float acc_z;
    float vel_x;
    float vel_y;
    float vel_z;
    float time;
    float battery;
} VFC_State;


void vfc_state_init();

VFC_State* vfc_state_get();

#endif