
#include "state/state.h"

VFC_State* _vfc_state;

void vfc_state_init(){
    VFC_State state;
    state.pos_x = 0;
    state.pos_y = 0;
    state.height = 0;
    state.acc_x = 0;
    state.acc_y = 0;
    state.acc_z = 0;
    state.vel_x = 0;
    state.vel_y = 0;
    state.vel_z = 0;
    state.time = 0;
    state.battery = 0;
    _vfc_state = &state;
}

VFC_State* vfc_state_get(){
    return _vfc_state;
}