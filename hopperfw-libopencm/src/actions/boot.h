#ifndef _VFC_ACTION_BOOT_H_
#define _VFC_ACTION_BOOT_H_


struct VFCBootState_t{
    bool boot_finished;
    bool boot_success;
    bool boot_failure;
}
uint8_t act_boot();
VFCBootState_t act_boot_state();

#endif