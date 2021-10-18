#ifndef _VFC_ACTION_HOVER_H_
#define _VFC_ACTION_HOVER_H_
/*
Action Desc: Command the UAV to hover at a desired height and maitain position

Action Params:
    - height: desired height
    - hold_duration: time maintain hover before finishing
    - speed: how fast to get to desired height
    - margin: how close to the desired height to be considered at the desired height
*/

struct VFCHoverParams_t{
    float height;
    float hold_duration;
    float speed;
    float margin;
};

struct VFCHoverState_t{
    float height;
    float delta;
    float speed;
    bool finished;
    bool failed;
    bool holding;
}

uint8_t act_hover_tick(VFCHoverParams_t params, VFCHoverState_t *state, float dt);

VFCHoverState_t act_hover_


#endif;