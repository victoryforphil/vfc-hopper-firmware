#include "state/state.h"
#include "actions/actions.h"
#include "actions/hover.h"


void act_hover_begin(VFCHoverParams params){
    VFC_State* state = vfc_state_get();
    VFCHoverState hover_state = {0};
    while(!hover_state.finished && !hover_state.failed){

        uint8_t target = params.height;

        uint8_t current = state->height;

        float errorHeight = current/target;
        float errorPos_x = state->pos_x;
        float errorPos_y = state->pos_y;

        hover_state.delta = errorHeight;
        hover_state.height = state->height;
        hover_state.speed = state->acc_z;
        

        if(errorHeight < params.margin){
            hover_state.finished = 1;
        }




    }
}