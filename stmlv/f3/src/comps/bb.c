#include "bb_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(bb);

HAL_PIN(left_in);
HAL_PIN(mid_in);
HAL_PIN(right_in);
HAL_PIN(mode_in);

HAL_PIN(left_offset);
HAL_PIN(left_gain);
HAL_PIN(mid_offset);
HAL_PIN(mid_gain);
HAL_PIN(right_offset);
HAL_PIN(right_gain);

HAL_PIN(mode_th);

HAL_PIN(lpf);
HAL_PIN(left_lp);
HAL_PIN(mid_lp);
HAL_PIN(right_lp);
HAL_PIN(mode_lp);

HAL_PIN(ratio);
HAL_PIN(damping);
HAL_PIN(diff);

HAL_PIN(torque);
HAL_PIN(front_vel);
HAL_PIN(back_vel);
HAL_PIN(vel);

HAL_PIN(vel_back_right);
HAL_PIN(vel_back_left);
HAL_PIN(vel_front_right);
HAL_PIN(vel_front_left);

HAL_PIN(state_back_right);
HAL_PIN(state_back_left);
HAL_PIN(state_front_right);
HAL_PIN(state_front_left);

HAL_PIN(en_out);

HAL_PIN(torque_back_right);
HAL_PIN(torque_back_left);
HAL_PIN(torque_front_right);
HAL_PIN(torque_front_left);

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr){
  struct bb_pin_ctx_t *pins = (struct bb_pin_ctx_t *)pin_ptr;
  PIN(left_offset) = 0.5;
  PIN(mid_offset) = 0.5;
  PIN(right_offset) = 0.5;
  PIN(mode_th) = 2.0;
  PIN(left_gain) = 2.0;
  PIN(mid_gain) = 2.0;
  PIN(right_gain) = 2.0;

  PIN(lpf) = 0.1;
  PIN(ratio) = 0.5;
  PIN(diff) = 2.0 * M_PI; // 1Nm / (2.0 * M_PI rad) / s
  PIN(damping) = 0.00002; // Nm / rad / s
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct bb_ctx_t * ctx = (struct bb_ctx_t *)ctx_ptr;
  struct bb_pin_ctx_t *pins = (struct bb_pin_ctx_t *)pin_ptr;
  
  PIN(left_lp) = PIN(left_lp) * (1.0 - PIN(lpf)) + (PIN(left_in) - PIN(left_offset)) * PIN(left_gain) * PIN(lpf);
  PIN(mid_lp) = PIN(mid_lp) * (1.0 - PIN(lpf)) + (PIN(mid_in) - PIN(mid_offset)) * PIN(mid_gain) * PIN(lpf);
  PIN(right_lp) = PIN(right_lp) * (1.0 - PIN(lpf)) + (PIN(right_in) - PIN(right_offset)) * PIN(right_gain) * PIN(lpf);
  PIN(mode_lp) = PIN(mode_lp) * (1.0 - PIN(lpf)) + PIN(mode_in) * PIN(lpf);

  PIN(front_vel) = (PIN(vel_front_right) + PIN(vel_front_left)) / 2.0;
  PIN(back_vel) = (PIN(vel_back_right) + PIN(vel_back_left)) / 2.0;
  PIN(vel) = (PIN(front_vel) + PIN(back_vel)) / 2.0;

  PIN(torque) = (PIN(right_lp) - PIN(left_lp) - PIN(vel) * PIN(damping)) * 2.0;

  if(PIN(mode_lp) > PIN(mode_th)){ // drift mode
    PIN(ratio) = 0.8;
    PIN(torque_back_right) = PIN(torque) * PIN(ratio) - (PIN(back_vel) - PIN(vel_back_right) * PIN(diff));
    PIN(torque_back_left) = PIN(torque) * PIN(ratio) - (PIN(back_vel) - PIN(vel_back_left) * PIN(diff));
    PIN(torque_front_right) = PIN(torque) * (1.0 - PIN(ratio)) - (PIN(front_vel) - PIN(vel_front_right) * PIN(diff));
    PIN(torque_front_left) = PIN(torque) * (1.0 - PIN(ratio)) - (PIN(front_vel) - PIN(vel_front_left) * PIN(diff));
  }
  else{ // 4x4 diff lock mode
    PIN(ratio) = 0.5;
    PIN(torque_back_right) = PIN(torque) * PIN(ratio) - (PIN(vel) - PIN(vel_back_right) * PIN(diff));
    PIN(torque_back_left) = PIN(torque) * PIN(ratio) - (PIN(vel) - PIN(vel_back_left) * PIN(diff));
    PIN(torque_front_right) = PIN(torque) * (1.0 - PIN(ratio)) - (PIN(vel) - PIN(vel_front_right) * PIN(diff));
    PIN(torque_front_left) = PIN(torque) * (1.0 - PIN(ratio)) - (PIN(vel) - PIN(vel_front_left) * PIN(diff));
  }

  PIN(en_out) = 1.0;
  if(PIN(state_back_right) + PIN(state_back_left) + PIN(state_front_right) + PIN(state_front_left) < 4.0){
    PIN(en_out) = 0.0; 
  }
}

hal_comp_t bb_comp_struct = {
    .name      = "bb",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct bb_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
