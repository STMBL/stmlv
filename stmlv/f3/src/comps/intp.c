#include "intp_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(intp);

HAL_PIN(pos_in);
HAL_PIN(pos_in_old);
HAL_PIN(vel);
HAL_PIN(pos_out);
HAL_PIN(max_time);
HAL_PIN(vel_timer);
HAL_PIN(max_pos_diff);
HAL_PIN(res);

extern volatile uint32_t intp_buffer[3 * 3 * 8 + 3];

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr){
  struct intp_pin_ctx_t *pins = (struct intp_pin_ctx_t *)pin_ptr;
  PIN(res) = 6;
  PIN(max_time) = 0.01;
  PIN(max_pos_diff) = 2.0 * M_PI / PIN(res) * 2.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct intp_ctx_t * ctx = (struct intp_ctx_t *)ctx_ptr;
  struct intp_pin_ctx_t *pins = (struct intp_pin_ctx_t *)pin_ptr;
  float pos_diff = minus(PIN(pos_in), PIN(pos_in_old));

  PIN(vel_timer) += period;
  PIN(pos_out) = mod(PIN(pos_out) + PIN(vel) * period);

  if(PIN(vel_timer) > PIN(max_time)){
    PIN(vel_timer) = PIN(max_time);
    PIN(vel) = 0.0;
    PIN(pos_out) = PIN(pos_in);
  }

  if((ABS(pos_diff) > 0.0) & (ABS(pos_diff) < PIN(max_pos_diff))){
    PIN(vel) = pos_diff / PIN(vel_timer);
    PIN(vel_timer) = 0.0;
    PIN(pos_out) = PIN(pos_in) - SIGN(PIN(vel) * M_PI / PIN(res));
    PIN(pos_in_old) = PIN(pos_in);
  }
}

hal_comp_t intp_comp_struct = {
    .name      = "intp",
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
    .pin_count = sizeof(struct intp_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
