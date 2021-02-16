#include "df_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(df);

HAL_PIN(vel_in);
HAL_PIN(vel_out);
HAL_PIN(vel_lpf);
HAL_PIN(polecount);

HAL_PIN(torque);
HAL_PIN(iq_cmd);
HAL_PIN(psi);

HAL_PIN(error);
HAL_PIN(state);

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr){
  struct df_pin_ctx_t *pins = (struct df_pin_ctx_t *)pin_ptr;
  PIN(polecount) = 15.0;
  PIN(vel_lpf) = 0.1;
  PIN(psi) = 0.02;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct df_ctx_t * ctx = (struct df_ctx_t *)ctx_ptr;
  struct df_pin_ctx_t *pins = (struct df_pin_ctx_t *)pin_ptr;
  
  PIN(vel_out) = PIN(vel_out) * (1.0 - PIN(vel_lpf)) + PIN(vel_lpf) * PIN(vel_in) * PIN(polecount);
  PIN(iq_cmd) = PIN(torque) / PIN(polecount) / PIN(psi) / 1.5;
  
  PIN(state) = 1.0;
  if(PIN(error) > 0.0){
    PIN(state) = 0.0;
  }
}

hal_comp_t df_comp_struct = {
    .name      = "df",
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
    .pin_count = sizeof(struct df_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
