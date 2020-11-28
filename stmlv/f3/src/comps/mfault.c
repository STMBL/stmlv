#include "mfault_comp.h"
#include "commands.h"
#include "defines.h"
#include "hal.h"
#include "hw.h"
#include "main.h"

HAL_COMP(mfault);

HAL_PIN(en);

HAL_PIN(dc);
HAL_PIN(iu);
HAL_PIN(iv);
HAL_PIN(iw);

HAL_PIN(temp);
HAL_PIN(mot_temp);


HAL_PIN(en_out);
HAL_PIN(scale);
HAL_PIN(max_cur_out);


HAL_PIN(max_cur);
HAL_PIN(abs_max_cur);
HAL_PIN(max_dc);
HAL_PIN(min_dc);
HAL_PIN(max_temp);
HAL_PIN(max_mot_temp);

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct mfault_ctx_t * ctx = (struct mfault_ctx_t *)ctx_ptr;
  struct mfault_pin_ctx_t *pins = (struct mfault_pin_ctx_t *)pin_ptr;

  PIN(max_cur) = 25.0;
  PIN(max_dc) = 75.0;
  PIN(min_dc) = 18.0;
  PIN(max_temp) = 70.0;
  PIN(max_mot_temp) = 70.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct mfault_ctx_t * ctx = (struct mfault_ctx_t *)ctx_ptr;
  struct mfault_pin_ctx_t *pins = (struct mfault_pin_ctx_t *)pin_ptr;
  PIN(en_out) = PIN(en);

  if(MAX3(ABS(PIN(iu)), ABS(PIN(iv)), ABS(PIN(iw))) > PIN(abs_max_cur) || PIN(dc) < PIN(min_dc) || PIN(temp) > PIN(max_temp)){
    PIN(en_out) = 0.0;
  }
  else if(MAX3(ABS(PIN(iu)), ABS(PIN(iv)), ABS(PIN(iw))) > PIN(max_cur) * 1.1 || PIN(dc) > PIN(max_dc)){
    PIN(en_out) = -1.0;
  }

  PIN(scale) = MAX(MIN(1.0 - (PIN(temp) - PIN(max_temp) + 10.0) / 10.0, 1.0), 0.0);
  PIN(scale) = MAX(MIN(1.0 - (PIN(mot_temp) - PIN(max_mot_temp) + 10.0) / 10.0, PIN(scale)), 0.0);

  PIN(max_cur_out) = PIN(max_cur) * PIN(scale);
}

hal_comp_t mfault_comp_struct = {
    .name      = "mfault",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct mfault_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
