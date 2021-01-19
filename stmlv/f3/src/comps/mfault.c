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

HAL_PIN(hv_error);
HAL_PIN(vel);

HAL_PIN(error);
HAL_PIN(en_out);
HAL_PIN(scale);
HAL_PIN(max_cur_out);


HAL_PIN(max_cur);
HAL_PIN(abs_max_cur);
HAL_PIN(max_dc);
HAL_PIN(min_dc);
HAL_PIN(max_temp);
HAL_PIN(max_mot_temp);
HAL_PIN(max_vel);

void disable(char *ptr) {
  hal_parse("mfault0.en = 0");
}
COMMAND("disable", disable, "disable STMLV");

void enable(char *ptr) {
  hal_parse("mfault0.en = 1");
}
COMMAND("enable", enable, "enable STMLV");

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct mfault_ctx_t * ctx = (struct mfault_ctx_t *)ctx_ptr;
  struct mfault_pin_ctx_t *pins = (struct mfault_pin_ctx_t *)pin_ptr;

  PIN(max_cur) = 40.0;
  PIN(abs_max_cur) = 60.0;
  PIN(max_dc) = 75.0;
  PIN(min_dc) = 18.0;
  PIN(max_temp) = 70.0;
  PIN(max_mot_temp) = 70.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct mfault_ctx_t * ctx = (struct mfault_ctx_t *)ctx_ptr;
  struct mfault_pin_ctx_t *pins = (struct mfault_pin_ctx_t *)pin_ptr;
  PIN(en_out) = PIN(en);

  if(PIN(en) <= 0.0){
    PIN(error) = 0;
  }

  if(MAX3(ABS(PIN(iu)), ABS(PIN(iv)), ABS(PIN(iw))) > PIN(abs_max_cur)){ // short
    PIN(en_out) = 0.0;
    PIN(error) = 1;
  } 
  if(PIN(dc) < PIN(min_dc)){ // under voltage
    PIN(en_out) = 0.0;
    PIN(error) = 2;
  } 
  if(PIN(temp) > PIN(max_temp)){ // over temp
    PIN(en_out) = 0.0;
    PIN(error) = 3;
  }
  if(MAX3(ABS(PIN(iu)), ABS(PIN(iv)), ABS(PIN(iw))) > PIN(max_cur) * 1.1){ // over current
    PIN(en_out) = -1.0;
    PIN(error) = 4;
  }
  if(PIN(dc) > PIN(max_dc)){ // over voltage
    PIN(en_out) = -1.0;
    PIN(error) = 5;
  }

  PIN(scale) = MAX(MIN(1.0 - (PIN(temp) - PIN(max_temp) + 10.0) / 10.0, 1.0), 0.0);
  PIN(scale) = MAX(MIN(1.0 - (PIN(mot_temp) - PIN(max_mot_temp) + 10.0) / 10.0, PIN(scale)), 0.0);
  PIN(scale) = MAX(MIN(1.0 - (ABS(PIN(vel)) - PIN(max_vel) + 10) / 10.0, PIN(scale)), 0.0);

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
