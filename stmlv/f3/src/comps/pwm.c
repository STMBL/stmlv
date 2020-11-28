#include "pwm_comp.h"
#include "commands.h"
#include "defines.h"
#include "hal.h"
#include "hw.h"
#include "main.h"

HAL_COMP(pwm);

HAL_PIN(en);
HAL_PIN(u);
HAL_PIN(v);
HAL_PIN(w);
HAL_PIN(dc);
HAL_PIN(max);

HAL_PIN(en_out);

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct pwm_ctx_t * ctx = (struct pwm_ctx_t *)ctx_ptr;
  struct pwm_pin_ctx_t *pins = (struct pwm_pin_ctx_t *)pin_ptr;

  PIN(max) = 0.975;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct pwm_ctx_t * ctx = (struct pwm_ctx_t *)ctx_ptr;
  struct pwm_pin_ctx_t *pins = (struct pwm_pin_ctx_t *)pin_ptr;

  if(PIN(en) == 0.0 || !(TIM1->BDTR & TIM_BDTR_MOE)){
    PIN(en_out) = 0.0;
  }

  switch((int) PIN(en)){
    case -1: // active short
      TIM1->BDTR |= TIM_BDTR_MOE;
      TIM1->CCR1 = 0;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = 0;
    break;

    case 0: // disabled, floating, undervoltage, 
      TIM1->BDTR &= ~TIM_BDTR_MOE;
      TIM1->CCR1 = 0;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = 0;
    break;

    case 1: // enabled
      if(PIN(en_out) == 0.0){
        TIM1->BDTR |= TIM_BDTR_MOE;
        PIN(en_out) = 1.0;
      }
      TIM1->CCR3 = CLAMP(PIN(u) / PIN(dc) * PWM_RES, 0, PIN(max) * PWM_RES);
      TIM1->CCR2 = CLAMP(PIN(v) / PIN(dc) * PWM_RES, 0, PIN(max) * PWM_RES);
      TIM1->CCR1 = CLAMP(PIN(w) / PIN(dc) * PWM_RES, 0, PIN(max) * PWM_RES);
    break;
  }
}

hal_comp_t pwm_comp_struct = {
    .name      = "pwm",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct pwm_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
