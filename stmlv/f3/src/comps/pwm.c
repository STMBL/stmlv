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

HAL_PIN(error);
HAL_PIN(state);

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct pwm_ctx_t * ctx = (struct pwm_ctx_t *)ctx_ptr;
  struct pwm_pin_ctx_t *pins = (struct pwm_pin_ctx_t *)pin_ptr;

  PIN(max) = 0.975;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct pwm_ctx_t * ctx = (struct pwm_ctx_t *)ctx_ptr;
  struct pwm_pin_ctx_t *pins = (struct pwm_pin_ctx_t *)pin_ptr;

  switch((int) PIN(state)){
    case -1: // active short
      if(!(TIM1->BDTR & TIM_BDTR_MOE)){ // hw overcurrent
        PIN(error) = 1.0;
      }
      if(PIN(en) == 0.0){ // disable
        PIN(state) = 0.0;
      }
      if(PIN(en) > 0.0){ // enable
        PIN(state) = 1.0;
      }
      TIM1->CCR1 = 0;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = 0;
    break;

    case 0: // disable, floating, undervoltage,
      TIM1->BDTR &= ~TIM_BDTR_MOE;
      TIM1->CCR1 = 0;
      TIM1->CCR2 = 0;
      TIM1->CCR3 = 0;
      PIN(error) = 0.0;
     
      if(PIN(en) > 0.0){ // enable
        PIN(state) = 1.0;        
        TIM1->BDTR |= TIM_BDTR_MOE;  
      }
      if(PIN(en) < 0.0){ // active short
        PIN(state) = -1.0;
        TIM1->BDTR |= TIM_BDTR_MOE;  
      }
    break;

    case 1: // enable
      if(!(TIM1->BDTR & TIM_BDTR_MOE)){ // hw overcurrent
        PIN(error) = 1.0;
      }
      if(PIN(en) == 0.0){ // disable
        PIN(state) = 0.0;
      }
      if(PIN(en) < 0.0){ // active short
        PIN(state) = -1.0;
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
