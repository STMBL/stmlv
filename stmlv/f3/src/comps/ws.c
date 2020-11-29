#include "ws_comp.h"
#include "commands.h"
#include "hal.h"
#include "hw.h"
#include "main.h"
#include "math.h"

#define INPUT_REF (OP_REF * OP_R_OUT_LOW / (OP_R_OUT_HIGH + OP_R_OUT_LOW))
#define INPUT_GAIN (OP_R_FEEDBACK / OP_R_INPUT * OP_R_OUT_LOW / (OP_R_OUT_HIGH + OP_R_OUT_LOW))
#define V_DIFF(ADC, OVER) ((((float)(ADC)) / (float)(OVER) / ADC_RES * ADC_REF - INPUT_REF) / INPUT_GAIN)

#define VOLT(ADC) ((float)(ADC) / ADC_RES * ADC_REF)

#define NTC_RES_TEMP1 (NTC_R0 * expf(NTC_B * (1 / (NTC_TEMP1 + 273.0) - 1.0 / (NTC_TEMP0 + 273.0))))
#define NTC_ADC_TEMP1 (ADC_RES * NTC_RES_TEMP1 / (NTC_RES_TEMP1 + NTC_PULLUP))
#define NTC_ADD (ADC_RES * NTC_R0 / (NTC_R0 + NTC_PULLUP))
#define NTC_MULT ((NTC_TEMP1 - NTC_TEMP0) / (NTC_ADC_TEMP1 - NTC_ADD))

HAL_COMP(ws);

HAL_PIN(r);
HAL_PIN(g);
HAL_PIN(b);
HAL_PIN(y);
HAL_PIN(timer);

HAL_PIN(tim);


extern volatile uint32_t ws_buffer[3 * 3 * 8 + 3];

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr){
  struct ws_pin_ctx_t *pins = (struct ws_pin_ctx_t *)pin_ptr;
  // red led
  PIN(tim) = 72000000.0 * 0.00000038;
}

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr){
  struct ws_pin_ctx_t *pins = (struct ws_pin_ctx_t *)pin_ptr;
  GPIOC->MODER |= GPIO_MODER_MODER15_0;

  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // clock en
  TIM6->ARR = PIN(tim);
  TIM6->DIER |= TIM_DIER_UDE; // dma req
  TIM6->CR1 |= TIM_CR1_CEN; // enable
   
  DMA2_Channel3->CNDTR = 3 * 3 * 8 + 3;
  DMA2_Channel3->CMAR = &(ws_buffer[0]);
  DMA2_Channel3->CPAR = &(GPIOC->BSRR);
  DMA2_Channel3->CCR |= DMA_CCR_PL_1 | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_DIR;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct ws_ctx_t * ctx = (struct ws_ctx_t *)ctx_ptr;
  struct ws_pin_ctx_t *pins = (struct ws_pin_ctx_t *)pin_ptr;
  TIM6->ARR = PIN(tim);
  uint32_t grb;
  // grb = ((uint8_t)(PIN(g) * 255)) << 16 | ((uint8_t)(PIN(r) * 255)) << 8 | ((uint8_t)(PIN(b) * 255));
  grb = (((uint8_t)(PIN(g) * 255)) << 16) | (((uint8_t)(PIN(r) * 255)) << 8) | (((uint8_t)(PIN(b) * 255)));

  uint32_t on = 0x1 << 15;
  uint32_t off = 0x1 << 31;

  DMA2_Channel3->CCR &= ~(DMA_CCR_EN);
  PIN(timer) -= 1.0;

  if(PIN(timer) <= 0.0){
    PIN(timer) = 100;
    ws_buffer[0] = off;
    ws_buffer[1] = off;
    ws_buffer[2] = off;

    for(int i = 3; i < 3 * 3 * 8 + 3; i += 3){
      if(grb & (1 << 23)){
        ws_buffer[i + 0] = on;
        ws_buffer[i + 1] = on;
        ws_buffer[i + 2] = off;
      }
      else{
        ws_buffer[i + 0] = on;
        ws_buffer[i + 1] = off;
        ws_buffer[i + 2] = off;
      }
      grb <<= 1;
    }

    DMA2_Channel3->CNDTR = 3 * 3 * 8 + 3;
    DMA2_Channel3->CCR |= DMA_CCR_EN;
  }
  else{
    if(PIN(y) > 0.0){
      GPIOC->BSRR = on;
    }
    else{
      GPIOC->BSRR = off;
    }
  }
}

hal_comp_t ws_comp_struct = {
    .name      = "ws",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct ws_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
