#include "io_comp.h"
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

HAL_COMP(io);

HAL_PIN(sin);
HAL_PIN(cos);
HAL_PIN(vref3);
HAL_PIN(vref4);
HAL_PIN(dc);
HAL_PIN(phase_volt);
HAL_PIN(temp);
HAL_PIN(a0);
HAL_PIN(a1);
HAL_PIN(a2);
HAL_PIN(a3);
HAL_PIN(a4);
HAL_PIN(iu);
HAL_PIN(iv);
HAL_PIN(iw);

HAL_PIN(iu_offset);
HAL_PIN(iv_offset);
HAL_PIN(iw_offset);

HAL_PIN(offset_counter);

HAL_PIN(MOSI);
HAL_PIN(SCK);
HAL_PIN(MISO);
HAL_PIN(CS);

HAL_PIN(dac);
HAL_PIN(hw_filter);

extern volatile struct adc12_struct_t adc12_buffer[3];
extern volatile struct adc34_struct_t adc34_buffer[3];

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr){
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;
  PIN(dac) = 800;
  DAC1->DHR12R1 = PIN(dac);
  PIN(hw_filter) = 10;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct io_ctx_t * ctx = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;

  uint16_t ioa = GPIOA->IDR;
  uint16_t iob = GPIOB->IDR;
  uint16_t ioc = GPIOC->IDR;
  uint16_t iod = GPIOD->IDR;

  DAC1->DHR12R1 = PIN(dac);
  TIM1->BDTR |= ((int) PIN(hw_filter) << TIM_BDTR_BKF_Pos) & TIM_BDTR_BKF_Msk;
  
  PIN(sin) = V_DIFF(adc12_buffer[0].sin, 1);
  PIN(cos) = V_DIFF(adc12_buffer[0].cos, 1);
  PIN(dc) = VOLT(adc12_buffer[0].dc) * DC_SCALE;
  PIN(phase_volt) = PIN(dc) / 2.0 * 1.15 * 0.95;
  PIN(temp) = NTC_MULT * (adc12_buffer[0].temp - NTC_ADD) + NTC_TEMP0;
  PIN(a0) = VOLT(adc12_buffer[0].a0) * AIN_SCALE;
  PIN(a1) = VOLT(adc12_buffer[0].a1) * AIN_SCALE;
  PIN(a2) = VOLT(adc12_buffer[0].a2) * AIN_SCALE;
  PIN(a3) = VOLT(adc12_buffer[0].a3) * AIN_SCALE;
  PIN(a4) = VOLT(adc34_buffer[0].a4) * AIN_SCALE;
  PIN(iu) = VOLT(adc34_buffer[0].iu + adc34_buffer[1].iu + adc34_buffer[2].iu) / 3.0 * CURRENT_SCALE + CURRENT_OFFSET - PIN(iu_offset);
  PIN(iv) = VOLT(adc12_buffer[0].iv + adc12_buffer[1].iv + adc12_buffer[2].iv) / 3.0 * CURRENT_SCALE + CURRENT_OFFSET - PIN(iv_offset);
  PIN(iw) = VOLT(adc12_buffer[0].iw + adc12_buffer[1].iw + adc12_buffer[2].iw) / 3.0 * CURRENT_SCALE + CURRENT_OFFSET - PIN(iw_offset);
  // PIN(iu) = VOLT(adc34_buffer[2].iu) * CURRENT_SCALE + CURRENT_OFFSET - PIN(iu_offset);
  // PIN(iv) = VOLT(adc12_buffer[2].iv) * CURRENT_SCALE + CURRENT_OFFSET - PIN(iv_offset);
  // PIN(iw) = VOLT(adc12_buffer[2].iw) * CURRENT_SCALE + CURRENT_OFFSET - PIN(iw_offset);
  PIN(vref3) = VOLT(adc34_buffer[0].vref3);
  PIN(vref4) = VOLT(adc34_buffer[0].vref4);
  
  PIN(SCK) = (ioc & (1 << 10)) > 0;
  PIN(MISO) = (ioc & (1 << 11)) > 0;
  PIN(MOSI) = (ioc & (1 << 12)) > 0;
  PIN(CS) = (iod & (1 << 2)) > 0;

  if(PIN(offset_counter) < 1000){
    PIN(iu_offset) += PIN(iu) / 25.0;
    PIN(iv_offset) += PIN(iv) / 25.0;
    PIN(iw_offset) += PIN(iw) / 25.0;
    PIN(offset_counter)++;
  }

}

hal_comp_t io_comp_struct = {
    .name      = "io",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct io_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
