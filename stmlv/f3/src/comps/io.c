#include "io_comp.h"
#include "commands.h"
#include "hal.h"
#include "hw.h"
#include "main.h"

#define INPUT_REF (OP_REF * OP_R_OUT_LOW / (OP_R_OUT_HIGH + OP_R_OUT_LOW))
#define INPUT_GAIN (OP_R_FEEDBACK / OP_R_INPUT * OP_R_OUT_LOW / (OP_R_OUT_HIGH + OP_R_OUT_LOW))
#define V_DIFF(ADC, OVER) ((((float)(ADC)) / (float)(OVER) / ADC_RES * ADC_REF - INPUT_REF) / INPUT_GAIN)

#define VOLT(ADC) ((float)(ADC) / ADC_RES * ADC_REF)

HAL_COMP(io);

HAL_PIN(sin);
HAL_PIN(cos);
HAL_PIN(vref3);
HAL_PIN(vref4);
HAL_PIN(dc);
HAL_PIN(temp);
HAL_PIN(a0);
HAL_PIN(a1);
HAL_PIN(a2);
HAL_PIN(a3);
HAL_PIN(a4);
HAL_PIN(iu);
HAL_PIN(iv);
HAL_PIN(iw);

HAL_PIN(MOSI);
HAL_PIN(SCK);
HAL_PIN(MISO);
HAL_PIN(CS);


HAL_PIN(dma1_ndtr);

extern volatile struct adc12_struct_t adc12_buffer[3];
extern volatile struct adc34_struct_t adc34_buffer[3];

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct io_ctx_t * ctx = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;

  uint16_t ioa = GPIOA->IDR;
  uint16_t iob = GPIOB->IDR;
  uint16_t ioc = GPIOC->IDR;
  uint16_t iod = GPIOD->IDR;

  PIN(sin) = V_DIFF(adc12_buffer[0].sin, 1);
  PIN(cos) = V_DIFF(adc12_buffer[0].cos, 1);
  PIN(dc) = VOLT(adc12_buffer[0].dc) * DC_SCALE;
  PIN(temp) = VOLT(adc12_buffer[0].temp);
  PIN(a0) = VOLT(adc12_buffer[0].a0) * AIN_SCALE;
  PIN(a1) = VOLT(adc12_buffer[0].a1) * AIN_SCALE;
  PIN(a2) = VOLT(adc12_buffer[0].a2) * AIN_SCALE;
  PIN(a3) = VOLT(adc12_buffer[0].a3) * AIN_SCALE;
  PIN(a4) = VOLT(adc34_buffer[0].a4) * AIN_SCALE;
  PIN(iu) = VOLT(adc34_buffer[2].iu) * CURRENT_SCALE - CURRENT_OFFSET;
  PIN(iv) = VOLT(adc12_buffer[2].iv) * CURRENT_SCALE - CURRENT_OFFSET;
  PIN(iw) = VOLT(adc12_buffer[2].iw) * CURRENT_SCALE - CURRENT_OFFSET;
  PIN(vref3) = VOLT(adc34_buffer[0].vref3);
  PIN(vref4) = VOLT(adc34_buffer[0].vref4);
  
  PIN(SCK) = (ioc & (1 << 10)) > 0;
  PIN(MISO) = (ioc & (1 << 11)) > 0;
  PIN(MOSI) = (ioc & (1 << 12)) > 0;
  PIN(CS) = (iod & (1 << 2)) > 0;

  PIN(dma1_ndtr) = DMA1_Channel1->CNDTR;
}

hal_comp_t io_comp_struct = {
    .name      = "io",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct io_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
