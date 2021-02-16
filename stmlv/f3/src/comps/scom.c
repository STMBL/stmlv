#include "scom_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "com.h"
#include "main.h"

HAL_COMP(scom);

HAL_PIN(vel_fb);
HAL_PIN(torque_cmd);

HAL_PIN(state);
HAL_PIN(en);

HAL_PIN(id);

HAL_PIN(timeout);
HAL_PIN(timer);
HAL_PIN(clear);
HAL_PIN(brr);

struct scom_ctx_t {
  mcom_t mcom;
  scom_t scom;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr){
  struct scom_pin_ctx_t *pins = (struct scom_pin_ctx_t *)pin_ptr;
  PIN(timeout) = 20.0 / 5000.0;
  PIN(brr) = 24 * 2;
}

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr){
  struct scom_ctx_t * ctx = (struct scom_ctx_t *)ctx_ptr;
  struct scom_pin_ctx_t *pins = (struct scom_pin_ctx_t *)pin_ptr;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // clock en
  RCC->CFGR3 |= RCC_CFGR3_USART3SW_SYSCLK; // sysclk

  GPIOC->MODER |= GPIO_MODER_MODER13_0; // pc13 tx en

  GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1; // af pb10/pb11 uart3 rx/tx
  GPIOB->AFR[1] |= (GPIO_AF7_USART3 << GPIO_AFRH_AFRH2_Pos) | (GPIO_AF7_USART3 << GPIO_AFRH_AFRH3_Pos);

  USART3->BRR = PIN(brr); // 3e6
  USART3->CR2 |= USART_CR2_SWAP; // swap tx/rx pins

  USART3->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_OVRDIS;
  USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

  DMA1_Channel2->CNDTR = sizeof(scom_t); // tx dma
  DMA1_Channel2->CMAR = &(ctx->scom);
  DMA1_Channel2->CPAR = &(USART3->TDR);
  DMA1_Channel2->CCR |= DMA_CCR_PL_1 | DMA_CCR_MINC | DMA_CCR_DIR;

  DMA1_Channel3->CNDTR = sizeof(mcom_t); // rx dma
  DMA1_Channel3->CMAR = &(ctx->mcom);
  DMA1_Channel3->CPAR = &(USART3->RDR);
  DMA1_Channel3->CCR |= DMA_CCR_PL_1 | DMA_CCR_MINC;

  DMA1_Channel3->CCR |= DMA_CCR_EN; // start rx dma
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct scom_ctx_t * ctx = (struct scom_ctx_t *)ctx_ptr;
  struct scom_pin_ctx_t *pins = (struct scom_pin_ctx_t *)pin_ptr;
  
  DMA1_Channel2->CCR &= ~DMA_CCR_EN; // stop tx dma
  GPIOC->ODR &= ~(1 << 13); // tx !en pc13

  PIN(timer)++;

  if(PIN(clear) > 0.0){
    PIN(clear) = 0.0;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN; // stop rx dma
    DMA1_Channel3->CNDTR = sizeof(mcom_t);
    DMA1_Channel3->CCR |= DMA_CCR_EN; // start rx dma
  }

  if(DMA1_Channel3->CNDTR == 0){ // buffer full
    PIN(clear) = 1.0;
    if(0 == crc8((uint8_t *)&(ctx->mcom), sizeof(mcom_t))){ // check crc
      PIN(torque_cmd) = ctx->mcom.torque_cmd[(int)PIN(id)];
      PIN(en) = ctx->mcom.en;
      PIN(timer) = 0.0;

      if(ctx->mcom.reply_id == PIN(id)){ // reply
        ctx->scom.reply_id = ctx->mcom.reply_id;
        ctx->scom.vel_fb = PIN(vel_fb);
        ctx->scom.df = 0;
        ctx->scom.state = PIN(state);
        ctx->scom.crc = 0;
        ctx->scom.crc = crc8((uint8_t *)&(ctx->scom), sizeof(scom_t));

        GPIOC->ODR |= 1 << 13; // tx en pc13
        DMA1_Channel2->CNDTR = sizeof(scom_t);
        DMA1_Channel2->CCR |= DMA_CCR_EN; // start tx dma
      }

      ctx->mcom.reply_id++; // destroy package
    }
  }

  if(PIN(timer) > PIN(timeout)){
    PIN(torque_cmd) = 0.0;
    PIN(en) = 0.0;
  }
}

hal_comp_t scom_comp_struct = {
    .name      = "scom",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct scom_ctx_t),
    .pin_count = sizeof(struct scom_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
