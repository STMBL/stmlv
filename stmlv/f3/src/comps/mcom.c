#include "mcom_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "com.h"
#include "main.h"

HAL_COMP(mcom);

HAL_PINA(torque_cmd, 4);
HAL_PINA(vel_fb, 4);
HAL_PINA(state, 4);

HAL_PIN(en);
HAL_PIN(id);

HAL_PIN(tx_timer);
HAL_PIN(tx_time);

HAL_PIN(timeout);
HAL_PIN(timer);

HAL_PIN(brr);

struct mcom_ctx_t {
  mcom_t mcom;
  scom_t scom;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr){
  struct mcom_pin_ctx_t *pins = (struct mcom_pin_ctx_t *)pin_ptr;

  PIN(tx_time) = 0.001;
  PIN(brr) = 24 * 2;
}

uint8_t crc8 (uint8_t * ptr, uint32_t len){
  uint8_t crc = 0;
  for(uint32_t i = 0; i < len; i++){
    crc ^= ptr[i];
  }
  return(crc);
}

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr){
  struct mcom_ctx_t * ctx = (struct mcom_ctx_t *)ctx_ptr;
  struct mcom_pin_ctx_t *pins = (struct mcom_pin_ctx_t *)pin_ptr;

  RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // clock en
  RCC->CFGR3 |= RCC_CFGR3_USART3SW_SYSCLK; // sysclk

  GPIOC->MODER |= GPIO_MODER_MODER9_0; // pc9 tx en

  GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1; // af pb10/pb11 uart3 tx/rx
  GPIOB->AFR[1] |= (GPIO_AF7_USART3 << GPIO_AFRH_AFRH2_Pos) | (GPIO_AF7_USART3 << GPIO_AFRH_AFRH3_Pos);

  USART3->BRR = PIN(brr); // 3e6
  USART3->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_OVRDIS;
  USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

  DMA1_Channel2->CNDTR = sizeof(mcom_t); // tx dma
  DMA1_Channel2->CMAR = &(ctx->mcom);
  DMA1_Channel2->CPAR = &(USART3->TDR);
  DMA1_Channel2->CCR |= DMA_CCR_PL_1 | DMA_CCR_MINC | DMA_CCR_DIR;

  DMA1_Channel3->CNDTR = sizeof(scom_t); // rx dma
  DMA1_Channel3->CMAR = &(ctx->scom);
  DMA1_Channel3->CPAR = &(USART3->RDR);
  DMA1_Channel3->CCR |= DMA_CCR_PL_1 | DMA_CCR_MINC;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct mcom_ctx_t * ctx = (struct mcom_ctx_t *)ctx_ptr;
  struct mcom_pin_ctx_t *pins = (struct mcom_pin_ctx_t *)pin_ptr;
  
  PIN(tx_timer) += period;

  if(PIN(tx_timer) > PIN(tx_time)){
    PIN(tx_timer) = 0.0;
    ctx->mcom.torque_cmd[0] = PINA(torque_cmd, 0);
    ctx->mcom.torque_cmd[1] = PINA(torque_cmd, 1);
    ctx->mcom.torque_cmd[2] = PINA(torque_cmd, 2);
    ctx->mcom.torque_cmd[3] = PINA(torque_cmd, 3);
    ctx->mcom.en = PIN(en);
    ctx->mcom.reply_id++;
    ctx->mcom.reply_id %= 4;
    ctx->mcom.crc = 0;
    ctx->mcom.crc = crc8((uint8_t *)&(ctx->mcom), sizeof(mcom_t));

    GPIOC->ODR |= 1 << 9; // tx en pc9
    DMA1_Channel2->CCR |= DMA_CCR_EN; // start tx dma

    DMA1_Channel3->CCR &= ~DMA_CCR_EN; // stop rx dma

    if(0 == crc8((uint8_t *)&(ctx->scom), sizeof(scom_t))){ // check crc
      PINA(vel_fb, ctx->scom.reply_id) = ctx->scom.vel_fb;
      PINA(state, ctx->scom.reply_id) = ctx->scom.state;
      ctx->scom.reply_id++; // destroy package
    }

    DMA1_Channel3->CNDTR = sizeof(mcom_t);
    DMA1_Channel3->CCR |= DMA_CCR_EN; // start rx dma
  }
  else{
    DMA1_Channel2->CCR &= ~DMA_CCR_EN; // stop tx dma
    DMA1_Channel2->CNDTR = sizeof(mcom_t); // tx dma
    GPIOC->ODR &= ~(1 << 9); // tx !en pc9
  }
}

hal_comp_t mcom_comp_struct = {
    .name      = "mcom",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct mcom_ctx_t),
    .pin_count = sizeof(struct mcom_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
