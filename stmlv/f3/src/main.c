/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hal.h"
#include "ringbuf.h"
#include "hw.h"
#include "commands.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t systick_freq;
volatile uint64_t systime = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SysTick_Handler(void) {
  /* USER CODE BEGIN SysTick_IRQn 0 */
  systime++;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

uint32_t hal_get_systick_value() {
  return (SysTick->VAL);
}

uint32_t hal_get_systick_reload() {
  return (SysTick->LOAD);
}

uint32_t hal_get_systick_freq() {
  return (systick_freq);
}

void bootloader(char *ptr) {
  hal_stop();

  GPIOA->MODER &= ~GPIO_MODER_MODER12_Msk;
  GPIOA->MODER |= GPIO_MODER_MODER12_0;
  GPIOA->ODR &= ~GPIO_PIN_12;
  HAL_Delay(1);

#ifdef USB_DISCONNECT_PIN
  HAL_GPIO_WritePin(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
#endif
  // RTC->BKP0R = 0xDEADBEEF;


  NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  NVIC_DisableIRQ(SysTick_IRQn);

  HAL_RCC_DeInit();
  RCC->APB2RSTR = 0x17F801;
  RCC->APB1RSTR = 0x76FEC837;
  RCC->AHBRSTR = 0x31FF0020;

  SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_0;

  void (*SysMemBootJump)(void);
  volatile uint32_t addr = 0x1FFFD800;
  SysMemBootJump = (void (*)(void))(*((uint32_t *)(addr + 4)));
  __set_MSP(*(uint32_t *)addr);
  SysMemBootJump();

//  NVIC_SystemReset();

  // __disable_irq();

  // SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_0;
  
  // volatile uint32_t addr = 0x1FFFD800;
  // void (*dfu_boot)(void);
  // dfu_boot = (void (*)(void)) (*((uint32_t *)(addr + 4)));
	// __set_MSP(__get_PSP());
  // dfu_boot();
}

COMMAND("bootloader", bootloader, "enter bootloader");

void reset(char *ptr) {
  hal_stop();

  GPIOA->MODER &= ~GPIO_MODER_MODER12_Msk;
  GPIOA->MODER |= GPIO_MODER_MODER12_0;
  GPIOA->ODR &= ~GPIO_PIN_12;
  HAL_Delay(100);

  HAL_NVIC_SystemReset();
}
COMMAND("reset", reset, "reset STMLV");

void TIM1_UP_TIM16_IRQHandler() {
  TIM1->SR &= ~TIM_SR_UIF;
  // GPIOD->ODR |= GPIO_PIN_2;
  hal_run_rt();
  if(TIM1->SR & TIM_SR_UIF) {
    hal_stop();
    hal.hal_state = RT_TOO_LONG;
  }
  // GPIOD->ODR &= ~GPIO_PIN_2;
}


volatile struct adc12_struct_t adc12_buffer[3];

volatile struct adc34_struct_t adc34_buffer[3];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  extern void *g_pfnVectors;
  SCB->VTOR = (uint32_t)&g_pfnVectors;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  systick_freq = HAL_RCC_GetHCLKFreq();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  // gpio clock
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOEEN | RCC_AHBENR_GPIOFEN;
  volatile uint32_t delay_counter = 1 * SystemCoreClock / 1000000;
  while(delay_counter != 0U){
    delay_counter--;
  }

  // usb disconnect
  // pull down usb+ for 1ms
  GPIOA->MODER |= GPIO_MODER_MODER12_0;
  delay_counter = 1000 * SystemCoreClock / 1000000;
  while(delay_counter != 0U){
    delay_counter--;
  }

  // gpio pwm tim
  GPIOA->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
  GPIOA->AFR[1] |= (GPIO_AF6_TIM1 << GPIO_AFRH_AFRH0_Pos) | (GPIO_AF6_TIM1 << GPIO_AFRH_AFRH1_Pos) | (GPIO_AF6_TIM1 << GPIO_AFRH_AFRH2_Pos);

  GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
  GPIOB->AFR[1] |= (GPIO_AF6_TIM1 << GPIO_AFRH_AFRH5_Pos) | (GPIO_AF6_TIM1 << GPIO_AFRH_AFRH6_Pos) | (GPIO_AF4_TIM1 << GPIO_AFRH_AFRH7_Pos);


  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // clock en
  TIM1->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CMS_1; // center aligned 3

  TIM1->CR2 |= TIM_CR2_MMS_1;  // update -> trgo -> adc

  TIM1->DIER |= TIM_DIER_UIE; // update interupt

  TIM1->ARR = HAL_RCC_GetHCLKFreq() / PWM_FREQ;

  TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; // preload enable
  TIM1->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // pwm mode 3

  TIM1->CCMR2 |= TIM_CCMR2_OC3PE; // preload enable
  TIM1->CCMR2 |= TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // pwm mode 3

  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE; // channel enable
  
  TIM1->BDTR |= 50; // 50/144e6 = 340ns deadtime
  // TIM1->BDTR |= TIM_BDTR_OSSI; // disable state
  // TIM1->BDTR |= TIM_BDTR_OSSR; // enable state
  TIM1->BDTR |= 0xa << TIM_BDTR_BKF_Pos; // brk filter
  TIM1->BDTR |= TIM_BDTR_BKP; // brk polarity high
  TIM1->BDTR |= TIM_BDTR_BKE; // brk enable
  
  TIM1->RCR = 5; // 15khz pwm -> 5khz rt


  // opamp gpio
  GPIOA->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER1_0;
  GPIOB->MODER |= GPIO_MODER_MODER1_1 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0;

  // enable syscfg block 
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  
  // wait 1µs
  delay_counter = 1 * SystemCoreClock / 1000000;
  while(delay_counter != 0U){
    delay_counter--;
  }
  
  OPAMP_HandleTypeDef hopamp;
  hopamp.Instance = OPAMP1;
  hopamp.State =  HAL_OPAMP_STATE_READY;
  HAL_OPAMP_SelfCalibrate(&hopamp);

  hopamp.Instance = OPAMP2;
  hopamp.State =  HAL_OPAMP_STATE_READY;
  HAL_OPAMP_SelfCalibrate(&hopamp);

  hopamp.Instance = OPAMP3;
  hopamp.State =  HAL_OPAMP_STATE_READY;
  HAL_OPAMP_SelfCalibrate(&hopamp);


  // opamp, pga 16x
  OPAMP1->CSR |= OPAMP_CSR_PGGAIN_1 | OPAMP_CSR_PGGAIN_0 | OPAMP_CSR_VMSEL_1 | OPAMP_CSR_VPSEL_1 | OPAMP_CSR_VPSEL_0 | OPAMP_CSR_OPAMPxEN;
  OPAMP2->CSR |= OPAMP_CSR_PGGAIN_1 | OPAMP_CSR_PGGAIN_0 | OPAMP_CSR_VMSEL_1 | OPAMP_CSR_VPSEL_1 | OPAMP_CSR_VPSEL_0 | OPAMP_CSR_OPAMPxEN;
  OPAMP3->CSR |= OPAMP_CSR_PGGAIN_1 | OPAMP_CSR_PGGAIN_0 | OPAMP_CSR_VMSEL_1 | OPAMP_CSR_VPSEL_1 | OPAMP_CSR_VPSEL_0 | OPAMP_CSR_OPAMPxEN;


  // comp , dac1_ch1 -> comp -> tim1_brk2
  COMP1->CSR |= COMP_CSR_COMPxOUTSEL_0 | COMP_CSR_COMPxINSEL_2 | COMP_CSR_COMPxEN;
  COMP2->CSR |= COMP_CSR_COMPxOUTSEL_0 | COMP_CSR_COMPxINSEL_2 | COMP_CSR_COMPxEN;
  COMP4->CSR |= COMP_CSR_COMPxOUTSEL_0 | COMP_CSR_COMPxINSEL_2 | COMP_CSR_COMPxEN;


  // dac gpio
  GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER4_0;

  // dac
  RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
  delay_counter = 1 * SystemCoreClock / 1000000;
  while(delay_counter != 0U){
    delay_counter--;
  }

  DAC1->DHR12R1 = (0.1 + 0.1) / 3.3 * 4096.0; // TODO fix
  DAC1->CR |= DAC_CR_EN1;


  // gpio adc
  GPIOA->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER5_0;
  GPIOB->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER12_0;
  GPIOC->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0;

  // adc clock
  RCC->AHBENR |= RCC_AHBENR_ADC12EN | RCC_AHBENR_ADC34EN;

  // wait 1µs
  delay_counter = 1 * SystemCoreClock / 1000000;
  while(delay_counter != 0U){
    delay_counter--;
  }

  // adc voltage regulator
  ADC1->CR = 0;
  ADC2->CR = 0;
  ADC3->CR = 0;
  ADC4->CR = 0;
  
  // enable vreg
  ADC1->CR |= ADC_CR_ADVREGEN_0;
  ADC2->CR |= ADC_CR_ADVREGEN_0;
  ADC3->CR |= ADC_CR_ADVREGEN_0;
  ADC4->CR |= ADC_CR_ADVREGEN_0;

  // wait 10µs
  delay_counter = 10 * SystemCoreClock / 1000000;
  while(delay_counter != 0U){
    delay_counter--;
  }

  // calibrate
  ADC1->CR &= (~ADC_CR_ADCALDIF);
  ADC1->CR |= ADC_CR_ADCAL;

  ADC2->CR &= (~ADC_CR_ADCALDIF);
  ADC2->CR |= ADC_CR_ADCAL;

  ADC3->CR &= (~ADC_CR_ADCALDIF);
  ADC3->CR |= ADC_CR_ADCAL;

  ADC4->CR &= (~ADC_CR_ADCALDIF);
  ADC4->CR |= ADC_CR_ADCAL;
  
  // timeout 10ms
  delay_counter = 10000 * SystemCoreClock / 1000000;
  while(((ADC1->CR & ADC_CR_ADCAL)) & (delay_counter != 0U)){
    delay_counter--;
  }
  if(delay_counter == 0){
    GPIOC->ODR |= GPIO_ODR_15;
  }

  // timeout 10ms
  delay_counter = 10000 * SystemCoreClock / 1000000;
  while(((ADC2->CR & ADC_CR_ADCAL)) & (delay_counter != 0U)){
    delay_counter--;
  }
  if(delay_counter == 0){
    GPIOC->ODR |= GPIO_ODR_15;
  }

  // timeout 10ms
  delay_counter = 10000 * SystemCoreClock / 1000000;
  while(((ADC3->CR & ADC_CR_ADCAL)) & (delay_counter != 0U)){
    delay_counter--;
  }
  if(delay_counter == 0){
    GPIOC->ODR |= GPIO_ODR_15;
  }

  // timeout 10ms
  delay_counter = 10000 * SystemCoreClock / 1000000;
  while(((ADC4->CR & ADC_CR_ADCAL)) & (delay_counter != 0U)){
    delay_counter--;
  }
  if(delay_counter == 0){
    GPIOC->ODR |= GPIO_ODR_15;
  }

  // 3 disc. conversions
  ADC1->CFGR |= (5 - 1) << ADC_CFGR_DISCNUM_Pos;
  ADC2->CFGR |= (5 - 1) << ADC_CFGR_DISCNUM_Pos;
  ADC3->CFGR |= (2 - 1) << ADC_CFGR_DISCNUM_Pos;
  ADC4->CFGR |= (2 - 1) << ADC_CFGR_DISCNUM_Pos;

  // disc. mode
  ADC1->CFGR |= ADC_CFGR_DISCEN;
  ADC2->CFGR |= ADC_CFGR_DISCEN;
  ADC3->CFGR |= ADC_CFGR_DISCEN;
  ADC4->CFGR |= ADC_CFGR_DISCEN;
  
  // ext. trigger rising edge
  ADC1->CFGR |= ADC_CFGR_EXTEN;
  ADC2->CFGR |= ADC_CFGR_EXTEN;
  ADC3->CFGR |= ADC_CFGR_EXTEN;
  ADC4->CFGR |= ADC_CFGR_EXTEN;
  
  // ext. trigger tim1 trgo
  ADC1->CFGR |= ADC1_2_EXTERNALTRIG_T1_TRGO;
  ADC2->CFGR |= ADC1_2_EXTERNALTRIG_T1_TRGO;
  ADC3->CFGR |= ADC3_4_EXTERNALTRIG_T1_TRGO;
  ADC4->CFGR |= ADC3_4_EXTERNALTRIG_T1_TRGO;

  // dma
  ADC1->CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;
  // ADC2->CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;
  ADC3->CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;
  // ADC4->CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;

  // sample time
  ADC1->SMPR1 |= ADC_SMPR1_SMP3_2 | ADC_SMPR1_SMP5_2 | ADC_SMPR1_SMP6_2 | ADC_SMPR1_SMP8_2 | ADC_SMPR1_SMP9_2;

  ADC2->SMPR1 |= ADC_SMPR1_SMP2_2 | ADC_SMPR1_SMP3_2 | ADC_SMPR1_SMP5_2 | ADC_SMPR1_SMP7_2;
  ADC2->SMPR2 |= ADC_SMPR2_SMP11_2;

  ADC3->SMPR1 |= ADC_SMPR1_SMP1_2;
  ADC3->SMPR2 |= ADC_SMPR2_SMP18_2;

  ADC4->SMPR1 |= ADC_SMPR1_SMP3_2;
  ADC4->SMPR2 |= ADC_SMPR2_SMP18_2;

  // regular group length
  ADC1->SQR1 = 3 * 5 - 1;
  ADC2->SQR1 = 3 * 5 - 1;
  ADC3->SQR1 = 3 * 2 - 1;
  ADC4->SQR1 = 3 * 2 - 1;
  
  // groups
  ADC1->SQR1 |= (IW << ADC_SQR1_SQ1_Pos) | (COS << ADC_SQR1_SQ2_Pos) | (A1 << ADC_SQR1_SQ3_Pos) | (DC << ADC_SQR1_SQ4_Pos);
  ADC1->SQR2 |= (TEMP << ADC_SQR2_SQ5_Pos);
  
  ADC1->SQR2 |= (IW << ADC_SQR2_SQ6_Pos) | (COS << ADC_SQR2_SQ7_Pos) | (A1 << ADC_SQR2_SQ8_Pos) | (DC << ADC_SQR2_SQ9_Pos);
  ADC1->SQR3 |= (TEMP << ADC_SQR3_SQ10_Pos);
  
  ADC1->SQR3 |= (IW << ADC_SQR3_SQ11_Pos) | (COS << ADC_SQR3_SQ12_Pos) | (A1 << ADC_SQR3_SQ13_Pos) | (DC << ADC_SQR3_SQ14_Pos);
  ADC1->SQR4 |= (TEMP << ADC_SQR4_SQ15_Pos);
  

  ADC2->SQR1 |= (IV << ADC_SQR1_SQ1_Pos) | (SIN << ADC_SQR1_SQ2_Pos) | (A0 << ADC_SQR1_SQ3_Pos) | (A2 << ADC_SQR1_SQ4_Pos);
  ADC2->SQR2 |= (A3 << ADC_SQR2_SQ5_Pos);

  ADC2->SQR2 |= (IV << ADC_SQR2_SQ6_Pos) | (SIN << ADC_SQR2_SQ7_Pos) | (A0 << ADC_SQR2_SQ8_Pos) | (A2 << ADC_SQR2_SQ9_Pos);
  ADC2->SQR3 |= (A3 << ADC_SQR3_SQ10_Pos);

  ADC2->SQR3 |= (IV << ADC_SQR3_SQ11_Pos) | (SIN << ADC_SQR3_SQ12_Pos) | (A0 << ADC_SQR3_SQ13_Pos) | (A2 << ADC_SQR3_SQ14_Pos);
  ADC2->SQR4 |= (A3 << ADC_SQR4_SQ15_Pos);


  ADC3->SQR1 |= (IU << ADC_SQR1_SQ1_Pos) | (VREF << ADC_SQR1_SQ2_Pos);

  ADC3->SQR1 |= (IU << ADC_SQR1_SQ3_Pos) | (VREF << ADC_SQR1_SQ4_Pos);

  ADC3->SQR2 |= (IU << ADC_SQR2_SQ5_Pos) | (VREF << ADC_SQR2_SQ6_Pos);


  ADC4->SQR1 |= (A4 << ADC_SQR1_SQ1_Pos) | (VREF << ADC_SQR1_SQ2_Pos);

  ADC4->SQR1 |= (A4 << ADC_SQR1_SQ3_Pos) | (VREF << ADC_SQR1_SQ4_Pos);

  ADC4->SQR2 |= (A4 << ADC_SQR2_SQ5_Pos) | (VREF << ADC_SQR2_SQ6_Pos);

  // calibration
  // ADC1->CR |= ADC_CR_ADCAL;
  // ADC2->CR |= ADC_CR_ADCAL;
  // ADC3->CR |= ADC_CR_ADCAL;
  // ADC4->CR |= ADC_CR_ADCAL;

  // while(ADC1->CR & ADC_CR_ADCAL);
  // while(ADC2->CR & ADC_CR_ADCAL);
  // while(ADC3->CR & ADC_CR_ADCAL);
  // while(ADC4->CR & ADC_CR_ADCAL);
  
  // common
  ADC12_COMMON->CCR |= ADC_CCR_VREFEN | ADC_CCR_CKMODE_0 | ADC_CCR_MDMA_1 | ADC_CCR_DMACFG | ADC_CCR_DUAL_0;
  ADC34_COMMON->CCR |= ADC_CCR_VREFEN | ADC_CCR_CKMODE_0 | ADC_CCR_MDMA_1 | ADC_CCR_DMACFG | ADC_CCR_DUAL_0;
  // ADC12_COMMON->CCR |= ADC_CCR_VREFEN | ADC_CCR_CKMODE_0;
  // ADC34_COMMON->CCR |= ADC_CCR_VREFEN | ADC_CCR_CKMODE_0;

  // enable
  ADC1->CR |= ADC_CR_ADEN;
  ADC2->CR |= ADC_CR_ADEN;
  ADC3->CR |= ADC_CR_ADEN;
  ADC4->CR |= ADC_CR_ADEN;

  ADC1->CR |= ADC_CR_ADSTART;
  ADC2->CR |= ADC_CR_ADSTART;
  ADC3->CR |= ADC_CR_ADSTART;
  ADC4->CR |= ADC_CR_ADSTART;

  // adc dma
  RCC->AHBENR |= RCC_AHBENR_DMA1EN | RCC_AHBENR_DMA2EN;
  delay_counter = 1 * SystemCoreClock / 1000000;
  while(delay_counter != 0U){
    delay_counter--;
  }

  // p -> m, circ, 12x
  DMA1_Channel1->CNDTR = 3 * 5;
  DMA1_Channel1->CMAR = &(adc12_buffer[0]);
  DMA1_Channel1->CPAR = &(ADC12_COMMON->CDR);
  DMA1_Channel1->CCR |= DMA_CCR_PL_1 | DMA_CCR_PL_0 | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;
  
  DMA2_Channel5->CNDTR = 3 * 2;
  DMA2_Channel5->CMAR = &(adc34_buffer[0]);
  DMA2_Channel5->CPAR = &(ADC34_COMMON->CDR);
  DMA2_Channel5->CCR |= DMA_CCR_PL_1 | DMA_CCR_PL_0 | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;


  MX_USB_DEVICE_Init();
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  /* USER CODE BEGIN 2 */
  hal_init(1.0 / 5000.0, 0.0);
  hal_parse("load term");
  hal_parse("term0.rt_prio = 10");

  hal_parse("flashloadconf");
  hal_parse("loadconf");

  TIM1->CR1 |= TIM_CR1_CEN;

  hal_parse("start");

  // GPIOC->MODER |= GPIO_MODER_MODER15_0;
  // GPIOD->MODER |= GPIO_MODER_MODER2_0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    cdc_poll();
    hal_run_nrt(); 

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_TIM1;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
