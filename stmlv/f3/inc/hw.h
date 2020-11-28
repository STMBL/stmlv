#pragma once

#define RED_PIN GPIO_Pin_15
#define RED_PORT GPIOC

#define PWM_FREQ (15000)
#define PWM_RES (72000000 / PWM_FREQ)

#define ADC_REF (3.3)     //analog reference voltage
#define ADC_RES (4096.0)  //analog resolution, 12 bit

#define OP_R_INPUT (10000.0)     //opamp input
#define OP_R_FEEDBACK (15000.0)  //opamp feedback
#define OP_R_OUT_LOW (470.0)     //opamp out low
#define OP_R_OUT_HIGH (22.0)     //opamp out high
#define OP_REF (ADC_REF / 2.0)            //opamp reference voltage

#define SHUNT_R (0.002)
#define SHUNT_SERIES_R (470.0)
#define SHUNT_PULLUP_R (15000.0)
#define SHUNT_OP_GAIN (16.0)

#define CURRENT_OFFSET (ADC_REF / (SHUNT_PULLUP_R + SHUNT_SERIES_R) * SHUNT_SERIES_R / SHUNT_R)
#define CURRENT_SCALE (-1.0 / SHUNT_OP_GAIN / SHUNT_R)

#define AIN_SCALE ((560.0 + 1000.0) / 1000.0)
#define DC_SCALE ((22000.0 + 22000.0 + 1500.0) / 1500.0)

#pragma pack(push, 1)
struct adc12_struct_t{
  uint16_t iw;
  uint16_t iv;
  uint16_t cos;
  uint16_t sin;
  uint16_t a1;
  uint16_t a0;
  uint16_t dc;
  uint16_t a2;
  uint16_t temp;
  uint16_t a3;
};

struct adc34_struct_t{
  uint16_t iu;
  uint16_t a4;
  uint16_t vref3;
  uint16_t vref4;
};
#pragma pack(pop)

#define IU (1)
#define IV (3)
#define IW (3)

#define IUi (12)
#define IVi (4)
#define IWi (2)

#define A0 (2)
#define A1 (5)
#define A2 (5)
#define A3 (11)
#define A4 (3)

#define SIN (7)
#define COS (6)

#define TEMP (9)
#define DC (8)

#define VREF (18)



#define NTC_TEMP0 (25.0)
#define NTC_TEMP1 (80.0)

#define NTC_R0 (4700.0)
#define NTC_B (3539.0)

#define NTC_PULLUP (1000.0)

/*

FB0_A PB4 PB11 PC6 
FB0_B PB3 PC7
FB0_C PB5 PB10 PC8
FB0_A_EN
FB0_B_EN
FB0_C_EN


AIN0 PA5 adc2_2
AIN1 PF4 adc1_5
AIN2 PC4 adc2_5
AIN3 PC5 adc2_11
AIN4_STO PB12 adc4_3

SIN PC1 adc2_7
COS PC0 adc1_6

DC_LINK PC2 adc1_8
TEMP PC3 adc1_9


iu pb1 opamp3 adc3
pb0
iv pa6 opamp2 adc2 
pa7
iw pa2 opamp1 adc1
pa1



adc1
 3, 6, 5, 8, 9
 iw, cos, a1, dc, temp
 iw, iw, cos, a1 15, 15, 6, 5
 iw, iw, cos, temp 15, 15, 6, 9
 iw, iw, cos, dc 15, 15, 6, 8

adc2
 3, 7, 2, 5, 11
 iv, sin, a0, a2, a3
 iv, iv, sin, a0 17, 17, 7, 2
 iv, iv, sin, a2 17, 17, 7, 5
 iv, iv, sin, a3 17, 17, 7, 11

adc3
 1, 18
 iu
 iu, iu, vref, vref 17, 17, 18, 18
 iu, iu, vref, vref 17, 17, 18, 18
 iu, iu, vref, vref 17, 17, 18, 18

adc4
 3, 18
 a4, a4, vref, vref 3, 3, 18, 18
 a4, a4, vref, vref 3, 3, 18, 18
 a4, a4, vref, vref 3, 3, 18, 18




pwm
 pa8 tim1_ch1
 pa9 tim1_ch2
 pa10 tim1_ch3
 pb13 tim1_ch1n
 pb14 tim1_ch2n
 pb15 tim1_ch3n
 pb12 tim1_bkin sto

cur
 pa1 op1_+, comp1_+
 pa2 op1_out
 pa7 op2_+, comp2_+
 pa6 op2_out
 pb0 op3_+, comp4_+
 pb1 op3_out
 pa4 dac_out1

usb
 pa11 -
 pa12 +

swd 
 pa13 swdio
 pa14 swclk
 nrst

fb0
 A
  pc6 tim3_ch1
  pb11 uart3_rx
  pb4 spi1_miso
 B
  pc7 tim3_ch2
  pb3 spi1_ck (uart2_tx)
  b_en
 C
  pc8 tim3_ch3
  pb10 uart3_tx
  pb5 spi1_mosi
  c_en pc9 tim3_ch4

fb1
 A
  pa15 tim8_ch1
  pb7 uart1_rx
  pc11 spi3_miso
  adc
 B
  pb8 tim8_ch2
  pc10 spi3_ck
  adc
  b_en tim2_ch4
 C
  pb9 tim8_ch3
  pb6 uart1_tx
  pc12 spi3_mosi
  c_en tim2_ch1

io
 adc2_5 uart1_tx hall
 adc2_11 uart1_rx hall
 adc hall
 adc ntc_mot
spi3 + cs solder jumper to fb1

misc
 adc dclink
 adc ntc_fet
 error_led


fb0
dma1_ch2 dma1_ch3 + dma1_ch6 + (dma1_ch7)
uart3 + spi1 + tim3 + (uart2)

fb1
dma1_ch4 dma1_ch5 + dma2_ch1 dma2_ch2 
uart1 + spi3 + tim8
+ tim2
sin/cos

adc
dma1_ch1 dma2_ch5
adc1 adc3
*/