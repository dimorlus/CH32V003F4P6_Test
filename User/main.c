// The following files are modified:
// ch32v00x_flash.c & ch32v00x_flash.h (void FlashOptionData(uint8_t data0, uint8_t data1) added)
// Link.ld (section ".no_init" added)
// Pins usage (CH32V003F4P6_MINI EVB):
// PD6/Rx
// PD5/Tx
// PD4/AN7
// PD0/LD1
// PD1/SWIO/LD2
// PC1/SDA
// PC2/SCL
// PC5/LD3
// PA1/XTAL1
// PA2/XTAL2
// PD3/AN4
// PD2/AN3
// PC4/LD4
// PC3/BTN
// PC0/PWM/LD7
// PC6/PWM/LD6
// PC7/PWM/LD5

//----------------------------------------------------------------------------------
#undef DEBUG
//#include "debug.h"

#define PRINT
//#define PRINT_ADC
//#define PRINT_AVG
#define PRINT_MEAN

#include "eusart1.h"

#define _PD7_ //Use PD7 as GPIO
#define MAGIC 0x55AA
//#define DELTA_ADC 20
#define DELTA_ADC 3
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
typedef union __attribute__((packed))
 {
  union
   {
    struct
     {
      uint8_t Btn_curr :1;
      uint8_t Btn_prev :1;
     };
    struct
     {
      uint8_t Btn :2;
      uint8_t Btn_cnt :6;
     };
   };
 } tBtn;
 //----------------------------------------------------------------------------------


/* Global Variable */
 //----------------------------------------------------------------------------------
static volatile uint32_t counter __attribute__ ((section (".no_init")));//persistent
static uint16_t magic __attribute__ ((section (".no_init")));
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
static volatile uint32_t dma_cntr = 0;
static volatile uint32_t dma_num = 0;
static volatile bool _100ms_flag = 0;
static volatile uint32_t _100ms = 0;
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
static void GPIO_INIT(void)
 {
  GPIO_InitTypeDef GPIO_InitStructure = {0};

 // disable SWDIO to allow GPIO usage.
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//  GPIO_PinRemapConfig(GPIO_Remap_SDI_Disable, ENABLE);


  //AFIO->PCFR1 = AFIO_PCFR1_SWJ_CFG_DISABLE; // disable SWDIO to allow GPIO usage.
  // use  WCH-LinkUtility Target|Clear All Code Flash-By Power Off for erase FLASH
  // before program
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  // PD0/LD1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;// | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

#ifdef _PD7_
  //Button1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //Input pulled Up
  GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif
  //Button
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //Input pulled Up
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //DMA interrupt indicator LD4, LD3
  //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
 }
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
#define ADC_NUMCHLS 4
static volatile uint16_t adc_buffer[ADC_NUMCHLS];

/*
 * initialize adc for DMA
 * D3/A4, D2/A3, C4/A2, A8
 */

static void adc_init(void)
 {
  // ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
  RCC->CFGR0 &= ~(0x1F << 11);

  // Enable GPIOD and ADC
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1;

  // PD3 is analog input chl 4
  GPIOD->CFGLR &= ~(0xf << (4 * 3));	// CNF = 00: Analog, MODE = 00: Input
  //                            ^
  //                            +- port pin number dma_cntr.e. PD3

  // PD2 is analog input chl 3
  GPIOD->CFGLR &= ~(0xf << (4 * 2));	// CNF = 00: Analog, MODE = 00: Input
  //                            ^
  //                            +- port pin number dma_cntr.e. PD2

  // PC4 is analog input chl 7
  GPIOD->CFGLR &= ~(0xf << (4 * 4));	// CNF = 00: Analog, MODE = 00: Input
  //                            ^
  //                            +- port pin number dma_cntr.e. PD4

  // Reset the ADC to init all regs
  RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
  RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

  // Set up four conversions on chl 4, 3, 2, Vref=1.2V (chl8)
  ADC1->RSQR1 = (ADC_NUMCHLS-1) << 20;	// 4 chls in the sequence
  ADC1->RSQR2 = 0; //SQ7..SQ12
  ADC1->RSQR3 = (4 << (5 * 0)) | (3 << (5 * 1)) | (7 << (5 * 2)) | (8 << (5 * 3)); //SQ1..SQ6
  //             ^         ^                                        ^         ^
  //             |         +- index=0                               |         +- index=3
  //             +----------- channel                               +----------- Vref=1.2V (chl8)

  // set sampling time for chl 4, 3, 2, Vref=1.2V (chl8)
  // 0:7 => 3/9/15/30/43/57/73/241 cycles
  ADC1->SAMPTR2 = (7 << (3 * 4)) | (7 << (3 * 3)) | (7 << (3 * 7)) | (7 << (3 * 8));
  //               ^         ^
  //               |         +- channel
  //               +----------- 241 cycles

  // turn on ADC
  ADC1->CTLR2 |= ADC_ADON;

  // Reset calibration
  ADC1->CTLR2 |= ADC_RSTCAL;
  while(ADC1->CTLR2 & ADC_RSTCAL);

  // Calibrate
  ADC1->CTLR2 |= ADC_CAL;
  while(ADC1->CTLR2 & ADC_CAL);

  // Turn on DMA
  RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

  //DMA1_Channel1 is for ADC
  DMA1_Channel1->PADDR = (uint32_t) &ADC1->RDATAR;
  DMA1_Channel1->MADDR = (uint32_t) adc_buffer;
  DMA1_Channel1->CNTR = ADC_NUMCHLS;
  DMA1_Channel1->CFGR =
    DMA_M2M_Disable |
    DMA_Priority_VeryHigh |
    DMA_MemoryDataSize_HalfWord |
    DMA_PeripheralDataSize_HalfWord |
    DMA_MemoryInc_Enable |
    DMA_Mode_Circular |
    DMA_DIR_PeripheralSRC |
    DMA_IT_TC; //DMA interrupt

  //NVIC_SetPriority(DMA1_Channel1_IRQn, (1<<7)); //We don't need to tweak priority.
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  // Turn on DMA channel 1
  DMA1_Channel1->CFGR |= DMA_CFGR1_EN;

  // enable scanning
  ADC1->CTLR1 |= ADC_SCAN;

  // Enable continuous conversion and DMA
  ADC1->CTLR2 |= ADC_CONT | ADC_DMA | ADC_EXTSEL;

  // start conversion
  ADC1->CTLR2 |= ADC_SWSTART;
 }
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
//low pass filters
static volatile uint16_t adc_avg[ADC_NUMCHLS] = {0};
static volatile uint16_t adc_mean[ADC_NUMCHLS] = {0};
static inline uint16_t avg(int idx)
{
 /* low pass filter */
 int32_t ii = (int32_t)(adc_buffer[idx] - adc_avg[idx]);
 ii /= 16;
 adc_avg[idx] += ii;
 return adc_avg[idx];
}

//moving mean
static inline uint16_t mean(int idx)
{
 adc_avg[idx]-=adc_avg[idx]/16;
 adc_avg[idx]+=adc_buffer[idx];

 return adc_mean[idx]=adc_avg[idx]/16;
}

//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel1_IRQHandler(void)
 {
  if(DMA1->INTFR & DMA1_FLAG_TC1)
   {
     dma_cntr++;
     GPIO_WriteBit(GPIOC, GPIO_Pin_4, (dma_cntr & 1)); // toggle DMA interrupt indicator
#ifdef PRINT_ADC
     TIM1->CH3CVR = 1023-adc_buffer[0]; //PC0 PWM = ADC0 (CH4/PD3)
     TIM1->CH2CVR = 1023-adc_buffer[2]; //PC7 PWM = ADC1 (CH3/PD2)
     TIM1->CH1CVR = 1023-adc_buffer[1];
#endif
#ifdef PRINT_MEAN
     TIM1->CH3CVR = 1023-mean(0);
     TIM1->CH1CVR = 1023-mean(1);
     TIM1->CH2CVR = 1023-mean(2);
     mean(3);
#endif
#ifdef PRINT_AVG
     TIM1->CH3CVR = 1023-avg(0); //PC0 PWM = ADC0 (CH4/PD3)
     TIM1->CH2CVR = 1023-avg(2); //PC7 PWM = ADC1 (CH3/PD2)
     TIM1->CH1CVR = 1023-avg(1);
     avg(3);
#endif
     DMA1->INTFCR = DMA_CTCIF1;
   }
 }
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
/*********************************************************************
 * @fn      IWDG_Init
 *
 * @brief   Initializes IWDG.
 *
 * @param   IWDG_Prescaler: specifies the IWDG Prescaler value.
 *            IWDG_Prescaler_4: IWDG prescaler set to 4.
 *            IWDG_Prescaler_8: IWDG prescaler set to 8.
 *            IWDG_Prescaler_16: IWDG prescaler set to 16.
 *            IWDG_Prescaler_32: IWDG prescaler set to 32.
 *            IWDG_Prescaler_64: IWDG prescaler set to 64.
 *            IWDG_Prescaler_128: IWDG prescaler set to 128.
 *            IWDG_Prescaler_256: IWDG prescaler set to 256.
 *          Reload: specifies the IWDG Reload value.
 *            This parameter must be a number between 0 and 0x0FFF.
 *
 * @return  none
 */
static void IWDG_Feed_Init(u16 prer, u16 rlr)
 {
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(prer);
  IWDG_SetReload(rlr);
  IWDG_ReloadCounter();
  IWDG_Enable();
 }
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
/*
 * initialize TIM1 for PWM
 */
#define GPIO_CNF_IN_ANALOG   0
#define GPIO_CNF_IN_FLOATING 4
#define GPIO_CNF_IN_PUPD     8
#define GPIO_CNF_OUT_PP      0
#define GPIO_CNF_OUT_OD      4
#define GPIO_CNF_OUT_PP_AF   8
#define GPIO_CNF_OUT_OD_AF   12

//----------------------------------------------------------------------------------
///******************************************************************************************
// * initialize TIM2 for PWM
// Timer 2 pin mappings by AFIO->PCFR1
//	00	AFIO_PCFR1_TIM2_REMAP_NOREMAP
//		D4		T2CH1ETR
//		D3		T2CH2
//		C0		T2CH3
//		D7		T2CH4  --note: requires disabling nRST in opt
//	01	AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP1
//		C5		T2CH1ETR_
//		C2		T2CH2_
//		D2		T2CH3_
//		C1		T2CH4_
//	10	AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP2
//		C1		T2CH1ETR_
//		D3		T2CH2
//		C0		T2CH3
//		D7		T2CH4  --note: requires disabling nRST in opt
//	11	AFIO_PCFR1_TIM2_REMAP_FULLREMAP
//		C1		T2CH1ETR_
//		C7		T2CH2_
//		D6		T2CH3_
//		D5		T2CH4_
// ******************************************************************************************/
//void t2pwm_init( void )
//{
//  // Enable GPIOC, GPIOD, TIM2, and AFIO *very important!*
//  RCC->APB2PCENR |= RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC;
//  RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
//
//  // PC5 is T2CH1_, 10MHz Output alt func, push-pull (also works in oepn drain OD_AF)
//  GPIOC->CFGLR &= ~(0xf<<(4*5));
//  GPIOC->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP_AF)<<(4*5);
//
//  // Reset TIM2 to init all regs
//  RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
//  RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;
//
//  // SMCFGR: default clk input is CK_INT
//  // set TIM2 clock prescaler divider
//  TIM2->PSC = 0x0000;
//  // set PWM total cycle width
//  TIM2->ATRLR = 1023; //10bit (255 - 8bit);
//
//  // for channel 1 and 2, let CCxS stay 00 (output), set OCxM to 110 (PWM I)
//  // enabling preload causes the new pulse width in compare capture register only to come into effect when UG bit in SWEVGR is set (= initiate update) (auto-clears)
//  TIM2->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1PE;
// // TIM2->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2PE;
//
//  // CTLR1: default is up, events generated, edge align
//  // enable auto-reload of preload
//  TIM2->CTLR1 |= TIM_ARPE;
//
//  // Enable CH1 output, positive pol
//  TIM2->CCER |= TIM_CC1E | TIM_CC1P;
//  // Enable CH2 output, positive pol
//  //TIM2->CCER |= TIM_CC2E | TIM_CC2P;
//
//  AFIO->PCFR1 |= AFIO_PCFR1_TIM2_REMAP_FULLREMAP;
//  // initialize counter
//  TIM2->SWEVGR |= TIM_UG;
//
//  // Enable TIM2
//  TIM2->CTLR1 |= TIM_CEN;
//}
////----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
static void t1pwm_init(void)
{
  // Enable GPIOC and TIM1 and AFIO *very important!*
  RCC->APB2PCENR |= RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1;

  // PC0 is T1CH3, 30MHz Output alt func, push-pull
  GPIOC->CFGLR &= ~(0xf<<(4*0));
  GPIOC->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP_AF)<<(4*0);

  // PC7 is T1CH2, 30MHz Output alt func, push-pull
  GPIOC->CFGLR &= ~(0xf<<(4*7));
  GPIOC->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP_AF)<<(4*7);

  // PC6 is T1CH1, 30MHz Output alt func, push-pull
  GPIOC->CFGLR &= ~(0xf<<(4*6));
  GPIOC->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP_AF)<<(4*6);


  // Reset TIM1 to init all regs
  RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
  RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

  // CTLR1: default is up, events generated, edge align
  // SMCFGR: default clk input is CK_INT

  // Prescaler
  TIM1->PSC = 0x0000;

  // Auto Reload - sets period
  TIM1->ATRLR = 1023;//10bit (255 - 8bit);

  // Reload immediately
  TIM1->SWEVGR |= TIM_UG;

  // Enable CH1 output, positive pol
  TIM1->CCER |= TIM_CC1E | TIM_CC1P;

  // Enable CH2 output, positive pol
  TIM1->CCER |= TIM_CC2E | TIM_CC2P;

  // Enable CH3 output, positive pol
  TIM1->CCER |= TIM_CC3E | TIM_CC3P;

  // CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
  TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;

  // CH2 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
  TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1;

  // CH3 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
  TIM1->CHCTLR2 |= TIM_OC3M_2 | TIM_OC3M_1;

  // Set the Capture Compare Register value to 50% initially
  TIM1->CH1CVR = 512;
  TIM1->CH4CVR = 512;
  TIM1->CH2CVR = 512;
  TIM1->CH3CVR = 512;

  // Enable TIM1 outputs
  TIM1->BDTR |= TIM_MOE;

  AFIO->PCFR1 |= AFIO_PCFR1_TIM1_REMAP_PARTIALREMAP; //RM=01

  // Enable TIM1
  TIM1->CTLR1 |= TIM_CEN;
}
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
/*
 * set timer channel PW
 */
//static void t1pwm_setpw(uint8_t chl, uint16_t width)
// {
//  switch(chl&3)
//   {
//    case 0: TIM1->CH1CVR = width; break;
//    case 1: TIM1->CH2CVR = width; break;
//    case 2: TIM1->CH3CVR = width; break;
//    case 3: TIM1->CH4CVR = width; break;
//   }
// }
//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
static inline uint8_t Button(void)
 {
  static volatile tBtn Btn = {0};
  //Btn.Btn_curr = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0)?0:1;
  Btn.Btn_curr = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3) ? 0 : 1;
  switch (Btn.Btn)
   {
   case 1: //press
    Btn.Btn_cnt = 1;
   break;
   case 0: //released
   case 2: //release
    Btn.Btn_cnt = 0;
   break;
   case 3: //pressed
    if (Btn.Btn_cnt < 63) Btn.Btn_cnt++;
   break;
   }
  Btn.Btn_prev = Btn.Btn_curr;
  return Btn.Btn_cnt;
 }
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
static inline uint8_t RButton(void)
 {
  static volatile tBtn Btn = {0};
  uint8_t ret = 0;
  //Btn.Btn_curr = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0)?0:1;
  Btn.Btn_curr = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3) ? 0 : 1;
  switch (Btn.Btn)
   {
   case 1: //press
    Btn.Btn_cnt = 1;
   break;
   case 0: //released
    Btn.Btn_cnt = 0;
   break;
   case 2: //release
    ret = Btn.Btn_cnt;
    Btn.Btn_cnt = 0;
   break;
   case 3: //pressed
    if (Btn.Btn_cnt < 63) Btn.Btn_cnt++;
   break;
   }
  Btn.Btn_prev = Btn.Btn_curr;
  return ret;
 }
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
static inline uint8_t RButton1(void)
 {
  static volatile tBtn Btn = {0};
  uint8_t ret = 0;
  Btn.Btn_curr = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) ? 0 : 1;
  switch (Btn.Btn)
   {
   case 1: //press
    Btn.Btn_cnt = 1;
   break;
   case 0: //released
    Btn.Btn_cnt = 0;
   break;
   case 2: //release
    ret = Btn.Btn_cnt;
    Btn.Btn_cnt = 0;
   break;
   case 3: //pressed
    if (Btn.Btn_cnt < 63) Btn.Btn_cnt++;
   break;
   }
  Btn.Btn_prev = Btn.Btn_curr;
  return ret;
 }
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      SysTick_IRQHandler
 *
 * @brief   SysTick Interrupt Service Function.
 *
 * @return  none
 */
void SysTick_Handler(void)
 {
  uint8_t btn = RButton();
  _100ms++;
  //GPIO_WriteBit(GPIOD, GPIO_Pin_7, (_100ms & 1));
  if (btn)
   {
    if (btn < 5) counter++;

    GPIO_WriteBit(GPIOD, GPIO_Pin_0, (counter & 1));
    GPIO_WriteBit(GPIOD, GPIO_Pin_1, (counter & 2));
    GPIO_WriteBit(GPIOC, GPIO_Pin_5, (counter & 4));


    //printf("Systick %d %d\r\n", btn, counter);
   }

//  printf( "%4d %4d ", adc_buffer[0], adc_buffer[1]);
//  printf( "%4d %4d  %d\r\n", adc_buffer[2], adc_buffer[3], dma_cntr);
  dma_num = dma_cntr;
  dma_cntr = 0;
  _100ms_flag = 1;
  SysTick->SR = 0;
 }
//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
/*********************************************************************
 * @fn      Option_Byte_CFG
 *
 * @brief   Config Option byte and disable reset pin.
 *
 * @return  none
 *
 * Bad solution. Each time it erase Option sector (including Data bytes) and
 * flash it again. Use FlashOptionUser() and FlashOptionData() instead
 * FlashOptionUser(0x20df);// b=0x20;((~b)&0xff)|(b<<8) RD7=GPIO
 * FlashOptionUser(0x08f7);// b=0x08;((~b)&0xff)|(b<<8) Default value RD7=nRST
 *
 */
//static void Option_Byte_CFG(void)
//{
// FLASH_Unlock();
// FLASH_EraseOptionBytes();
// FLASH_UserOptionByteConfig(OB_IWDG_SW, OB_STOP_NoRST, OB_STDBY_NoRST, OB_RST_NoEN);
// FLASH_Lock();
//}
//----------------------------------------------------------------------------------
#ifndef DEBUG
__attribute__((used))
int _write(int fd, char *buf, int size)
 {
  int i = 0;
  int writeSize = size;
  for (i = 0; i < size; i++)
   {
//    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//    USART_SendData(USART1, *buf++);
    EUSART1_Write(*buf++);
   }
  return writeSize;
 }

//----------------------------------------------------------------------------------
/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
__attribute__((used))
void *_sbrk(ptrdiff_t incr)
 {
  extern char _end[];
  extern char _heap_end[];
  static char *curbrk = _end;

  if ((curbrk + incr < _end) || (curbrk + incr > _heap_end)) return NULL - 1;

  curbrk += incr;
  return curbrk - incr;
 }
#endif //DEBUG
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//inline uint8_t delta(uint32_t v1, uint32_t v2) __attribute__((always_inline));
static inline uint8_t delta(uint32_t v1, uint32_t v2)
 {
  uint32_t ret;
  ret = v1>v2?v1-v2:v2-v1;
  if (ret > 255) return 255;
  else return ret & 0xff;
 }
//----------------------------------------------------------------------------------
//follow data programs to FLASH via programmer. See .eesegment definition in the Link.ld file
//This area placed to the last 64 bytes of the FLASH memory. Address is 0x00003fc0. We have to
//add 0x08000000 to it for FLASH API usage.
const uint8_t ee[64] __attribute__((section(".eesegment"))) =
  {
   0, 1,  2,  3,  4,  5,  6,  7,
   8, 9, 10, 11, 12, 13, 13, 15
  };
//----------------------------------------------------------------------------------
static uint8_t buf[64];

static void FlashTest(void)
 {
  FLASH_Status s;

  //EE and ptr points physically the same area
  const uint8_t const * ptr = (uint8_t*)(0x08000000L+(uint32_t)&ee[0]);

  printf("@EE:%08x\r\n", (uint32_t)&ee[0]);
  {
    int i;
    for(i=0; i<16;i++) printf("%02x ", ee[i]);
    printf("\r\n");
  }

  {
   printf("@PEE:%08x\r\n", (uint32_t)ptr);
   {
     int i;
     for(i=0; i<16;i++) printf("%02x ", ptr[i]);
     printf("\r\n");
   }
  }

  printf("Erase\n\r");
  s = FLASH_ROM_ERASE((uint32_t)ptr, 64);
  if(s != FLASH_COMPLETE)
  {
      printf("check FLASH_ADR_RANGE_ERROR FLASH_ALIGN_ERROR or FLASH_OP_RANGE_ERROR\r\n");
      return;
  }
  printf("@PEE:%08x\r\n", (uint32_t)ptr);
  {
    int i;
    for(i=0; i<16;i++) printf("%02x ", ptr[i]);
    printf("\r\n");
  }
  printf("@EE:%08x\r\n", (uint32_t)&ee[0]);
  {
    int i;
    for(i=0; i<16;i++) printf("%02x ", ee[i]);
    printf("\r\n");
  }

  //fill the buffer
  {
   int i;
   for(i=0; i<64; i++) buf[i] = 63-(uint8_t)i;
  }

  printf("Write\n\r");
  s = FLASH_ROM_WRITE((uint32_t)ptr,  (uint32_t *)buf, 64);
  if(s != FLASH_COMPLETE)
  {
      printf("check FLASH_ADR_RANGE_ERROR FLASH_ALIGN_ERROR or FLASH_OP_RANGE_ERROR\r\n");
      return;
  }
  printf("@EE:%08x\r\n", (uint32_t)&ee[0]);
  {
    int i;
    for(i=0; i<16;i++) printf("%02x ", ee[i]);
    printf("\r\n");
  }
  printf("@PEE:%08x\r\n", (uint32_t)ptr);
  {
    int i;
    for(i=0; i<16;i++) printf("%02x ", ptr[i]);
    printf("\r\n");
  }
 }
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
int main(void)
 {
  SystemCoreClockUpdate();
#ifdef DEBUG
  Delay_Init();
#endif
#ifdef _PD7_
  FlashOptionUser(0x20df);// b=0x20;((~b)&0xff)|(b<<8) RD7=GPIO
#else
  FlashOptionUser(0x08f7);// b=0x08;((~b)&0xff)|(b<<8) Default value RD7=nRST
#endif
  if (magic != 0x55aa)
   {
    //Power ON reset
    counter = 0;
   }
  magic = 0x55aa;

  EUSART1_Initialize();

  // init TIM1 for PWM

#ifdef PRINT
#ifdef DEBUG
#if (SDI_PRINT == SDI_PR_OPEN)
  SDI_Printf_Enable();
#else
  USART_Printf_Init(115200);
#endif
#endif //DEBUG
  printf("SystemClk:%d\r\n", SystemCoreClock);
  printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
  printf("User:%08x\n\r", *(uint32_t *)OB_BASE);
#endif
  uint8_t bootcnt = OB->Data0;
  bootcnt++;
  FlashOptionData(bootcnt, 0);
#ifdef PRINT
  printf("Boot count is %d\r\n", bootcnt);
#endif
  GPIO_INIT();

  //NVIC_SetPriority(SysTicK_IRQn, (1<<6)); //We don't need to tweak priority.
  NVIC_EnableIRQ(SysTicK_IRQn);

  SysTick->SR &= ~(1 << 0);
  SysTick->CMP = (SystemCoreClock / 10) - 1; //100ms period
  SysTick->CNT = 0;
  SysTick->CTLR = 0xF;

  IWDG_Feed_Init(IWDG_Prescaler_32, 4000);   // 1s IWDG reset

  adc_init();

// init op-amp
// opamp_init();

  FlashTest();

  t1pwm_init();

  GPIO_WriteBit(GPIOD, GPIO_Pin_0, (counter & 1));
  GPIO_WriteBit(GPIOD, GPIO_Pin_1, (counter & 2));
  GPIO_WriteBit(GPIOC, GPIO_Pin_5, (counter & 4));

  {
   uint16_t adc_buffer_prev[ADC_NUMCHLS] = {0};
   while(1) //main loop
    {
     while (EUSART1_is_rx_ready()) //UART echo
      {
       char c = EUSART1_Read();
       EUSART1_Write(c);
      }
 #ifdef _PD7_
     if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) == 1)  //PD7
 #endif
      {
       IWDG_ReloadCounter();   //Feed dog
      }
#ifdef PRINT
     if (_100ms_flag)
      {
	_100ms_flag = 0;
#ifdef PRINT_ADC
	if ((delta(adc_buffer_prev[0], adc_buffer[0]) >= DELTA_ADC)||
	    (delta(adc_buffer_prev[1], adc_buffer[1]) >= DELTA_ADC)||
	    (delta(adc_buffer_prev[2], adc_buffer[2]) >= DELTA_ADC))
	 { //print if changed
	  printf( "%4d %4d ", adc_buffer[0], adc_buffer[1]);
	  printf( "%4d %4d  %d\r\n", adc_buffer[2], adc_buffer[3], dma_num);
	  adc_buffer_prev[0] = adc_buffer[0];
	  adc_buffer_prev[1] = adc_buffer[1];
	  adc_buffer_prev[2] = adc_buffer[2];
	 }
#endif
#ifdef PRINT_AVG
	if ((delta(adc_buffer_prev[0], adc_avg[0]) > DELTA_ADC)||
	    (delta(adc_buffer_prev[1], adc_avg[1]) > DELTA_ADC)||
	    (delta(adc_buffer_prev[2], adc_avg[2]) > DELTA_ADC))
	 { //print if changed
	  printf( "%4d %4d ", adc_avg[0], adc_avg[1]);
	  printf( "%4d %4d  %d\r\n", adc_avg[2], adc_avg[3], dma_num);
	  adc_buffer_prev[0] = adc_avg[0];
	  adc_buffer_prev[1] = adc_avg[1];
	  adc_buffer_prev[2] = adc_avg[2];
	 }
#endif
#ifdef PRINT_MEAN
	if ((delta(adc_buffer_prev[0], adc_mean[0]) > DELTA_ADC)||
	    (delta(adc_buffer_prev[1], adc_mean[1]) > DELTA_ADC)||
	    (delta(adc_buffer_prev[2], adc_mean[2]) > DELTA_ADC))
	 { //print if changed
	  printf( "%4d %4d ", adc_mean[0], adc_mean[1]);
	  printf( "%4d %4d  %d\r\n", adc_mean[2], adc_mean[3], dma_num);
	  adc_buffer_prev[0] = adc_mean[0];
	  adc_buffer_prev[1] = adc_mean[1];
	  adc_buffer_prev[2] = adc_mean[2];
	 }
#endif
      }
#endif
    }
  }
 }
//----------------------------------------------------------------------------------

