// This MounRiver project example shows SysTick interrupt, GPIO, A2D&DMA, DMA interrupt, button reading,
// Option area flash, persistent (non initialize) variables, Watchdog, UART debug printf and
// PD1/SWIO as GPIO usage (commented in void GPIO_INIT(void)).
// The following files are modified:
// ch32v00x_flash.c & ch32v00x_flash.h (void FlashOptionData(uint8_t data0, uint8_t data1) added)
// Link.ld (section ".no_init" added)
// Pins usage (CH32V003F4P6_MINI EVB):
// PD6/Rx
// PD5/Tx
// PD4/LD3
// PD0/LD1
// PD1/SWIO/LD2
// PC1/BTN
// PA1/XTAL1
// PA2/XTAL2
// PD3/AN4
// PD2/AN3
// PC4/AN2

#include "debug.h"

#define MAGIC 0x55AA

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

/* Global Variable */
static volatile uint32_t counter __attribute__ ((section (".no_init")));//persistent
static uint16_t magic __attribute__ ((section (".no_init")));
static volatile tBtn Btn0 = {0};
static volatile uint8_t dma_cntr = 0;
static volatile uint8_t dma_num = 0;
static volatile uint8_t _100ms_flag = 0;

void GPIO_INIT(void)
 {
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  //AFIO->PCFR1 = AFIO_PCFR1_SWJ_CFG_DISABLE; // disable SWDIO to allow GPIO usage.
  // use  WCH-LinkUtility Target|Clear All Code Flash-By Power Off for erase FLASH
  // before program
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_4;// | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  //Button
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //Input pulled Up
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //DMA interrupt indicator
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

 }


void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel1_IRQHandler(void)
 {
//  dma_cntr++;
//  GPIO_WriteBit(GPIOC, GPIO_Pin_2, (dma_cntr & 1)); // toggle DMA interrupt indicator
//  DMA1->INTFCR = DMA1_IT_GL1;

  if(DMA1->INTFR & DMA1_FLAG_TC1)
   {
     dma_cntr++;
     GPIO_WriteBit(GPIOC, GPIO_Pin_2, (dma_cntr & 1)); // toggle DMA interrupt indicator
     DMA1->INTFCR = DMA_CTCIF1;
   }

 }

#define ADC_NUMCHLS 4

volatile uint16_t adc_buffer[ADC_NUMCHLS];

/*
 * initialize adc for DMA
 * D3/A4, D2/A3, C4/A2, A8
 */

void adc_init(void)
 {
  // ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
  RCC->CFGR0 &= ~(0x1F << 11);

  // Enable GPIOD and ADC
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1;

  // PD3 is analog input chl 4
  GPIOD->CFGLR &= ~(0xf << (4 * 3));	// CNF = 00: Analog, MODE = 00: Input
  //                            ^
  //                            +- port pin number dma_cntr.e. PD3

  // PD2 is analog input chl 3
  GPIOD->CFGLR &= ~(0xf << (4 * 2));	// CNF = 00: Analog, MODE = 00: Input
  //                            ^
  //                            +- port pin number dma_cntr.e. PD2

  // PC4 is analog input chl 2
  GPIOC->CFGLR &= ~(0xf << (4 * 4));	// CNF = 00: Analog, MODE = 00: Input
  //                            ^
  //                            +- port pin number dma_cntr.e. PC4

  // Reset the ADC to init all regs
  RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
  RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

  // Set up four conversions on chl 4, 3, 2, Vref=1.2V (chl8)
  ADC1->RSQR1 = (ADC_NUMCHLS-1) << 20;	// 3 chls in the sequence
  ADC1->RSQR2 = 0; //SQ7..SQ12
  ADC1->RSQR3 = (4 << (5 * 0)) | (3 << (5 * 1)) | (2 << (5 * 2)) | (8 << (5 * 3)); //SQ1..SQ6
  //             ^         ^
  //             |         +- index
  //             +----------- channel

  // set sampling time for chl 4, 3, 2, Vref=1.2V (chl8)
  // 0:7 => 3/9/15/30/43/57/73/241 cycles
  ADC1->SAMPTR2 = (7 << (3 * 4)) | (7 << (3 * 3)) | (7 << (3 * 2)) | (7 << (3 * 8));
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

  NVIC_SetPriority(DMA1_Channel1_IRQn, (1<<7)); //We don't need to tweak priority.
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

/*
 * turn on op-amp, select input pins
 */
void opamp_init(void)
 {
  // turn on the op-amp
  EXTEN->EXTEN_CTR |= EXTEN_OPA_EN;

  // select op-amp pos pin: 0 = PA2, 1 = PD7
  //EXTEN->EXTEN_CTR |= EXTEN_OPA_PSEL;

  // select op-amp neg pin: 0 = PA1, 1 = PD0
  //EXTEN->EXTEN_CTR |= EXTEN_OPA_NSEL;
 }

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
void IWDG_Feed_Init(u16 prer, u16 rlr)
 {
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(prer);
  IWDG_SetReload(rlr);
  IWDG_ReloadCounter();
  IWDG_Enable();
 }

uint8_t Button(void)
 {
  //Btn0.Btn_curr = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0)?0:1;
  Btn0.Btn_curr = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) ? 0 : 1;
  switch (Btn0.Btn)
   {
   case 1: //press
    Btn0.Btn_cnt = 1;
   break;
   case 0: //released
   case 2: //release
    Btn0.Btn_cnt = 0;
   break;
   case 3: //pressed
    if (Btn0.Btn_cnt < 63) Btn0.Btn_cnt++;
   break;
   }
  Btn0.Btn_prev = Btn0.Btn_curr;
  return Btn0.Btn_cnt;
 }

uint8_t RButton(void)
 {
  uint8_t ret = 0;
  //Btn0.Btn_curr = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0)?0:1;
  Btn0.Btn_curr = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) ? 0 : 1;
  switch (Btn0.Btn)
   {
   case 1: //press
    Btn0.Btn_cnt = 1;
   break;
   case 0: //released
    Btn0.Btn_cnt = 0;
   break;
   case 2: //release
    ret = Btn0.Btn_cnt;
    Btn0.Btn_cnt = 0;
   break;
   case 3: //pressed
    if (Btn0.Btn_cnt < 63) Btn0.Btn_cnt++;
   break;
   }
  Btn0.Btn_prev = Btn0.Btn_curr;
  return ret;
 }

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
  if (btn)
   {
    if (btn < 5) counter++;

    GPIO_WriteBit(GPIOD, GPIO_Pin_0, (counter & 1));
    GPIO_WriteBit(GPIOD, GPIO_Pin_1, (counter & 2));
    GPIO_WriteBit(GPIOD, GPIO_Pin_4, (counter & 4));


    //printf("Systick %d %d\r\n", btn, counter);
   }

//  printf( "%4d %4d ", adc_buffer[0], adc_buffer[1]);
//  printf( "%4d %4d  %d\r\n", adc_buffer[2], adc_buffer[3], dma_cntr);
  dma_num = dma_cntr;
  dma_cntr = 0;
  _100ms_flag = 1;
  SysTick->SR = 0;
 }

//FLASH_Unlock();
//FLASH_EraseOptionBytes();
//FLASH_UserOptionByteConfig(OB_STOP_NoRST, OB_STDBY_NoRST, OB_RST_NoEN, OB_PowerON_Start_Mode_USER);
//FLASH_Unlock();

/*********************************************************************
 * @fn      Option_Byte_CFG
 *
 * @brief   Config Option byte and enable reset pin.
 *
 * @return  none
 */
void Option_Byte_CFG(void)
 {
  FLASH_Unlock();
  FLASH_EraseOptionBytes();
//FLASH_UserOptionByteConfig(OB_IWDG_SW, OB_STDBY_NoRST, OB_RST_EN_DT12ms, OB_PowerON_Start_Mode_BOOT);
  FLASH_UserOptionByteConfig(OB_IWDG_SW, OB_STDBY_NoRST, OB_RST_NoEN, OB_PowerON_Start_Mode_USER);
  FLASH_Lock();
 }

/*********************************************************************
 * @fn      KEY_PRESS
 *
 * @brief   Key processing funcation.
 *
 * @return  0 - Press the key.
 *          1 - Release Key.
 */

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
 {
  SystemCoreClockUpdate();
  Delay_Init();
  if (magic != 0x55aa) counter = 0;
  magic = 0x55aa;
#if (SDI_PRINT == SDI_PR_OPEN)
  SDI_Printf_Enable();
#else
  USART_Printf_Init(115200);
#endif
  printf("SystemClk:%d\r\n", SystemCoreClock);
  printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

  uint8_t bootcnt = OB->Data0;
  bootcnt++;
  FlashOptionData(bootcnt, 0);
  printf("Boot count is %d\r\n", bootcnt);

  GPIO_INIT();

  //NVIC_SetPriority(SysTicK_IRQn, (1<<6)); //We don't need to tweak priority.
  NVIC_EnableIRQ(SysTicK_IRQn);
  SysTick->SR &= ~(1 << 0);
  SysTick->CMP = (SystemCoreClock / 10) - 1; //100ms period
  SysTick->CNT = 0;
  SysTick->CTLR = 0xF;

  IWDG_Feed_Init(IWDG_Prescaler_32, 4000);   // 1s IWDG reset

      printf("\r\r\n\nadc_dma_opamp example\n\r");

      // init adc
      printf("initializing adc...");
      adc_init();
      printf("done.\n\r");

      // init op-amp
//	printf("initializing op-amp...\n\r");
//	opamp_init();
//	printf("done.\n\r");


  while(1)
   {
    //if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 1)  //PC1
     {
      IWDG_ReloadCounter();   //Feed dog
      if (_100ms_flag)
       {
	_100ms_flag = 0;
         printf( "%4d %4d ", adc_buffer[0], adc_buffer[1]);
         printf( "%4d %4d  %d\r\n", adc_buffer[2], adc_buffer[3], dma_num);
       }
     }
   }

  //while(1);
 }

