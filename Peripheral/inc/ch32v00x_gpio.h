/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v00x_gpio.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/02/27
 * Description        : This file contains all the functions prototypes for the
 *                      GPIO firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32V00x_GPIO_H
#define __CH32V00x_GPIO_H

#ifdef __cplusplus
extern "C"
 {
#endif

#include <ch32v00x.h>

#define _INLINE_GPIO_

/* Output Maximum frequency selection */
typedef enum
 {
 GPIO_Speed_10MHz = 1,
 GPIO_Speed_2MHz,
 GPIO_Speed_30MHz
 } GPIOSpeed_TypeDef;

#define GPIO_Speed_50MHz GPIO_Speed_30MHz

/* Configuration Mode enumeration */
typedef enum
 {
 GPIO_Mode_AIN = 0x0,
 GPIO_Mode_IN_FLOATING = 0x04,
 GPIO_Mode_IPD = 0x28,
 GPIO_Mode_IPU = 0x48,
 GPIO_Mode_Out_OD = 0x14,
 GPIO_Mode_Out_PP = 0x10,
 GPIO_Mode_AF_OD = 0x1C,
 GPIO_Mode_AF_PP = 0x18
 } GPIOMode_TypeDef;

/* GPIO Init structure definition */
typedef struct
 {
  uint16_t GPIO_Pin; /* Specifies the GPIO pins to be configured.
   This parameter can be any value of @ref GPIO_pins_define */

  GPIOSpeed_TypeDef GPIO_Speed; /* Specifies the speed for the selected pins.
   This parameter can be a value of @ref GPIOSpeed_TypeDef */

  GPIOMode_TypeDef GPIO_Mode; /* Specifies the operating mode for the selected pins.
   This parameter can be a value of @ref GPIOMode_TypeDef */
 } GPIO_InitTypeDef;

/* Bit_SET and Bit_RESET enumeration */
typedef enum
 {
 Bit_RESET = 0,
 Bit_SET
 } BitAction;

/* GPIO_pins_define */
#define GPIO_Pin_0                     ((uint16_t)0x0001) /* Pin 0 selected */
#define GPIO_Pin_1                     ((uint16_t)0x0002) /* Pin 1 selected */
#define GPIO_Pin_2                     ((uint16_t)0x0004) /* Pin 2 selected */
#define GPIO_Pin_3                     ((uint16_t)0x0008) /* Pin 3 selected */
#define GPIO_Pin_4                     ((uint16_t)0x0010) /* Pin 4 selected */
#define GPIO_Pin_5                     ((uint16_t)0x0020) /* Pin 5 selected */
#define GPIO_Pin_6                     ((uint16_t)0x0040) /* Pin 6 selected */
#define GPIO_Pin_7                     ((uint16_t)0x0080) /* Pin 7 selected */
#define GPIO_Pin_All                   ((uint16_t)0xFFFF) /* All pins selected */

/* GPIO_Remap_define */
#define GPIO_Remap_SPI1                ((uint32_t)0x00000001) /* SPI1 Alternate Function mapping */
#define GPIO_PartialRemap_I2C1         ((uint32_t)0x10000002) /* I2C1 Partial Alternate Function mapping */
#define GPIO_FullRemap_I2C1            ((uint32_t)0x10400002) /* I2C1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_USART1      ((uint32_t)0x80000004) /* USART1 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_USART1      ((uint32_t)0x80200000) /* USART1 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_USART1          ((uint32_t)0x80200004) /* USART1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_TIM1        ((uint32_t)0x00160040) /* TIM1 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM1        ((uint32_t)0x00160080) /* TIM1 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_TIM1            ((uint32_t)0x001600C0) /* TIM1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_TIM2        ((uint32_t)0x00180100) /* TIM2 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM2        ((uint32_t)0x00180200) /* TIM2 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_TIM2            ((uint32_t)0x00180300) /* TIM2 Full Alternate Function mapping */
#define GPIO_Remap_PA1_2               ((uint32_t)0x00008000) /* PA1 and PA2 Alternate Function mapping */
#define GPIO_Remap_ADC1_ETRGINJ        ((uint32_t)0x00200002) /* ADC1 External Trigger Injected Conversion remapping */
#define GPIO_Remap_ADC1_ETRGREG        ((uint32_t)0x00200004) /* ADC1 External Trigger Regular Conversion remapping */
#define GPIO_Remap_LSI_CAL             ((uint32_t)0x00200080) /* LSI calibration Alternate Function mapping */
#define GPIO_Remap_SDI_Disable         ((uint32_t)0x00300400) /* SDI Disabled */

/* GPIO_Port_Sources */
#define GPIO_PortSourceGPIOA           ((uint8_t)0x00)
#define GPIO_PortSourceGPIOC           ((uint8_t)0x02)
#define GPIO_PortSourceGPIOD           ((uint8_t)0x03)

/* GPIO_Pin_sources */
#define GPIO_PinSource0                ((uint8_t)0x00)
#define GPIO_PinSource1                ((uint8_t)0x01)
#define GPIO_PinSource2                ((uint8_t)0x02)
#define GPIO_PinSource3                ((uint8_t)0x03)
#define GPIO_PinSource4                ((uint8_t)0x04)
#define GPIO_PinSource5                ((uint8_t)0x05)
#define GPIO_PinSource6                ((uint8_t)0x06)
#define GPIO_PinSource7                ((uint8_t)0x07)

void GPIO_DeInit(GPIO_TypeDef *GPIOx);
void GPIO_AFIODeInit(void);
void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef *GPIO_InitStruct);
#ifdef _INLINE_GPIO_
uint8_t inline GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint16_t inline GPIO_ReadInputData(GPIO_TypeDef *GPIOx);
uint8_t inline GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint16_t inline GPIO_ReadOutputData(GPIO_TypeDef *GPIOx);
void inline GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void inline GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void inline GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void inline GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t PortVal);
#else
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef *GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef *GPIOx);
void GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t PortVal);
#endif
void GPIO_PinLockConfig(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_EventOutputCmd(FunctionalState NewState);
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);

#ifdef _INLINE_GPIO_
/*********************************************************************
 * @fn      GPIO_ReadInputDataBit
 *
 * @brief   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *
 * @param    GPIO_Pin - specifies the port bit to read.
 *             This parameter can be GPIO_Pin_x where x can be (0..7).
 *
 * @return  The input port pin value.
 */
uint8_t inline GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
 {
  uint8_t bitstatus = 0x00;

  if ((GPIOx->INDR & GPIO_Pin) != (uint32_t) Bit_RESET)
   {
    bitstatus = (uint8_t) Bit_SET;
   }
  else
   {
    bitstatus = (uint8_t) Bit_RESET;
   }

  return bitstatus;
 }

/*********************************************************************
 * @fn      GPIO_ReadInputData
 *
 * @brief   Reads the specified GPIO input data port.
 *
 * @param   GPIOx: where x can be (A..D) to select the GPIO peripheral.
 *
 * @return  The input port pin value.
 */
uint16_t inline GPIO_ReadInputData(GPIO_TypeDef *GPIOx)
 {
  return ((uint16_t) GPIOx->INDR);
 }

/*********************************************************************
 * @fn      GPIO_ReadOutputDataBit
 *
 * @brief   Reads the specified output data port bit.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bit to read.
 *            This parameter can be GPIO_Pin_x where x can be (0..7).
 *
 * @return  none
 */
uint8_t inline GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
 {
  uint8_t bitstatus = 0x00;

  if ((GPIOx->OUTDR & GPIO_Pin) != (uint32_t) Bit_RESET)
   {
    bitstatus = (uint8_t) Bit_SET;
   }
  else
   {
    bitstatus = (uint8_t) Bit_RESET;
   }

  return bitstatus;
 }

/*********************************************************************
 * @fn      GPIO_ReadOutputData
 *
 * @brief   Reads the specified GPIO output data port.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *
 * @return  GPIO output port pin value.
 */
uint16_t inline GPIO_ReadOutputData(GPIO_TypeDef *GPIOx)
 {
  return ((uint16_t) GPIOx->OUTDR);
 }

/*********************************************************************
 * @fn      GPIO_SetBits
 *
 * @brief   Sets the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..7).
 *
 * @return  none
 */
void inline GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
 {
  GPIOx->BSHR = GPIO_Pin;
 }

/*********************************************************************
 * @fn      GPIO_ResetBits
 *
 * @brief   Clears the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..7).
 *
 * @return  none
 */
void inline GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
 {
  GPIOx->BCR = GPIO_Pin;
 }

/*********************************************************************
 * @fn      GPIO_WriteBit
 *
 * @brief   Sets or clears the selected data port bit.
 *
 * @param   GPIO_Pin - specifies the port bit to be written.
 *            This parameter can be one of GPIO_Pin_x where x can be (0..7).
 *          BitVal - specifies the value to be written to the selected bit.
 *            Bit_RESET - to clear the port pin.
 *            Bit_SET - to set the port pin.
 *
 * @return  none
 */
void inline GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction BitVal)
 {
  if (BitVal != Bit_RESET)
   {
    GPIOx->BSHR = GPIO_Pin;
   }
  else
   {
    GPIOx->BCR = GPIO_Pin;
   }
 }

/*********************************************************************
 * @fn      GPIO_Write
 *
 * @brief   Writes data to the specified GPIO data port.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *          PortVal - specifies the value to be written to the port output data register.
 *
 * @return  none
 */
void inline GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t PortVal)
 {
  GPIOx->OUTDR = PortVal;
 }
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CH32V00x_GPIO_H */
