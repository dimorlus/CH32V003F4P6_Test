
/**
  Section: Included Files
*/
#include "eusart1.h"

/**
  Section: Macro Declarations
*/

#define EUSART1_TX_BUFFER_SIZE 8
#define EUSART1_RX_BUFFER_SIZE 8

/**
  Section: Global Variables
*/
volatile uint8_t eusart1TxHead = 0;
volatile uint8_t eusart1TxTail = 0;
volatile uint8_t eusart1TxBuffer[EUSART1_TX_BUFFER_SIZE];
volatile uint8_t eusart1TxBufferRemaining;

volatile uint8_t eusart1RxHead = 0;
volatile uint8_t eusart1RxTail = 0;
volatile uint8_t eusart1RxBuffer[EUSART1_RX_BUFFER_SIZE];
volatile eusart1_status_t eusart1RxStatusBuffer[EUSART1_RX_BUFFER_SIZE];
volatile uint8_t eusart1RxCount;
volatile eusart1_status_t eusart1RxLastError;

/**
  Section: EUSART1 APIs
*/

void EUSART1_Initialize(void)
 {
  eusart1RxLastError.status = 0;

  // initializing the driver state
  eusart1TxHead = 0;
  eusart1TxTail = 0;
  eusart1TxBufferRemaining = sizeof(eusart1TxBuffer);

  eusart1RxHead = 0;
  eusart1RxTail = 0;
  eusart1RxCount = 0;

  GPIO_InitTypeDef  GPIO_InitStructure = {0};
  USART_InitTypeDef USART_InitStructure = {0};
  NVIC_InitTypeDef  NVIC_InitStructure = {0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

  /* USART1 TX-->D.5   RX-->D.6 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

  USART_Init(USART1, &USART_InitStructure);
  // enable receive interrupt
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART1->CTLR1 |= USART_CTLR1_RXNEIE;

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);;

  USART_Cmd(USART1, ENABLE);
 }

bool EUSART1_is_tx_ready(void)
 {
  return (eusart1TxBufferRemaining ? true : false);
 }

bool EUSART1_is_rx_ready(void)
 {
  return (eusart1RxCount ? true : false);
 }

bool EUSART1_is_tx_done(void)
 {
  return USART1->STATR & USART_STATR_TXE;
 }

eusart1_status_t EUSART1_get_last_status(void)
 {
  return eusart1RxLastError;
 }

uint8_t EUSART1_Read(void)
 {
  uint8_t readValue  = 0;

  while(0 == eusart1RxCount)
   {
   }

  eusart1RxLastError = eusart1RxStatusBuffer[eusart1RxTail];

  readValue = eusart1RxBuffer[eusart1RxTail++];
  if(sizeof(eusart1RxBuffer) <= eusart1RxTail)
   {
    eusart1RxTail = 0;
   }
  //PIE3bits.RC1IE = 0;
  //USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
  USART1->CTLR1 &= (uint16_t)(~USART_CTLR1_RXNEIE);
  eusart1RxCount--;
  //PIE3bits.RC1IE = 1;
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART1->CTLR1 |= (uint16_t)(USART_CTLR1_RXNEIE);

  return readValue;
}

void EUSART1_Write(uint8_t txData)
 {
  while(0 == eusart1TxBufferRemaining)
   {
   }

  if((USART1->CTLR1 & USART_CTLR1_TXEIE) == 0)//(0 == PIE3bits.TX1IE)
   {
    //TX1REG = txData;
    USART1->DATAR = txData; //USART_SendData(USART1, txData);
   }
  else
   {
    //PIE3bits.TX1IE = 0;
    //USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    USART1->CTLR1 &= (uint16_t)(~USART_CTLR1_TXEIE);
    eusart1TxBuffer[eusart1TxHead++] = txData;
    if(sizeof(eusart1TxBuffer) <= eusart1TxHead)
     {
      eusart1TxHead = 0;
     }
    eusart1TxBufferRemaining--;
   }
  //PIE3bits.TX1IE = 1;
  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  USART1->CTLR1 |= (uint16_t)(USART_CTLR1_TXEIE);
 }


void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      USART1_IRQHandler
 *
 * @brief   This function handles USART1 global interrupt request.
 *
 * @return  none
 */
void USART1_IRQHandler(void)
{
 //if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  if (USART1->STATR & USART_STATR_RXNE)
  {
    // use this default receive interrupt handler code
    eusart1RxBuffer[eusart1RxHead++] = USART1->DATAR&0xff; //USART_ReceiveData(USART1);
    if(sizeof(eusart1RxBuffer) <= eusart1RxHead)
     {
      eusart1RxHead = 0;
     }
    eusart1RxCount++;
  }
 //if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
 if (USART1->STATR & USART_STATR_TXE)
  {
    // add your EUSART1 interrupt custom code
    if(sizeof(eusart1TxBuffer) > eusart1TxBufferRemaining)
     {
      USART1->DATAR = eusart1TxBuffer[eusart1TxTail++]; //USART_SendData(USART1, eusart1TxBuffer[eusart1TxTail++]);
      if(sizeof(eusart1TxBuffer) <= eusart1TxTail)
       {
	eusart1TxTail = 0;
       }
      eusart1TxBufferRemaining++;
     }
    else
     {
      //PIE3bits.TX1IE = 0;
      //USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
      USART1->CTLR1 &= (uint16_t)(~USART_CTLR1_TXEIE);
     }
  }
}


/**
  End of File
*/
