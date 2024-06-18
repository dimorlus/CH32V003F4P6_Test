
#ifndef EUSART1_H
#define EUSART1_H

/**
  Section: Included Files
*/

#include <ch32v00x.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif


/**
  Section: Macro Declarations
*/

#define EUSART1_DataReady  (EUSART1_is_rx_ready())

/**
  Section: Data Type Definitions
*/

typedef union {
    struct {
        unsigned perr : 1;
        unsigned ferr : 1;
        unsigned oerr : 1;
        unsigned reserved : 5;
    };
    uint8_t status;
}eusart1_status_t;

/**
 Section: Global variables
 */
extern volatile uint8_t eusart1TxBufferRemaining;
extern volatile uint8_t eusart1RxCount;

/**
  Section: EUSART1 APIs
*/

/**
  @Summary
    Initialization routine that takes inputs from the EUSART1 GUI.

  @Description
    This routine initializes the EUSART1 driver.
    This routine must be called before any other EUSART1 routine is called.

  @Preconditions
    None

  @Param
    None

  @Returns
    None

  @Comment
    
*/
void EUSART1_Initialize(void);

/**
  @Summary
    Checks if the EUSART1 transmitter is ready to transmit data

  @Description
    This routine checks if EUSART1 transmitter is ready 
    to accept and transmit data byte

  @Preconditions
    EUSART1_Initialize() function should have been called
    before calling this function.
    EUSART1 transmitter should be enabled before calling 
    this function

  @Param
    None

  @Returns
    Status of EUSART1 transmitter
    TRUE: EUSART1 transmitter is ready
    FALSE: EUSART1 transmitter is not ready
    
  @Example
    <code>
    void main(void)
    {
        volatile uint8_t rxData;
        
        // Initialize the device
        SYSTEM_Initialize();
        
        while(1)
        {
            // Logic to echo received data
            if(EUSART1_is_rx_ready())
            {
                rxData = UART1_Read();
                if(EUSART1_is_tx_ready())
                {
                    EUSART1Write(rxData);
                }
            }
        }
    }
    </code>
*/
bool EUSART1_is_tx_ready(void);

/**
  @Summary
    Checks if the EUSART1 receiver ready for reading

  @Description
    This routine checks if EUSART1 receiver has received data 
    and ready to be read

  @Preconditions
    EUSART1_Initialize() function should be called
    before calling this function
    EUSART1 receiver should be enabled before calling this 
    function

  @Param
    None

  @Returns
    Status of EUSART1 receiver
    TRUE: EUSART1 receiver is ready for reading
    FALSE: EUSART1 receiver is not ready for reading
    
  @Example
    <code>
    void main(void)
    {
        volatile uint8_t rxData;
        
        // Initialize the device
        SYSTEM_Initialize();
        
        while(1)
        {
            // Logic to echo received data
            if(EUSART1_is_rx_ready())
            {
                rxData = UART1_Read();
                if(EUSART1_is_tx_ready())
                {
                    EUSART1_Write(rxData);
                }
            }
        }
    }
    </code>
*/
bool EUSART1_is_rx_ready(void);

/**
  @Summary
    Checks if EUSART1 data is transmitted

  @Description
    This function return the status of transmit shift register

  @Preconditions
    EUSART1_Initialize() function should be called
    before calling this function
    EUSART1 transmitter should be enabled and EUSART1_Write
    should be called before calling this function

  @Param
    None

  @Returns
    Status of EUSART1 receiver
    TRUE: Data completely shifted out if the USART shift register
    FALSE: Data is not completely shifted out of the shift register
    
  @Example
    <code>
    void main(void)
    {
        volatile uint8_t rxData;
        
        // Initialize the device
        SYSTEM_Initialize();
        
        while(1)
        {
           if(EUSART1_is_tx_ready())
            {
		LED_0_SetHigh();
                EUSART1_Write(rxData);
            }
	   if(EUSART1_is_tx_done()
            {
                LED_0_SetLow();
            }
        }
    }
    </code>
*/
bool EUSART1_is_tx_done(void);

/**
  @Summary
    Gets the error status of the last read byte.

  @Description
    This routine gets the error status of the last read byte.

  @Preconditions
    EUSART1_Initialize() function should have been called
    before calling this function. The returned value is only
    updated after a read is called.

  @Param
    None

  @Returns
    the status of the last read byte

  @Example
	<code>
    void main(void)
    {
        volatile uint8_t rxData;
        volatile eusart1_status_t rxStatus;
        
        // Initialize the device
        SYSTEM_Initialize();
        
        // Enable the Global Interrupts
        INTERRUPT_GlobalInterruptEnable();
        
        while(1)
        {
            // Logic to echo received data
            if(EUSART1_is_rx_ready())
            {
                rxData = EUSART1_Read();
                rxStatus = EUSART1_get_last_status();
                if(rxStatus.ferr){
                    LED_0_SetHigh();
                }
            }
        }
    }
    </code>
 */
eusart1_status_t EUSART1_get_last_status(void);

/**
  @Summary
    Read a byte of data from the EUSART1.

  @Description
    This routine reads a byte of data from the EUSART1.

  @Preconditions
    EUSART1_Initialize() function should have been called
    before calling this function. The transfer status should be checked to see
    if the receiver is not empty before calling this function.

  @Param
    None

  @Returns
    A data byte received by the driver.
*/
uint8_t EUSART1_Read(void);

 /**
  @Summary
    Writes a byte of data to the EUSART1.

  @Description
    This routine writes a byte of data to the EUSART1.

  @Preconditions
    EUSART1_Initialize() function should have been called
    before calling this function. The transfer status should be checked to see
    if transmitter is not busy before calling this function.

  @Param
    txData  - Data byte to write to the EUSART1

  @Returns
    None
*/
void EUSART1_Write(uint8_t txData);



#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif  // EUSART1_H
/**
 End of File
*/
