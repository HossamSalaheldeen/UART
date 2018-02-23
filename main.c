/**********************
 * Tuesday Feb 21st, 2018. 10:00 PM
 *
 * This code works.
 * FIFO is ENABLED.
 * No get_status function yet.
 * No interrupts yet.
 * GPIO Alternate function needs modification.
 * LCD Added
 *
**********************/



#include <stdint.h>
#include "M4MemMap.h"
#include "GPIO.h"
#include "GPIO_Cfg.h"
#include "UART.h"
#include "UART_Cfg.h"

#include "LCD.h"
#include "LCD_Cfg.h"

#define RX 1

int main(void)
    {
    uint8_t count = 0;


#if RX == 1

    UART_ChkType x;

    uint8_t* ptr_rx;
    uint8_t message_rx[4];
    ptr_rx = &message_rx[0];


    GPIO_Init();
    UART_Init();

    LCD_Init();
    LCD_Clear();



    do
    {
    x = UART_Rx_Init(ptr_rx,4,0);
    }while(x == UART_NOK);

    while(count<4)
    {

        UART_Rx(0);
        LCD_Clear();
        LCD_DispString(message_rx);
        count++;
    }


#else

	UART_ChkType x;

	uint8_t* ptr;
	uint8_t message[50] = "Are you fucking insane, you filthy bloody bastard?";
	ptr = &message[0];

	GPIO_Init();
	UART_Init();

	do
	{
	    x = UART_Tx_Init(ptr,50,0);
	}while(x == UART_NOK);

	while(1)
	{
	    UART_Tx(0);
	}
#endif

	return 0;

    }

