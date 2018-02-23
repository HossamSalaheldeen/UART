#include "UART.h"
#include "UART_Cfg.h"
#include "M4MemMap.h"
#include <stdint.h>

typedef volatile uint32_t* const UART_RegAddType;

#define PORTS_NUMBER 8U

#define LOOPBACKMODE 0


/* Registers memory map */


#define UART0_BASE_ADDRESS 0x4000C000
#define UART1_BASE_ADDRESS 0x4000D000
#define UART2_BASE_ADDRESS 0x4000E000
#define UART3_BASE_ADDRESS 0x4000F000
#define UART4_BASE_ADDRESS 0x40010000
#define UART5_BASE_ADDRESS 0x40011000
#define UART6_BASE_ADDRESS 0x40012000
#define UART7_BASE_ADDRESS 0x40013000



static const uint32_t PortsBaseAddressLut[PORTS_NUMBER]
={
  UART0_BASE_ADDRESS,
  UART1_BASE_ADDRESS,
  UART2_BASE_ADDRESS,
  UART3_BASE_ADDRESS,
  UART4_BASE_ADDRESS,
  UART5_BASE_ADDRESS,
  UART6_BASE_ADDRESS,
  UART7_BASE_ADDRESS
};


#define UART_REG_ADDRESS(ID,REG_OFFSET) (PortsBaseAddressLut[ID] + REG_OFFSET)


/* Data Control */
#define UARTDR_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x000))


/* Clock Control */

#define UARTCC_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0xFC8))
#define UARTCTL_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x030))


/* DMA Control */

#define UARTDMACTL_REG(PORT_ID)         *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x048))


/* Interrupt Control */

#define UARTIFLS_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x034))
#define UARTIM_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x038))
#define UARTRIS_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x03C))
#define UARTMIS_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x040))
#define UARTICR_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x044))


/* Status Control */

#define UARTRSR_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x004))
#define UARTFR_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x018))
#define UARTLCRH_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x02C))
#define UARTILPR_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x020))
#define UART9BITADDR_REG(PORT_ID)       *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x0A4))
#define UART9BITAMASK_REG(PORT_ID)      *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x0A8))
#define UARTPP_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0xFC0))


/* Baud Rate Control */

#define UARTIBRD_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x024))
#define UARTFBRD_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x028))


/* Defining some constants */

#define SYSCLK          16000000.0

#define PEN_BIT_NO      1U
#define STP2_BIT_NO     3U
#define FEN_BIT_NO      4U
#define WLEN_BIT_NO     5U

#define UARTEN_BIT_NO   0U
#define LBE_BIT_NO      7U
#define TXE_BIT_NO      8U
#define RXE_BIT_NO      9U

uint8_t DONE = 1;


/* Defining some variables */
static uint8_t UART_Init_Done[UART_GROUPS_NUMBER] = {0};
static uint8_t UART_Tx_Init_Done[UART_GROUPS_NUMBER] = {0};
static uint8_t UART_Rx_Init_Done[UART_GROUPS_NUMBER] = {0};


typedef struct
{

    uint8_t TxChVal;
    uint8_t RxChVal;

} GPIO_AlternFunc;


static const GPIO_AlternFunc UART_AlternFunc[PORTS_NUMBER]
={

 {0x01,0x01},
 {0x02,0x02},
 {0x01,0x01},
 {0x01,0x01},
 {0x01,0x01},
 {0x01,0x01},
 {0x01,0x01},
 {0x01,0x01}

};


static uint8_t UART_TxLength[UART_GROUPS_NUMBER];
static uint8_t UART_TxCount[UART_GROUPS_NUMBER];
static uint8_t* UART_TxBuffPtr[UART_GROUPS_NUMBER];

static uint8_t UART_RxLength[UART_GROUPS_NUMBER];
static uint8_t UART_RxCount[UART_GROUPS_NUMBER];
static uint8_t* UART_RxBuffPtr[UART_GROUPS_NUMBER];


/* Initialization Function */
UART_ChkType UART_Init (void)
{

    UART_ChkType RetVar;
    uint8_t LoopIndex;
    uint8_t TempVar1;
    uint16_t TempVar2;
    const UART_ConfigType* CfgPtr;
    double BaudRateDiv;

    for (LoopIndex = 0; LoopIndex < UART_GROUPS_NUMBER; LoopIndex++)
    {
        CfgPtr = &UART_ConfigParam[LoopIndex];
        if(((CfgPtr->StopBits)   <=  TwoStopBit)&&
           ((CfgPtr->WordLen)    <=  Data_8)&&
           ((CfgPtr->TxFIFOSize) <=   16)&&
           ((CfgPtr->RxFIFOSize) <=   16))
        {
            /* Configure GPIO port's alternate function as UART*/

            /* Enable UART port clock */
            RCGCUART_REG |= 1 << (CfgPtr->UARTPortID);

            /* Disable the UART port*/
            UARTCTL_REG(CfgPtr->UARTPortID) = 0x00;

            /* BaudRate value */
            BaudRateDiv = SYSCLK /(16* (CfgPtr->BaudRate));

            // The integer part
            UARTIBRD_REG(CfgPtr->UARTPortID) = (uint32_t) BaudRateDiv;

            // The floating part
            BaudRateDiv -= (uint32_t) BaudRateDiv;
            UARTFBRD_REG(CfgPtr->UARTPortID) = (uint32_t)((BaudRateDiv * 64) + 0.5);

            /* Selecting the system clock */
            UARTCC_REG(CfgPtr->UARTPortID) = 0x0;

            /* Configuring the UART Line Control register */
            // Initializing the temporary variable
            TempVar1 = 0x00;
            // Word Length = 8 bits (11 in binary)
            TempVar1 |= (CfgPtr->WordLen) << WLEN_BIT_NO;
            // One stop bit
            TempVar1 |= (CfgPtr->StopBits) << STP2_BIT_NO;
            // Parity
            TempVar1 |= (CfgPtr->Parity) << PEN_BIT_NO;
            // FIFO
            if(((CfgPtr->TxFIFOSize) > 0) ||
               ((CfgPtr->RxFIFOSize) > 0))
            {
            TempVar1 |= (CfgPtr->FIFOEN) << FEN_BIT_NO;
            }
            // Configuring the register
            UARTLCRH_REG(CfgPtr->UARTPortID) = TempVar1;

            /* Enabling the UART port */
            TempVar2 = 0x00;
            TempVar2 |= 1 << UARTEN_BIT_NO;
            TempVar2 |= 1 << TXE_BIT_NO;
            TempVar2 |= 1 << RXE_BIT_NO;

#if LOOPBACKMODE
            TempVar2 |= 1 << LBE_BIT_NO; // Loopback mode
#endif
            UARTCTL_REG(CfgPtr->UARTPortID) = TempVar2;

            RetVar = UART_OK;

            UART_Init_Done[LoopIndex] = DONE;
        }
        else
        {
            RetVar = UART_NOK;
        }

    }
    return RetVar;

}



UART_ChkType UART_Tx_Init(const uint8_t* TxBuffPtr, uint8_t TxLen, uint8_t groupID)
{
    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;

    if(groupID < UART_GROUPS_NUMBER)
    {
        CfgPtr = &UART_ConfigParam[groupID];

        if (UART_Init_Done[groupID] == DONE)
        {
            UART_TxLength[groupID] = TxLen;
            UART_TxBuffPtr[groupID] = TxBuffPtr;
            UART_TxCount[groupID] = 0;

            if((UARTFR_REG(CfgPtr->UARTPortID) & 0x20) == 0)
            {
                UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                UART_TxCount[groupID]++;
                UART_Tx_Init_Done[groupID] = DONE;
                RetVar = UART_OK;
            }
            else
            {
                RetVar = UART_NOK;
            }
        }
    }

    else
    {
        RetVar = UART_NOK;

    }

    return RetVar;
}

UART_ChkType UART_Tx (uint8_t groupID)
{

    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;

    if(groupID < UART_GROUPS_NUMBER)
    {
        CfgPtr = &UART_ConfigParam[groupID];

        if(UART_Tx_Init_Done[groupID] == DONE)
        {

            if(UART_TxCount[groupID] < UART_TxLength[groupID])
            {
                if((UARTFR_REG(CfgPtr->UARTPortID) & 0x20) == 0)
                {
                UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                UART_TxCount[groupID]++;
                RetVar = UART_OK;
                }
            }


        }
        else
            {
                RetVar = UART_NOK;
            }
    }
    return RetVar;

}



UART_ChkType UART_Rx_Init (const uint8_t* RxBuffPtr, uint8_t RxLen, uint8_t groupID)
{
    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;

    if (groupID < UART_GROUPS_NUMBER)
        {
        CfgPtr = &UART_ConfigParam[groupID];

        if(UART_Init_Done[groupID] == DONE)
        {
            UART_RxLength[groupID] = RxLen;
            UART_RxCount[groupID] = 0;
            UART_RxBuffPtr[groupID] = RxBuffPtr;


            if((UARTFR_REG(CfgPtr->UARTPortID) & 0x10) == 0)
            {
                *(UART_RxBuffPtr[groupID] + UART_RxCount[groupID]) = UARTDR_REG(CfgPtr->UARTPortID);
                UART_RxCount[groupID]++;
                UART_Rx_Init_Done [groupID] = DONE;
                RetVar = UART_OK;
            }
            else

            {
                RetVar = UART_NOK;
            }

        }

        }
    else
    {
        RetVar=UART_NOK;
    }

    return RetVar;
}


UART_ChkType UART_Rx (uint8_t groupID)
{

    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;

    if(groupID < UART_GROUPS_NUMBER)
    {
        CfgPtr = &UART_ConfigParam[groupID];

        if(UART_Rx_Init_Done[groupID] == DONE)
        {
            if(UART_RxCount[groupID] < UART_RxLength[groupID])
            {
                if((UARTFR_REG(CfgPtr->UARTPortID) & 0x10) == 0)
                {
                    *(UART_RxBuffPtr[groupID] + UART_RxCount[groupID]) = UARTDR_REG(CfgPtr->UARTPortID);
                    UART_RxCount[groupID]++;
                    RetVar = UART_OK;
                }
            }



        }
        else
        {
            RetVar = UART_NOK;
        }

    }
    return RetVar;


}
