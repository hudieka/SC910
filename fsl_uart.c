#include "board.h"
#include "modbus.h"
#include "fsl_uart_driver.h"
#include "adc_hw_trigger.h"

extern unsigned char g_ucUART0_MB35charTime;

extern RxBuffer  s_RxUART0Buf;
extern TxBuffer  s_TxUART0Buf;

extern char g_cUART0Clear;

unsigned long ulRecvTimeIntv = 0;

void UART0_RX_TX_IRQHandler()
{
  uint8_t tmp = 0;
  
  if(((UART0_C2 & UART_C2_RIE_MASK) != 0)
     &&((UART0_S1 & UART_S1_RDRF_MASK)!= 0))
  {
    /* Get data and put into receive buffer */
    static unsigned long s_ulUART0RecvTimeLast = 0;
    ulRecvTimeIntv = get_waittime(); 
    ulRecvTimeIntv = ulRecvTimeIntv - s_ulUART0RecvTimeLast;
    if((ulRecvTimeIntv >= g_ucUART0_MB35charTime)&&(s_ulUART0RecvTimeLast != 0))
    { 
      if((UART0_S1 & (UART_S1_PF_MASK|UART_S1_FE_MASK|UART_S1_NF_MASK|UART_S1_OR_MASK))||(UART0_S2 & UART_S2_LBKDE_MASK))
      {
        tmp = UART0_D;
        UART0_S2 &= UART_S2_LBKDE_MASK;
      }
      g_cUART0Clear = false;
      s_RxUART0Buf.uCount = 0;
    }
    s_ulUART0RecvTimeLast = get_waittime();
    
    tmp = UART0_D;
    s_RxUART0Buf.byData[s_RxUART0Buf.uCount] = tmp;
    s_RxUART0Buf.uCount++;
    
    if(s_RxUART0Buf.uCount >= 6)
    {
      //s_RxUART0Buf.uCount = 0;
      if(ModbusReceive((unsigned char*)s_RxUART0Buf.byData, s_RxUART0Buf.uCount, 0))
      {
        if( !g_cUART0Clear )
        {
          // Receive over
          g_cUART0Clear = true;
          s_RxUART0Buf.uCount = 0;
          //break;
        }
        else
        {	
          // First Character was omitted
          unsigned short i;
          for (i = 0; i < s_RxUART0Buf.uCount; i++) 
          {
            s_RxUART0Buf.byData[i] = s_RxUART0Buf.byData[i + 1];
          }
          s_RxUART0Buf.uCount = s_RxUART0Buf.uCount - 1;
        }//end of if(!g_bClear)
      }
    }
    
  }
  else
  {
    tmp = UART0_D;
  }
}