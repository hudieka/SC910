/*
* Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "adc_hw_trigger.h"

#include "fsl_pit_driver.h"
#include "fsl_ftm_driver.h"
#include "fsl_pdb_driver.h"
#include "fsl_dac_driver.h"
#include "fsl_uart_driver.h"
#include "fsl_uart_edma_driver.h"
#include "modbus.h"

#include "app_sc910.h"

#define PIT2_TIMER                      //PIT2复用为LVDT激励以及Modbus应答，LVDT输出默认20us,所以4个周期产生


SIM_Type * gSimBase[] = SIM_BASE_PTRS; // SIM base address


static unsigned long g_ultimer1ms = 0;
static unsigned long g_ultimerXus = 0;
unsigned char g_ucUART0_MB35charTime = 4*100/LVDT_PWM_REFRESH;  //PIT2 100us 时，此值为4


extern unsigned long g_ucUART0ModbusTime;
extern unsigned long g_ucUART1ModbusTime;
extern unsigned long g_ucUART2ModbusTime;
extern unsigned long g_ucUART3ModbusTime;

extern unsigned char g_ucUART0ModbusFlag;
extern unsigned char g_ucUART1ModbusFlag;
extern unsigned char g_ucUART2ModbusFlag;
extern unsigned char g_ucUART3ModbusFlag;

extern unsigned char g_ucUART0_MB35charTime;
extern unsigned char g_ucUART1_MB35charTime;
extern unsigned char g_ucUART2_MB35charTime;
extern unsigned char g_ucUART3_MB35charTime;

extern unsigned char g_ucSend0Len;
extern unsigned char g_ucSend1Len;
extern unsigned char g_ucSend2Len;
extern unsigned char g_ucSend3Len;

extern TxBuffer  s_TxUART0Buf;

extern uint8_t g_ucAI1Count;
extern uint8_t g_ucAI2Count;
extern uint8_t g_ucAI3Count;

//SC910参数
extern SC910Parameter g_sc910param;  
extern uint16_t g_uiPWM_Data[SINGAL_NUM];
extern uint16_t g_uiAI3[NR_SAMPLES];
extern uint32_t g_ulmodValue;

uint16_t g_uilvdtbuf[NR_SAMPLES];                       //一个LVDT周期存储的数据BUF
unsigned char g_uilvdtflag = 0;                       //一个LVDT周期数据数据采集完成，0表示未完成，1表示完成


LvdtSeek g_lvdtseek;



/*******************************************************************************
* Variables
******************************************************************************/

/*******************************************************************************
* Code
******************************************************************************/


/*!
* @Brief enable the trigger source of PIT0, chn0
*/
void init_trigger_pit(uint32_t adcInstance)
{
  
  pit_user_config_t pitUserInit;
  
  PIT_DRV_Init(0, true);
 
#if 1
  // Initialize PIT0
  pitUserInit.isInterruptEnabled = false;
  pitUserInit.periodUs = 1000; 
  // Init PIT0 module and enable run in debug
  
  // Initialize PIT0 timer 0
  PIT_DRV_InitChannel(0, 0, &pitUserInit); 
  PIT_DRV_StartTimer(0, 0);    

#endif 

  // Initialize PIT1  ADC trigger
  pitUserInit.isInterruptEnabled = false;
  //yangliang
  //pitUserInit.periodUs = 6;
  pitUserInit.periodUs = 10;
  PIT_DRV_InitChannel(0, 1, &pitUserInit);  
  PIT_DRV_StartTimer(0, 1);
  
  ADC0_SC1A &= ~0x1F;
  //ADC0_SC1A |= 0x09;
  ADC0_SC1A |= 0x04;
  // Configure SIM for ADC hw trigger source selection
  SIM_HAL_SetAdcAlternativeTriggerCmd(gSimBase[0], adcInstance, true);
  SIM_HAL_SetAdcPreTriggerMode(gSimBase[0], adcInstance, kSimAdcPretrgselA);
  SIM_HAL_SetAdcTriggerMode(gSimBase[0], adcInstance, kSimAdcTrgSelPit1);

  
  // Initialize PIT2
  pitUserInit.isInterruptEnabled = true;
  //pitUserInit.periodUs = LVDT_PWM_REFRESH;
  pitUserInit.periodUs = 20;
  PIT_DRV_InitChannel(0, 2, &pitUserInit);  
  PIT_DRV_StartTimer(0, 2);
  NVIC_SetPriority(PIT2_IRQn, 0);
  
  
  
}

/*!
* @Brief disable the trigger source
*/
void deinit_trigger_pit(uint32_t adcInstance)
{
  PIT_DRV_StopTimer(0, adcInstance);
}



void PIT0_IRQHandler(void)
{

  
  /* Clear interrupt flag.*/
  PIT_HAL_ClearIntFlag(g_pitBase[0], 0U); 
  
  g_ultimer1ms++;
  
  //sc910_output();
  
}



void PIT1_IRQHandler(void)
{
  /* Clear interrupt flag.*/
  PIT_HAL_ClearIntFlag(g_pitBase[0], 1U);
  
}

void PIT2_IRQHandler(void)
{
  static uint16_t i = 0;
  /* Clear interrupt flag.*/
  PIT_HAL_ClearIntFlag(g_pitBase[0], 2U);
  
  g_ultimerXus++;
  
#if 1
  //only for test
  // g_sc910param.AO_CHSEL |= HIGH_BIT3;
  
  //LVDT selected
  if(g_sc910param.AO_CHSEL & HIGH_BIT3)
  {
      set_lvdt_cv(g_uiPWM_Data[i]);
      //set_lvdt_cv(50);
      //DAC_DRV_Output(BOARD_DAC_DEMO_DAC_INSTANCE, g_uiPWM_Data[i]);
#if 0
      if(i == 0)
      {
          PIT_DRV_StopTimer(0, 1);
          PIT_DRV_StartTimer(0, 1);
          g_ucAI1Count = 0;
          g_ucAI2Count = 0;
          g_ucAI3Count = 0;
      }
#endif
      
      i++;
      if(i >= (SINGAL_NUM))
      {
          
          memcpy( g_uilvdtbuf, g_uiAI3, NR_SAMPLES*2);
          g_uilvdtflag = true;
          i = 0;
          
      }
  }
#endif
  
#if 1
  
  if(g_ucUART0ModbusFlag == MODBUS_SEND)
  {
    if((g_ultimerXus - g_ucUART0ModbusTime) > g_ucUART0_MB35charTime)
    {
      //send_uart_dma0(g_ucSend0Len);
      g_ucSend0Len+=2;          //DMA结束时，还有2个字节在缓冲区里，这里故意多发两个字节
      RS485_EN = 1;
      UART_DRV_EdmaSendData(BOARD_DEBUG_UART_INSTANCE, s_TxUART0Buf.byData, g_ucSend0Len);
      
      g_ucSend0Len = 0;
      g_ucUART0ModbusFlag = MODBUS_CLOSE;
    }  
  }
#endif
  
}



unsigned long get_1ms()
{
  return g_ultimer1ms;
}

unsigned long get_waittime()
{
  return g_ultimerXus;
}






/*! @} */

/*******************************************************************************
* EOF
******************************************************************************/

