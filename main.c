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

///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////

// Standard C Included Files
#include <string.h>
#include <stdio.h>
// SDK Included Files

#include "adc_hw_trigger.h"
#include "fsl_adc16_driver.h"
#include "math.h"
#include "arm_math.h"
#include "modbus.h"
#include "app_sc910.h"


///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Prototypes
///////////////////////////////////////////////////////////////////////////////



#ifdef USE_DAC_OUT_AS_SOURCE
extern void dac_gen_wave(void);
extern void dac_stop_wave(void);
#endif

extern void fir_cal();

///////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////

static volatile bool gAdcDone = false; // sync object for adc convert result
static volatile uint8_t gCurChan = 0;

extern uint16_t g_uiAI3[NR_SAMPLES];
extern uint16_t g_uiAI1[NR_SAMPLES];
extern uint16_t g_uiAI2[NR_SAMPLES];

uint16_t g_uiPWM_Data[SINGAL_NUM] = {0};

const uint8_t buffStart[]   = "\n\r++++++++++++++++ UART-DMA Send/Receive Non Blocking Example +++++++++++++++++\n\r";

///////////////////////////////////////////////////////////////////////////////
// Code
///////////////////////////////////////////////////////////////////////////////

/*!
 * @brief Main demo function
 */
int main(void)
{
    unsigned long t1 = 0;
    int i = 0;
    uint32_t byteCountBuff = 0;
    
    for(i = 0; i < SINGAL_NUM;i++)
    {
    	//SIN
    	g_uiPWM_Data[i] = (uint16_t)(HALF_LVDT_PWM_PERIOD*sin(2*PI/SINGAL_NUM * i + PI) + HALF_LVDT_PWM_PERIOD);
        //g_uiPWM_Data[i] = (uint16_t)(2048*sin(2*PI/SINGAL_NUM * i + PI) + 2048);
    	//TRI
        //g_uiPWM_Data[i] = i + (PWM0_PERIOD/PWM0_POINT_NUM);
    }
    
   sc910_load_param();

    // init the hardware board
    hardware_init(); 
    
    Kinetis_FlashInit();
	
    while(1)
    {
        sc910_input();
        sc910_iec();
        //sc910_output();
        CstModbusSendAck();
        sc910_comm_process();
        sc910_save_param();
        
        //fir_cal();
#if 0   //For Uart TEST
        if((get_1ms() - t1) > 1000)
        {
            RS485_EN = 1;
            t1 = get_1ms();
            //UART0_D = i;
            byteCountBuff = sizeof(buffStart);
            UART_DRV_EdmaSendData(BOARD_DEBUG_UART_INSTANCE, buffStart, byteCountBuff);
            i++;
            //RS485_EN = 0;
        }
#endif
        
    }

}
