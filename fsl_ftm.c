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

// Standard C Included Files
#include <stdio.h>
#include <string.h>
#include "fsl_ftm_driver.h"
#include "fsl_ftm_hal.h"
#include "adc_hw_trigger.h"
#include "fsl_os_abstraction.h"

int g_iLVDTPWM_Flag = 0;                                //0:表示LVDT的PWM尚未开启, 1:表示已经开启
int g_iAO2_Flag = 0;                                    //0:表示AO2的PWM尚未开启, 1:表示已经开启

void init_ao2_pwm()
{
    SIM_SCGC6  |= SIM_SCGC6_FTM1_MASK;
    
    FTM1_SC = 0;
    FTM1_MODE |= FTM_MODE_WPDIS_MASK;
    FTM1_CNT = 0;
    FTM1_CNTIN = 0;
    FTM1_MOD = PWM1_PERIOD;
    FTM1_PWMLOAD = 0xFFFFFFFF;
    
    FTM1_SC &= ~FTM_SC_CLKS_MASK;
    FTM1_SC |= FTM_SC_CLKS(1);
    
    //Set Mode
    FTM1_MODE &= ~FTM_MODE_FTMEN_MASK;
    FTM1_QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;
    FTM1_SC &= ~FTM_SC_CPWMS_MASK;
    
    FTM1_C0SC |= (FTM_CnSC_MSB_MASK|FTM_CnSC_MSA_MASK);
    
    //FTM1_C0SC &= ~FTM_CnSC_ELSA_MASK;           //High true
    //FTM1_C0SC |= FTM_CnSC_ELSB_MASK;
    
    FTM1_C0SC |= FTM_CnSC_ELSA_MASK;              //low true
    FTM1_C0SC &= ~FTM_CnSC_ELSB_MASK;
    
    FTM1_C0V = PWM1_CV_DEFAULT;
    
    disable_ao2_pwm();
   
    
}

void init_lvdt_pwm()
{  
    
    SIM_SCGC6  |= SIM_SCGC6_FTM0_MASK;
  
    FTM0_SC = 0;
    FTM0_MODE |= FTM_MODE_WPDIS_MASK;
    FTM0_CNT = 0;
    FTM0_CNTIN = 0;
    FTM0_MOD = LVDT_PWM_PERIOD;
    FTM0_PWMLOAD = 0xFFFFFFFF;
    
    FTM0_SC &= ~FTM_SC_CLKS_MASK;
    FTM0_SC |= FTM_SC_CLKS(1);
    
    FTM0_SC &= ~FTM_SC_PS_MASK;
    FTM0_SC |= FTM_SC_PS(0); 
    
    //Set Mode
    FTM0_MODE &= ~FTM_MODE_FTMEN_MASK;
    FTM0_QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;
    FTM0_SC &= ~FTM_SC_CPWMS_MASK;
    
    FTM0_C3SC |= (FTM_CnSC_MSB_MASK|FTM_CnSC_MSA_MASK);
    
    //FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;           //High true
    //FTM0_C3SC |= FTM_CnSC_ELSB_MASK;
    
    FTM0_C3SC |= FTM_CnSC_ELSA_MASK;              //low true
    FTM0_C3SC &= ~FTM_CnSC_ELSB_MASK;
    
    FTM0_C3V = LVDT_PWM_CV_DEFAULT;
    
    disable_lvdt_pwm();
    
}

void clear_lvdt_tof(void)
{
    FTM0_SC |= FTM_SC_TOF_MASK;
}

void set_lvdt_cv(uint16_t value)
{
    FTM0_C3V = value;
}

void disable_lvdt_pwm()
{
    if(g_iLVDTPWM_Flag == 1)
    {
      FTM0_SC &= ~FTM_SC_CLKS(3);
      g_iLVDTPWM_Flag = 0;
    }
    
}

void enable_lvdt_pwm()
{
    if(g_iLVDTPWM_Flag == 0)
    {   
      FTM0_SC |= FTM_SC_CLKS(1);  
      g_iLVDTPWM_Flag = 1;
    }
    
}

void clear_ao2_tof(void)
{
    FTM1_SC |= FTM_SC_TOF_MASK;
}

void set_ao2_cv(uint16_t value)
{
    FTM1_C0V = value;
}

void disable_ao2_pwm()
{
    if(g_iAO2_Flag == 1)
    {
       FTM1_SC &= ~FTM_SC_CLKS(3);
       g_iAO2_Flag = 0;
    }
    
}

void enable_ao2_pwm()
{
    if(g_iAO2_Flag == 0)
    {        
        FTM1_SC |= FTM_SC_CLKS(1);
        g_iAO2_Flag = 1;
    }
    
}

