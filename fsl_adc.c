/*
 * Copyright (c) 2013 -2014, Freescale Semiconductor, Inc.
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
#include <stdint.h>
#include <stdbool.h>
// SDK Included Files
#include "fsl_adc16_driver.h"
#include "adc_hw_trigger.h"

///////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////


uint16_t g_uiAI1[NR_SAMPLES] = {0};
uint16_t g_uiAI2[NR_SAMPLES] = {0};
uint16_t g_uiAI3[NR_SAMPLES] = {0};

uint8_t g_ucAI1Count = 0;
uint8_t g_ucAI2Count = 0;
uint8_t g_ucAI3Count = 0;

uint16_t g_uiAI1_Current = 0;
uint16_t g_uiAI2_Current = 0;
uint16_t g_uiAI3_Current = 0;

// Define array to keep run-time callback set by application.
void (* volatile g_AdcTestCallback[ADC_INSTANCE_COUNT][ADC_SC1_COUNT])(void);
volatile uint16_t g_AdcValueInt[ADC_INSTANCE_COUNT][ADC_SC1_COUNT];



///////////////////////////////////////////////////////////////////////////////
// Code
///////////////////////////////////////////////////////////////////////////////

/*!
 * @brief ADC channel0 callback for fetching sample data.
 */
static void adc_chn0_isr_callback(void)
{
    uint16_t temp = 0;
    
    //ADC0_SE4b
    if((ADC0_SC1A & 0x1F) == 0x04)
    {      
        g_uiAI3_Current = ADC16_DRV_GetConvValueRAW(ADC_INST, 0);
        
        g_uiAI3[g_ucAI3Count++] = g_uiAI3_Current;
        
        temp = ADC0_SC1A;
        temp &= ~0x1F;
        temp|= 0x09;       
        ADC0_SC1A = temp;  
        if(g_ucAI3Count == NR_SAMPLES)
        {
            g_ucAI3Count = 0;
        }        

       
    }
    else if((ADC0_SC1A & 0x1F) == 0x09)
    {
        g_uiAI1_Current = ADC16_DRV_GetConvValueRAW(ADC_INST, 0);
        g_uiAI1[g_ucAI1Count++] = g_uiAI1_Current;
        
        temp = ADC0_SC1A;
        temp &= ~0x1F;
        //temp|= 0x0F;   
        temp|= 0x04; 
        ADC0_SC1A = temp;   
        
        if(g_ucAI1Count == NR_SAMPLES)
        {
            g_ucAI1Count = 0;
        }
    }    
    //ADC0_SE15
    else if((ADC0_SC1A & 0x1F) == 0x0F)
    {
        g_uiAI2_Current = ADC16_DRV_GetConvValueRAW(ADC_INST, 0);
        g_uiAI2[g_ucAI2Count++] = g_uiAI2_Current;
        
        temp = ADC0_SC1A;
        temp &= ~0x1F;
        temp|= 0x04;       
        ADC0_SC1A = temp;  
        
        if(g_ucAI2Count == NR_SAMPLES)
        {
            g_ucAI2Count = 0;
        }
    }    

    //PC3_T = 1;   
    //PC3 = 0;
    
}

#if 0
/*!
 * @brief ADC channel1 callback for fetching sample data.
 */
static void adc_chn1_isr_callback(void)
{
  
    static uint8_t j = 0, k = 0;  
    uint32_t temp = 0;   

    //ADC0_SE9
    if((ADC0_SC1B & 0x1F) == 0x9)
    {
        g_uiAI1[j++] = ADC16_DRV_GetConvValueRAW(ADC_INST, 1);
        
        temp = ADC0_SC1B;
        temp &= ~0x1F;
        temp|= 0x0F;       
        ADC0_SC1B = temp;   
        
        if(j == NR_SAMPLES)
        {
            j = 0;
        }
    }    
    //ADC0_SE15
    else if((ADC0_SC1B & 0x1F) == 0xF)
    {
        g_uiAI2[k++] = ADC16_DRV_GetConvValueRAW(ADC_INST, 1);
        
        temp = ADC0_SC1B;
        temp &= ~0x1F;
        temp|= 0x04;       
        ADC0_SC1B = temp;  
        
        if(k == NR_SAMPLES)
        {
            k = 0;
        }
    }    

}
#endif

/*!
 * @brief Initialize the ADCx for HW trigger.
 *
 * @param instance The ADC instance number
 */
int32_t init_adc(uint32_t instance)
{
#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    adc16_calibration_param_t adcCalibraitionParam;
#endif
    adc16_converter_config_t adcUserConfig;
    adc16_chn_config_t adcChnConfig;
    adc16_hw_average_config_t adchwaverageConfig;

    //ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);    
    /* Special configuration for highest accuracy. */
    adcUserConfig.lowPowerEnable = false;
    adcUserConfig.clkDividerMode = kAdc16ClkDividerOf1;
    adcUserConfig.longSampleTimeEnable = false;
    //yanglliang
    adcUserConfig.resolution = kAdc16ResolutionBitOfSingleEndAs16;
    //adcUserConfig.resolution = kAdc16ResolutionBitOfSingleEndAs12;
    adcUserConfig.clkSrc = kAdc16ClkSrcOfBusClk;   //kAdc16ClkSrcOfAsynClk;
    adcUserConfig.asyncClkEnable = true;
    adcUserConfig.highSpeedEnable = true;
    adcUserConfig.longSampleCycleMode = kAdc16LongSampleCycleOf24;
    adcUserConfig.hwTriggerEnable = false;
    adcUserConfig.refVoltSrc = kAdc16RefVoltSrcOfVref;
    adcUserConfig.continuousConvEnable = false;
    
#if (  defined(FRDM_KL43Z)   /* CPU_MKL43Z256VLH4 */ \
    || defined(TWR_KL43Z48M) /* CPU_MKL43Z256VLH4 */ \
    || defined(FRDM_KL27Z)   /* CPU_MKL27Z64VLH4  */ \
    )
    adcUserConfig.refVoltSrc = kAdc16RefVoltSrcOfValt;
#endif
    ADC16_DRV_Init(instance, &adcUserConfig);

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    // Auto calibraion.
    ADC16_DRV_GetAutoCalibrationParam(instance, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(instance, &adcCalibraitionParam);
#endif

    // Initialization ADC for
    // 12bit resolution, interrrupt mode, hw trigger enabled.
    // normal convert speed, VREFH/L as reference,
    // disable continuouse convert mode.
    //ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
    adcUserConfig.hwTriggerEnable = true;
    adcUserConfig.continuousConvEnable = false;
#if (  defined(FRDM_KL43Z)   /* CPU_MKL43Z256VLH4 */ \
    || defined(TWR_KL43Z48M) /* CPU_MKL43Z256VLH4 */ \
    || defined(FRDM_KL27Z)   /* CPU_MKL27Z64VLH4  */ \
    )
    adcUserConfig.refVoltSrc = kAdc16RefVoltSrcOfValt;
#endif
    
    
    ADC16_DRV_Init(instance, &adcUserConfig);

    // Install Callback function into ISR
    ADC_TEST_InstallCallback(instance, 0U, adc_chn0_isr_callback);
    //ADC_TEST_InstallCallback(instance, 1U, adc_chn1_isr_callback);

    adcChnConfig.chnIdx = (adc16_chn_t)ADC_AI1;
#if FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adcChnConfig.diffConvEnable = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
    adcChnConfig.convCompletedIntEnable = true;
    
    ADC0_CFG2 |= ADC_CFG2_MUXSEL_MASK;

    // Configure channel0
    ADC16_DRV_ConfigConvChn(instance, 0U, &adcChnConfig);
    
    adchwaverageConfig.hwAverageEnable=false;
    //yangliang for hardware average
    adchwaverageConfig.hwAverageCountMode=kAdc16HwAverageCountOf4;
    //adchwaverageConfig.hwAverageCountMode=kAdc16HwAverageCountOf16;
    ADC16_DRV_ConfigHwAverage(0, & adchwaverageConfig);                  //Set Hardware Average numbers

    // Configure channel1, which is used in PDB trigger case
    //ADC16_DRV_ConfigConvChn(instance, 1U, &adcChnConfig);

    return 0;
}


/* User-defined function to install callback. */
void ADC_TEST_InstallCallback(uint32_t instance, uint32_t chnGroup, void (*callbackFunc)(void) )
{
    g_AdcTestCallback[instance][chnGroup] = callbackFunc;
}

/* User-defined function to read conversion value in ADC ISR. */
uint16_t ADC_TEST_GetConvValueRAWInt(uint32_t instance, uint32_t chnGroup)
{
    return g_AdcValueInt[instance][chnGroup];
}

/* User-defined ADC ISR. */
static void ADC_TEST_IRQHandler(uint32_t instance)
{
    uint32_t chnGroup;
    for (chnGroup = 0U; chnGroup < ADC_SC1_COUNT; chnGroup++)
    {
        if (   ADC16_DRV_GetChnFlag(instance, chnGroup, kAdcChnConvCompleteFlag) )
        {
            g_AdcValueInt[instance][chnGroup] = ADC16_DRV_GetConvValueRAW(instance, chnGroup);
            if ( g_AdcTestCallback[instance][chnGroup] )
            {
                (void)(*(g_AdcTestCallback[instance][chnGroup]))();
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// IRQ Handlers
///////////////////////////////////////////////////////////////////////////////

/* ADC IRQ handler that would cover the same name's APIs in startup code */
void ADC0_IRQHandler(void)
{
    // Add user-defined ISR for ADC0
    ADC_TEST_IRQHandler(0U);
}

#if (HW_ADC_INSTANCE_COUNT > 1U)
void ADC1_IRQHandler(void)
{
    // Add user-defined ISR for ADC1
    ADC_TEST_IRQHandler(1U);
}
#endif

#if (HW_ADC_INSTANCE_COUNT > 2U)
void ADC2_IRQHandler(void)
{
    // Add user-defined ISR for ADC2
    ADC_TEST_IRQHandler(2U);
}
#endif

#if (HW_ADC_INSTANCE_COUNT > 3U)
void ADC3_IRQHandler(void)
{
    // Add user-defined ISR for ADC3
    ADC_TEST_IRQHandler(3U);
}
#endif
