/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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
#include <stdio.h>
// SDK Included Files
#include "fsl_debug_console.h"
#include "adc_hw_trigger.h"
#include "fsl_pdb_driver.h"
#include "fsl_clock_manager.h"

uint32_t g_ulmodValue = 0;

///////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////

extern SIM_Type * gSimBase[]; // driver state store

///////////////////////////////////////////////////////////////////////////////
// Code
///////////////////////////////////////////////////////////////////////////////

/*!
 * @Brief PDB0 IRQ Handler to handle the seq error if any
 */
void PDB0_IRQHandler(void)
{
    // clear the IF
    PDB_DRV_ClearTimerIntFlag(0);
    
    // clear the EIE
    if (1U == PDB_DRV_GetAdcPreTriggerSeqErrFlags(0, 0, 1U ))
    {
        PDB_DRV_ClearAdcPreTriggerSeqErrFlags(0, 0, 1U);
    }
    if (2U == PDB_DRV_GetAdcPreTriggerSeqErrFlags(0, 0, 2U))
    {
        PDB_DRV_ClearAdcPreTriggerSeqErrFlags(0, 0, 2U);
    }
}

/*!
 * @Brief enable the trigger source
 */
void init_trigger_pdb(uint32_t adcInstance)
{
    pdb_timer_config_t PdbTimerConfig;
  
    pdb_adc_pretrigger_config_t pdbAdcTriggerConfig;
    pdb_clk_prescaler_div_t divMode = kPdbClkPreDivBy1;
    
    //uint32_t modValue = PDB_MODDATA;
    uint32_t busClock, modValue;
    uint8_t preDivider = 1 << divMode;

    // get the bus clock freq which for PDB
    CLOCK_SYS_GetFreq(kBusClock, &busClock);

    // calculate for MOD value
    modValue = SINGAL_FRQ*NR_SAMPLES;
    modValue = busClock/preDivider/modValue*4;
    modValue = 4000;
    g_ulmodValue = modValue;

    // init the pdb user config structure for
    // enable SW trigger, enable continuous mode,
    // set the correct MOD and DLY
    // Enable the sequency error interrupt.
    
    // Initialize PDB driver
    PdbTimerConfig.loadValueMode = kPdbLoadValueImmediately;
    PdbTimerConfig.seqErrIntEnable = false;
    PdbTimerConfig.clkPreDiv = divMode;
    PdbTimerConfig.clkPreMultFactor = kPdbClkPreMultFactorAs1;
    //PdbTimerConfig.clkPreMultFactor = kPdbClkPreMultFactorAs20;
    PdbTimerConfig.triggerInput = kPdbSoftTrigger;
    PdbTimerConfig.continuousModeEnable = false;

    PdbTimerConfig.dmaEnable = false;
        //yangliang
    PdbTimerConfig.intEnable = false;
    PdbTimerConfig.seqErrIntEnable = false;
    //PdbTimerConfig.intEnable = false;
    //PdbTimerConfig.seqErrIntEnable = false;
    
    PDB_DRV_Init(0U, &PdbTimerConfig);
    

    // Configure the PDB channel for ADC_adcInstance
    
    pdbAdcTriggerConfig.adcPreTriggerIdx = 0U;
    pdbAdcTriggerConfig.preTriggerEnable = true;
    pdbAdcTriggerConfig.preTriggerOutputEnable = true;
    pdbAdcTriggerConfig.preTriggerBackToBackEnable = false;
    PDB_DRV_ConfigAdcPreTrigger(0U, adcInstance, &pdbAdcTriggerConfig);
#if 0 
    pdbAdcTriggerConfig.adcPreTriggerIdx = 1U;
    pdbAdcTriggerConfig.preTriggerEnable = true;
    pdbAdcTriggerConfig.preTriggerOutputEnable = true;
    pdbAdcTriggerConfig.preTriggerBackToBackEnable = false;
    PDB_DRV_ConfigAdcPreTrigger(0U, adcInstance, &pdbAdcTriggerConfig);
#endif  
    // the Pre-trigger A delay is set to 1/4 MOD,
    // the Pre-trigger B delay is set to 3/4 MOD,
    // so the trigger interval between A/B, B/A is same.
    PDB_DRV_SetTimerModulusValue(0U, modValue);
    PDB_DRV_SetAdcPreTriggerDelayValue(0U, adcInstance, 0U, 200);
    //PDB_DRV_SetAdcPreTriggerDelayValue(0U, adcInstance, 1U, modValue);
    PDB_DRV_LoadValuesCmd(0U);

    // Configure SIM for ADC hw trigger source PDB
    SIM_HAL_SetAdcAlternativeTriggerCmd(gSimBase[0], adcInstance, false);

    // Trigger the PDB, let it go
    // PDB_DRV_SoftTriggerCmd(0);
}

/*!
 * @Brief disable the trigger source
 */
void deinit_trigger_pdb(uint32_t adcInstance)
{
    PDB_DRV_Deinit(0);
}

void updata_pdb_register(uint32_t mod, uint32_t dly)
{

    PDB_DRV_SetTimerModulusValue(0U, mod);
    PDB_DRV_SetAdcPreTriggerDelayValue(0U, 0, 0U, dly);
    PDB_DRV_LoadValuesCmd(0U);
}
