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

#ifndef _ADC_HW_TRIGGER_H
#define _ADC_HW_TRIGGER_H

#include "fsl_sim_hal.h"
#include "board.h"
#include "flash_kinetis.h"
#include "fsl_dac_driver.h"

//LVDT_PWM
#define SINGAL_FRQ                      1000               //Hz
#define SINGAL_NUM                      50
#define LVDT_PWM_PERIOD                 100
#define HALF_LVDT_PWM_PERIOD            (LVDT_PWM_PERIOD/2)
#define LVDT_PWM_CV_DEFAULT             80
#define LVDT_PWM_REFRESH                (1000000/(SINGAL_FRQ*SINGAL_NUM*2))

//LVDT MODE
#define  SEEK_PHASE_DNOE                0
#define  SEEK_PHASE_MODE                1
#define  SKIP_CYCNUM			2


//For PWM1  AO2
#define PWM1_PERIOD                     0xFF                                   //PWM_PERIOD = period * 80MHz;
#define PWM1_CV_DEFAULT                 PWM1_PERIOD/2

//For ADC
#define ADC_INST                        0U  /*!< ADC instance */
#define NR_SAMPLES                      50 //26 //26 /*!< number of samples in one period */
//#define NR_SAMPLES                      100 //26 /*!< number of samples in one period */

#define ADC_AI1    kAdc16Chn4           //(kAdc16Chn23) /* default input signal channel */


//DO
#define DO1 BITBAND_REG(PTA->PDOR, 19)
#define DO2 BITBAND_REG(PTD->PDOR, 5)
#define DO3 BITBAND_REG(PTC->PDOR, 3)
//DI
#define DI1 BITBAND_REG(PTE->PDIR, 25)
#define DI2 BITBAND_REG(PTA->PDIR, 1)
#define DI3 BITBAND_REG(PTA->PDIR, 2)

//RS485_EN
#define RS485_EN  BITBAND_REG(PTD->PDOR, 4)
//AO2_DIR
#define AO2_DIR BITBAND_REG(PTE->PDOR, 24)

//KEY
#define KEY1 BITBAND_REG(PTE->PDIR, 16)
#define KEY2 BITBAND_REG(PTE->PDIR, 17)
#define KEY3 BITBAND_REG(PTE->PDIR, 18)
#define KEY4 BITBAND_REG(PTE->PDIR, 19)



/* TYPES */
typedef struct tagLvdtSeek
{
	uint32_t loop;
	uint32_t phase;
	uint32_t flag;
        uint32_t timer;
	uint32_t addone;
	uint16_t addata[SINGAL_NUM];
}LvdtSeek;


void clear_lvdt_tof();
void init_lvdt_pwm();
void set_lvdt_cv(uint16_t value);
void enable_lvdt_pwm();
void disable_lvdt_pwm();

void clear_ao2_tof();
void init_ao2_pwm();
void set_ao2_cv(uint16_t value);
void enable_ao2_pwm();
void disable_ao2_pwm();

int32_t init_adc(uint32_t instance);
void ADC_TEST_InstallCallback(uint32_t instance, uint32_t chnGroup, void (*callbackFunc)(void) );
uint16_t ADC_TEST_GetConvValueRAWInt(uint32_t instance, uint32_t chnGroup);


void init_trigger_pdb(uint32_t adcInstance);
void init_trigger_pit(uint32_t adcInstance);


void lvdtseekinit();
void seekzero(uint32_t sample_n);

void Kinetis_FlashInit();



extern unsigned long get_waittime();



#endif /*_ADC_HW_TRIGGER_H*/
