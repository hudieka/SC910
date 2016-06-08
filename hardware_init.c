/*
* Copyright (c) 2013-2014, Freescale Semiconductor, Inc.
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

#include "board.h"
#include "pin_mux.h"
#include "fsl_clock_manager.h"
#include "fsl_uart_edma_driver.h"
#include "fsl_uart_driver.h"

#include "adc_hw_trigger.h"

// Store runtime state structure for the eDMA driver
edma_state_t                state;
// Store runtime state structure for UART driver with EDMA
uart_edma_state_t           uartStateEdma;

void hardware_init(void) {
  
  dac_converter_config_t dacUserConfig;
  uart_state_t uartState;
  
  // Config the eDMA driver
  edma_user_config_t userConfig = {
    .chnArbitration  = kEDMAChnArbitrationRoundrobin,
    .notHaltOnError  = false
  };
  // Config the UART(RS485)
  uart_user_config_t uartConfig = {
    .bitCountPerChar = kUart8BitsPerChar,
    .parityMode      = kUartParityDisabled,
    .stopBitCount    = kUartOneStopBit,
    .baudRate        = BOARD_DEBUG_UART_BAUD
  };
  // Config the UART_EDMA driver
  uart_edma_user_config_t     uartedmaConfig = {
    .bitCountPerChar = kUart8BitsPerChar,
    .parityMode      = kUartParityDisabled,
    .stopBitCount    = kUartOneStopBit,
    .baudRate        = BOARD_DEBUG_UART_BAUD
  };
  
  /* enable clock for PORTs */
  CLOCK_SYS_EnablePortClock(PORTA_IDX);
  CLOCK_SYS_EnablePortClock(PORTE_IDX);
  CLOCK_SYS_EnablePortClock(PORTC_IDX);
  CLOCK_SYS_EnablePortClock(PORTD_IDX);
  CLOCK_SYS_EnablePortClock(PORTB_IDX);
  
  /* Init board clock */
  BOARD_ClockInit();
  //dbg_uart_init();
  
  //UART(RS485)
  configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);
  //UART_DRV_Init(BOARD_DEBUG_UART_INSTANCE, &uartState, &uartConfig);
  
  // Initialize EDMA module for UART
  EDMA_DRV_Init(&state, &userConfig);
  UART_DRV_EdmaInit(BOARD_DEBUG_UART_INSTANCE, &uartStateEdma, &uartedmaConfig);
  //En Rx
  UART0_C2 |= UART_C2_RE_MASK | UART_C2_RIE_MASK;
  //En Interrupt
  NVIC_EnableIRQ(UART0_RX_TX_IRQn);  
  NVIC_SetPriority(UART0_RX_TX_IRQn, 0);
  
  //DAC
  DAC_DRV_StructInitUserConfigNormal(&dacUserConfig);
  DAC_DRV_Init(BOARD_DAC_DEMO_DAC_INSTANCE, &dacUserConfig);
  
  //PWM for LVDT 
  PORT_PCR_REG(PORTC, 4)  |= PORT_PCR_MUX(4);
  
  //AO2
  PORT_PCR_REG(PORTE, 24)  |= PORT_PCR_MUX(1);          //DIR
  PORT_PCR_REG(PORTB, 0)  |= PORT_PCR_MUX(3);           //PWM
  PTE->PDDR |= (1 <<24);
  
  //DO
  PORT_PCR_REG(PORTA, 19)  |= PORT_PCR_MUX(1);          //DO1
  PORT_PCR_REG(PORTD, 5)  |= PORT_PCR_MUX(1);           //DO2
  PORT_PCR_REG(PORTC, 3)  |= PORT_PCR_MUX(1);           //DO3
  
  PTA->PDDR |= (1 <<19);
  PTD->PDDR |= (1 <<5);
  PTC->PDDR |= (1 <<3);
  //DI
  PORT_PCR_REG(PORTE, 25)  |= PORT_PCR_MUX(1);          //DI1
  PORT_PCR_REG(PORTA, 1)  |= PORT_PCR_MUX(1);           //DI2
  PORT_PCR_REG(PORTA, 2)  |= PORT_PCR_MUX(1);           //DI3
  //KEY
  PORT_PCR_REG(PORTE, 16)  |= PORT_PCR_MUX(1);          //KEY1
  PORT_PCR_REG(PORTE, 17)  |= PORT_PCR_MUX(1);          //KEY2
  PORT_PCR_REG(PORTE, 18)  |= PORT_PCR_MUX(1);          //KEY3
  PORT_PCR_REG(PORTE, 19)  |= PORT_PCR_MUX(1);          //KEY4
  
  //SPI_LCD
  PORT_PCR_REG(PORTC, 5)  |= PORT_PCR_MUX(1);           //CLK     
  PORT_PCR_REG(PORTC, 6)  |= PORT_PCR_MUX(1);           //MOSI
  PORT_PCR_REG(PORTC, 7)  |= PORT_PCR_MUX(1);           //MISO
  
  PTC->PDDR |= (1 << 5);
  PTC->PDDR |= (1 << 6);
  
  //RS485_EN
  PORT_PCR_REG(PORTD, 4)  |= PORT_PCR_MUX(1);
  PTD->PDDR |= (1 <<4);
  
#if 1
  // initialize the ADC
  init_adc(ADC_INST); 
  // setup pwm for lvdt
  init_lvdt_pwm();
  // setup pwm for AO2
  init_ao2_pwm();
  // setup the PIT trigger source
  init_trigger_pit(ADC_INST);
  
#endif
  
  
}

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.4 [05.10]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
