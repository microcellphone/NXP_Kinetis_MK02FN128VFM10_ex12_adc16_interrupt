/*
 * Copyright 2016-2023 NXP
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
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
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

/**
 * @file    MK02FN128VFM10_ex00_basic_MK02FN128VFM10.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK02F12810.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "fsl_adc16.h"
#include "MK02FN128VFM10_uart.h"
#include "xprintf.h"
#include "my_delay.h"

/* TODO: insert other definitions and declarations here. */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ADC16_CHANNEL_GROUP 0U
#define ADC16_USER_CHANNEL  4U /* PTB0, ADC0_SE4b */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue;
volatile uint32_t g_Adc16InterruptCounter;
const uint32_t g_Adc16_12bitFullRange = 4096U;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* ADC0_IRQn interrupt handler */
void ADC0_IRQHANDLER(void) {
  /* Array of result values*/
  uint32_t result_values[2] = {0};
  /* Get flags for each group */
  for ( int i=0; i<2; i++){
  uint32_t status = ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, i);
  	if ( status == kADC16_ChannelConversionDoneFlag){
  		result_values[i] = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, i);
  	}
  }

  /* Place your code here */
  g_Adc16ConversionDoneFlag = true;
  /* Read conversion result to clear the conversion completed flag. */
  g_Adc16ConversionValue = result_values[0];
  g_Adc16InterruptCounter++;

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

/*
 * @brief   Application entry point.
 */
int main(void) {
    adc16_channel_config_t adc16ChannelConfigStruct;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U)) while (1);

    xdev_out(UART0_PutByte);
    xdev_in(UART0_GetByte);

    xprintf("\r\nMK02FN128VFM10_ex12_adc16_interrupt\r\n");

#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(ADC0_PERIPHERAL)) {
        xprintf("ADC16_DoAutoCalibration() Done.\r\n");
    } else {
        xprintf("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    xprintf("ADC Full Range: %d\r\n", g_Adc16_12bitFullRange);

    adc16ChannelConfigStruct.channelNumber                        = ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

    g_Adc16InterruptCounter = 0U;

    /* Force the counter to be placed into memory. */
    volatile static int cnt = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        /* Delay 1000 ms */
        SysTick_Delay(1000U);
        g_Adc16ConversionDoneFlag = false;
        /*
         When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
         function, which works like writing a conversion command and executing it. For another channel's conversion,
         just to change the "channelNumber" field in channel configuration structure, and call the function
         "ADC16_ChannelConfigure()"" again.
         Also, the "enableInterruptOnConversionCompleted" inside the channel configuration structure is a parameter for
         the conversion command. It takes affect just for the current conversion. If the interrupt is still required
         for the following conversion, it is necessary to assert the "enableInterruptOnConversionCompleted" every time
         for each command.
        */
        ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
        while (!g_Adc16ConversionDoneFlag);
        xprintf("ADC Value: %d\r\n", g_Adc16ConversionValue);
        xprintf("ADC Interrupt Count: %d\r\n", g_Adc16InterruptCounter);
        cnt++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}
