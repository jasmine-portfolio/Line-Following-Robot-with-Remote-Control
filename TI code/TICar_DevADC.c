/*
 * MIT License
 * 
 * Copyright (c) 2025 jasmine-portfolio
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// Filename:            TICar_DevADC.c
//
// Description:         Initialization code for Synchronous Communication Interface SCI (UART)
//
// Target:              TMS320F28379D
//
// Author:              Jasmine
//
// Date:                18 November 2024


#include <Headers/F2837xD_device.h>


#define ADC_ADCA 0
#define ADC_ADCB 1
#define ADC_RESOLUTION_12BIT 0
#define ADC_SIGNALMODE_SINGLE 0


//
// ADC Initialization for block A
// using channel 2 and channel 3

void AdcInit(void) {
    volatile uint32_t count;
    EALLOW;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;    // Enable the clock for ADC-B
    AdcbRegs.ADCCTL2.bit.PRESCALE = 4;    // Set ADC clock prescaler to /4 (50 MHz ADC clock)
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;    // Power up the ADC

    AdcbRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_12BIT;
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_SINGLE;

    for (count=0; count<2000;count++){}                // Delay for 1ms to allow ADC to power up
    EDIS;
}

Uint16 AdcRead(uint16_t channel) {
    Uint16 adcResult;

    // Configure SOC1 of ADC-B to convert the selected channel
    EALLOW;
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = channel; // Set SOC1 channel select to ADC-B channel 5
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 15;      // Set sample window to 16 cycles
    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1;   // Set end of conversion to SOC1
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1;     // Enable ADC interrupt 2

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;   // Clear ADC interrupt flag
    EDIS;

    // Start conversion on SOC1
    AdcbRegs.ADCSOCFRC1.all = 0x02;
    // Wait for end of conversion
    while (AdcbRegs.ADCINTFLG.bit.ADCINT2 == 0);
    // Read the result from the correct result register
    adcResult = AdcbResultRegs.ADCRESULT1;
    // Clear ADC interrupt flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;

    return adcResult;
}

