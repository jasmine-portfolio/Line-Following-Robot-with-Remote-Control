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
// Filename:            LabProject_DeviceInit.c
//
// Description:	        Initialization code
//
// Target:              TMS320F28379D
//
// Author:              Kento
//
// Date:                6Nov2024


#include <Headers/F2837xD_EPwm_defines.h> // ks // EPWM header file used in C2000Ware PWM example
#include <Headers/F2837xD_device.h>
#include <Headers/F2837xD_epwm.h>

extern void DelayUs(Uint16);


void initADC()
{
    EALLOW;
    //---------------------------------------------------------------
    // INITIALIZE A-D
    //---------------------------------------------------------------
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1; // enable A-D clock for ADC-A
    AdcaRegs.ADCCTL2.bit.PRESCALE = 0xf;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    // generate INT pulse on end of conversion:
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    // wait 1 ms after power-up before using the ADC:
    DelayUs(1000);

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 2;   // trigger source = CPU1 Timer 1
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 5;    // set SOC0 to sample ADCA5
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 139;   // set SOC0 window to 139 SYSCLK cycles
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // connect interrupt ADCINT1 to EOC0
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // enable interrupt ADCINT1
    EDIS;
}

// ks // function to set up PWM1
void initEPWM1(uint16_t frequency, uint16_t clkDiv, uint16_t hspClkDiv)
{
    EALLOW;
    // Enable EPWM1 and set GPIO0 to PWM mode
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Set GPIO1 to EPWM1
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;     // Enable EPWM1 clock
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Disable EPWM Time Base Clock sync

    // Set the period (frequency) for the PWM
    uint32_t sysClock = 200000000; // 200 MHz system clock
    uint16_t tbprd = (sysClock / (frequency * 4 * (1 << clkDiv) * (1 << hspClkDiv))) - 1;
    EPwm1Regs.TBPRD = tbprd;

    // Initialize Timer Counter and Phase
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;           // Reset counter

    // Set Compare value based on duty cycle
    EPwm1Regs.CMPA.bit.CMPA = (uint16_t)(tbprd * 0);

    // Setup counter mode and clock division
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Up-down counting mode
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.CLKDIV = clkDiv;           // Set clock divider
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = hspClkDiv;     // Set high-speed clock divider

    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;   // Set PWM1A on compare-up
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear PWM1A on compare-down

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Enable EPWM Time Base Clock sync
    EDIS;
}

// ks // function to set up PWM
void initEPWM2(uint16_t frequency, uint16_t clkDiv, uint16_t hspClkDiv)
{
    EALLOW;
    // Enable EPWM2 and set GPIO3 to PWM mode
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Set GPIO3 to EPWM2
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;     // Enable EPWM2 clock
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Disable EPWM Time Base Clock sync

    // Set the period (frequency) for the PWM
    uint32_t sysClock = 200000000; // 200 MHz system clock
    uint16_t tbprd = (sysClock / (frequency * 4 * (1 << clkDiv) * (1 << hspClkDiv))) - 1;
    EPwm2Regs.TBPRD = tbprd;

    // Initialize Timer Counter and Phase
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;           // Reset counter

    // Set Compare value based on duty cycle
    EPwm2Regs.CMPA.bit.CMPA = (uint16_t)(tbprd * 0);

    // Setup counter mode and clock division
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Up-down counting mode
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.CLKDIV = clkDiv;           // Set clock divider
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = hspClkDiv;     // Set high-speed clock divider

    EPwm2Regs.AQCTLB.bit.CAU = AQ_SET;   // Set PWM2A on compare-up
    EPwm2Regs.AQCTLB.bit.CAD = AQ_CLEAR; // Clear PWM2A on compare-down

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Enable EPWM Time Base Clock sync
    EDIS;
}
// ks // function to set up PWM3
void initEPWM3(uint16_t frequency, uint16_t clkDiv, uint16_t hspClkDiv)
{
    EALLOW;
    // Enable EPWM3 and set GPIO4 to PWM mode
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Set GPIO4 to EPWM3
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;     // Enable EPWM1 clock
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Disable EPWM Time Base Clock sync

    // Set the period (frequency) for the PWM
    uint32_t sysClock = 200000000; // 200 MHz system clock
    uint16_t tbprd = (sysClock / (frequency * 4 * (1 << clkDiv) * (1 << hspClkDiv))) - 1;
    EPwm3Regs.TBPRD = tbprd;

    // Initialize Timer Counter and Phase
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;           // Reset counter

    // Set Compare value based on duty cycle
    EPwm3Regs.CMPA.bit.CMPA = (uint16_t)(tbprd * 0);

    // Setup counter mode and clock division
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Up-down counting mode
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm3Regs.TBCTL.bit.CLKDIV = clkDiv;           // Set clock divider
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = hspClkDiv;     // Set high-speed clock divider

    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;   // Set PWM3A on compare-up
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear PWM3A on compare-down

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Enable EPWM Time Base Clock sync
    EDIS;
}


void initBoardLED(void)
{
    EALLOW;
    // initialize GPIO lines:
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0; // D9 (red LED)
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
    //GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // Toggle GPIO31 for debugging


    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0; // D10 (blue LED)
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
    //GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;  // Toggle GPIO31 for debugging

    EDIS;
}

void initEncoders(void) // KS Setting up GPIOs for interrupts to calculate RPM of motors
{
    EALLOW;
    //XINT1
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;  // Enable GPIO on GPIO0
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 0;   // Configure GPIO0 as input

    InputXbarRegs.INPUT4SELECT = 1;           // Map GPIO0 to XINT1
    XintRegs.XINT1CR.bit.POLARITY = 0;        // Rising edge detection
    XintRegs.XINT1CR.bit.ENABLE = 1;          // Enable the XINT1 interrupt

    //XINT2
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;   // Enable pullup on GPIO2
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;  // Enable GPIO on GPIO2
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;   // Configure GPIO2 as input

    InputXbarRegs.INPUT5SELECT = 2;           // Map GPIO2 to XINT2
    XintRegs.XINT2CR.bit.POLARITY = 1;        // Rising edge detection
    XintRegs.XINT2CR.bit.ENABLE = 1;          // Enable the XINT2 interrupt
    EDIS;
}

void initRS(void) // KS // Setting up the Reflectance Array Sensors as inputs
{
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;  // Enable GPIO on GPIO0
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 0;   // Configure GPIO0 as input

    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;  // Enable GPIO on GPIO0
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 0;   // Configure GPIO0 as input

    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;  // Enable GPIO on GPIO0
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 0;   // Configure GPIO0 as input

    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;  // Enable GPIO on GPIO0
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;   // Configure GPIO0 as input

    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;  // Enable GPIO on GPIO0
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;   // Configure GPIO0 as input

    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;  // Enable GPIO on GPIO0
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 0;   // Configure GPIO0 as input

    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;  // Enable GPIO on GPIO0
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;   // Configure GPIO0 as input
}

void DeviceInit(void)
{
    // ks // initialize GPIO and peripheral registers:
    initEPWM1(15000, TB_DIV1, TB_DIV1); // ks // initialize PWM1
    initEPWM2(15000, TB_DIV1, TB_DIV1); // ks // initialize PWM2
    initEPWM3(15000, TB_DIV1, TB_DIV1); // ks // initialize PWM3

    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;  // Enable GPIO on GPIO25
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;   // Configure GPIO0 as input
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
    EDIS;

    //initADC();
    initBoardLED();
    initEncoders();
    initRS();
   }
