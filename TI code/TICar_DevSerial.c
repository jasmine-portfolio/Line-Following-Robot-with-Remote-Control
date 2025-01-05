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
// Filename:            TICar_DevUART.c
//
// Description:         Initialization code for Synchronous Communication Interface SCI (UART)
//
// Target:              TMS320F28379D
//
// Author:              Jasmine
//
// Date:                10 November 2024


#include <Headers/F2837xD_device.h>

// Define the GPIO pins for SCI-B
#define GPIO_TX 18
#define GPIO_RX 19


// Function to initialize the SCI (UART)
void SciInit(void) {

    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_B = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;  // Set GPIO18 to SCITXDA
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;   // Set GPIO18 as output
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;  // Set GPIO19 to SCIRXDA
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;   // Set GPIO19 as input

    EDIS;

    ScibRegs.SCICCR.all = 0x0007;   // 1 stop bit, No loopback
                                    // No parity, 8 char bits,
                                    // async mode, idle-line protocol
    ScibRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.all = 0x0003;  // Enable transmit and receive
    ScibRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset

    // Configure the baud rate for 115200 with a 200 MHz clock
    ScibRegs.SCIHBAUD.all =0;  // High byte of baud rate divider
    ScibRegs.SCILBAUD.all =54; // Low byte of baud rate divider

    // Enable SCI FIFO
    ScibRegs.SCIFFTX.all = 0xE040; // Reset FIFOs, set TX FIFO level to 16
    ScibRegs.SCIFFRX.all = 0x2044; // Reset RX FIFO, set RX FIFO level to 4: 1 byte= 2041, 3 bytes=2043, 4 bytes= 2044
    ScibRegs.SCIFFCT.all = 0x0;

    // Enable RX interrupt
    ScibRegs.SCIFFRX.bit.RXFFIENA = 1;   // Enable RX FIFO interrupt

    EINT;                               // Enable Global interrupt INTM
}

// Function to send a single character
void SciSend(char data) { //SciSend
    // Wait until transmission buffer is ready
    while (ScibRegs.SCICTL2.bit.TXRDY == 0);

    // Send the character
    ScibRegs.SCITXBUF.all = data;
}


