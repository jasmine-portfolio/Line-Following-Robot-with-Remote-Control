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
 
// Filename:            main.c
//
// Description:         Line following robot with remote control
//
// Target:              TMS320F28379D
//
// Authors:             Kento and Jasmine
//
// Date:                Dec. 1, 2024

// defines:
#define xdc__strict // suppress typedef warnings
#define VREFHI 3.0

// includes:
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <Headers/F2837xD_device.h>

//////////// function prototypes//////////
extern void DeviceInit(void);
extern void SciInit(void);
extern void SciSend(char data);
extern void AdcInit(void);
extern Uint16 AdcRead(char data);

//////////Variables declaration//////////

//swi
extern const Swi_Handle rpm1_swi;
extern const Swi_Handle rpm2_swi;
extern const Swi_Handle pid_swi;
extern const Swi_Handle rx_swi;
extern const Swi_Handle tx_swi;

//encoders
float rpm1;
float rpm2;

volatile uint32_t current_time1, current_time2;
volatile uint32_t last_time1 = 0, last_time2 = 0;
volatile uint32_t time_diff1, time_diff2;

volatile UInt tickCount = 0; //counter incremented by timer interrupt

// PID and motor speed variables
float dutyCycle1, dutyCycle2;  // Duty cycle for motors
float Kp = 15, Ki = 0, Kd = 0; // PID constants
float error = 0, P = 0, I = 0, D = 0, PID_value = 0; // PID calculation variables
float previous_error = 0, previous_I = 0; // Previous PID values for calculation
int base_motor_speed = 30; // Base speed of the motor
uint16_t rs1, rs2, rs3, rs4, rs5, rs6, rs7; // Reflective sensor values
volatile Bool isrFlag = FALSE; // Flag to indicate ISR activity

//sci rx & tx
volatile char receivedData[4]; // Stores data received from ESP32
volatile UInt carState = 1; // Car state (on/off)
Uint16 counter = 0; // Counter for data transmission


/* ======== main ======== */
Int main()
{
    // Initialization of peripherals and devices
    DeviceInit();
    SciInit();
    AdcInit();

    // Set initial duty cycle for motors
    dutyCycle1 = 0.0;
    dutyCycle2 = 0.0;

    // Update motor duty cycle using PWM
    EPwm1Regs.CMPA.bit.CMPA = (uint16_t)(EPwm1Regs.TBPRD * ((100.0-dutyCycle1) / 100.0)); // Update duty cycle for motor 1
    EPwm2Regs.CMPA.bit.CMPA = (uint16_t)(EPwm2Regs.TBPRD * ((100.0-dutyCycle2) / 100.0)); // Update duty cycle for motor 2
    EPwm3Regs.CMPA.bit.CMPA = (uint16_t)(EPwm3Regs.TBPRD * ((100.0-0) / 100.0)); // Update duty cycle for motor 2
    // Initialize the median filter for RPM calculations

    // jump to RTOS (does not return):
    BIOS_start();
    return (0);
}
/**********************************
 * Motors, Encoders (speed sensor)
 ***********************************/

// ISR for Encoder 1 (GPIO1)
void encoder_1_hwi(void)
{
    last_time1 = current_time1;
    current_time1 = tickCount;
    Swi_post(rpm1_swi);// Post Swi for calculate_rpm1_swi
}

// ISR for Encoder 2 (GPIO2)
void encoder_2_hwi(void)
{
    last_time2 = current_time2;
    current_time2 = tickCount;
    Swi_post(rpm2_swi);// Post Swi for calculate_rpm2_swi
}

// Swi function to calculate RPM for motor 1
void calculate_rpm1_swi(void)
{
    time_diff1 = current_time1 - last_time1;
    if (time_diff1 > 0)
    {  // Avoid divide by zero
        rpm1 = (60000.0) / ((float)time_diff1 * 20.0);
    }
}
// Swi function to calculate RPM for motor 2
void calculate_rpm2_swi(void)
{
    time_diff2 = current_time2 - last_time2;
    if (time_diff2 > 0)
    {  // Avoid divide by zero
        rpm2 =  (60000.0) / ((float)time_diff2 * 20.0);
    }
}

// swi for controlling motor speeds using PID control which is called from timer every 1ms
void control_loop_swi(void)
{
    // Read reflective sensors (for line following)
    rs1 = GpioDataRegs.GPADAT.bit.GPIO6;
    rs2 = GpioDataRegs.GPADAT.bit.GPIO7;
    rs3 = GpioDataRegs.GPADAT.bit.GPIO8;
    rs4 = GpioDataRegs.GPADAT.bit.GPIO9;
    rs5 = GpioDataRegs.GPADAT.bit.GPIO10;
    rs6 = GpioDataRegs.GPADAT.bit.GPIO11;
    rs7 = GpioDataRegs.GPADAT.bit.GPIO14;

    //Calibrated valued based on rx data
    if (receivedData[0] == 1) {//Control state 1= PID control
        Kp = receivedData[1]/10;
        Ki = receivedData[2]/100;
        Kd = receivedData[3]/100;
    }
    // Calculate the error based on the active sensor
        if (rs1 == 1)
            error = 3;
        else if (rs2 == 1)
            error = 2;
        else if (rs3 == 1)
            error = 1;
        else if (rs4 == 1)
            error = 0;
        else if (rs5 == 1)
            error = -1;
        else if (rs6 == 1)
            error = -2;
        else if (rs7 == 1)
            error = -3;
        else  // No sensor is active
            error = 0; // Or handle this case as needed

        // PID calculations
        P = error;
        I = I + previous_I;
        D = error - previous_error;
        PID_value = (Kp * P) + (Ki * I) + (Kd * D);
        // Update previous PID values for next iteration
        previous_I = I;
        previous_error = error;

        // Calculate duty cycles based on PID value
        dutyCycle1 = base_motor_speed - PID_value;
        dutyCycle2 = base_motor_speed+ PID_value;

        // Clamp duty cycles to be between 0 and 100
        if (dutyCycle1 > 100) dutyCycle1 = 100;
        if (dutyCycle1 < 0) dutyCycle1 = 0;

        if (dutyCycle2 > 100) dutyCycle2 = 100;
        if (dutyCycle2 < 0) dutyCycle2 = 0;


    // Apply PWM duty cycle to motors
    EPwm1Regs.CMPA.bit.CMPA = (uint16_t)(EPwm1Regs.TBPRD * ((100.0-dutyCycle1) / 100.0)); // Update duty cycle for motor 1
    EPwm2Regs.CMPA.bit.CMPA = (uint16_t)(EPwm2Regs.TBPRD * ((100.0-dutyCycle2) / 100.0)); // Update duty cycle for motor 2
}


//Timer interrupt function: Increments tick count and posts control_loop_swi and txData_swi
Void msTimerFxn(Void)
{
    tickCount++; //increment the tick counter

    if (tickCount - last_time1 > 500) rpm1 = 0.0;  // Encoder 1 stopped
    if (tickCount - last_time2 > 500) rpm2 = 0.0;  // Encoder 2 stopped

    if(tickCount % 1000 == 0) {
        isrFlag = TRUE;
    }

    if(carState){
        Swi_post(pid_swi); // post to control_loop_swi
    } else{ // when car is off
        dutyCycle1=0;
        dutyCycle2=0;
        EPwm1Regs.CMPA.bit.CMPA = (uint16_t)(EPwm1Regs.TBPRD * ((100.0-dutyCycle1) / 100.0)); // Update duty cycle for motor 1
        EPwm2Regs.CMPA.bit.CMPA = (uint16_t)(EPwm2Regs.TBPRD * ((100.0-dutyCycle2) / 100.0)); // Update duty cycle for motor 2
    }
    Swi_post(tx_swi); // post to txData_swi
}

//Idle function that is called repeatedly from RTOS
Void myIdleFxn(Void) {
    // Heart beat led
    if(isrFlag){
        isrFlag = FALSE;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // Toggle GPIO34 for heartbeat
    }
}

/**********************************
 * SCI (UART)
 ***********************************/

//Reads incoming data from the serial port (ESP32)
Void sci_rx_hwi(Void) {
    // Read the received data
    receivedData[0] = ScibRegs.SCIRXBUF.all;
    receivedData[1] = ScibRegs.SCIRXBUF.all;
    receivedData[2] = ScibRegs.SCIRXBUF.all;
    receivedData[3] = ScibRegs.SCIRXBUF.all;

    // Clear the interrupt flag
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;

//Display data for troubleshooting
    System_printf("Received data0: %d\n", (int)receivedData[0]);
    System_printf("Received data1:%d\n", (int)receivedData[1]);
    System_printf("Received data2:%d\n", (int)receivedData[2]);
    System_printf("Received data3:%d\n", (int)receivedData[3]);

    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Toggle GPIO31 (Blue LED) to indicate data is being received
    Swi_post(rx_swi); // post to rxData_swi

}
//Process data from sci_rx_hwi
Void rxData_swi(Void)
{
    //Control state 2= Car control
       if (receivedData[0] ==2) {
           //Car state
           if (receivedData[1] ==1){
               System_printf("Car ON, ");
               carState=1;
           } else{
               System_printf("Car OFF, ");
               carState=0;
           }
           //Speaker state
           if (receivedData[3] ==1) {
               EPwm3Regs.CMPA.bit.CMPA = (uint16_t)(EPwm3Regs.TBPRD * ((100.0 - 25) / 100.0)); // Update duty cycle for speaker
               System_printf("Speaker ON\n");
           } else{
               EPwm3Regs.CMPA.bit.CMPA = (uint16_t)(EPwm3Regs.TBPRD * ((100.0 - 0) / 100.0)); // Update duty cycle for speaker
               System_printf("Speaker OFF\n");
           }
           //Speed
           if(carState==1){
               base_motor_speed=receivedData[2];
               System_printf("Speed: %d\n", receivedData[2]);
             } else
               base_motor_speed=0;

           // Reset received data variable
           receivedData[0] = '\0';
           receivedData[1] = '\0';
           receivedData[2] = '\0';
           receivedData[3] = '\0';

       }
}



/**********************************
 * Power consumption
 ***********************************/
// Sends RPM data to ESP32 (in 2 bytes)
Void sendRpm(Void){
    Uint16 rpm1Uint=(Uint16)rpm1;

    char OneByte;
    OneByte=(rpm1Uint & 0xFF);
    SciSend(OneByte);
    OneByte=(rpm1Uint >> 8) & 0xFF;
    SciSend(OneByte);
}

// Measures power by averaging ADC readings and calculates power
// Sends calculated power to ESP32 (in 2 bytes)
Void sendPower(Void){
    Uint16 AdcValid;// check if adc value is valid
    Uint16 count=5;
    Uint16 adcValue1;//sum of 5 measured value from adc (0-4096)
    Uint16 adcValue2;//sum of 5 measured value from adc (0-4096)
    Uint16 power;

    //Get voltage 1
    adcValue1 =0;
    while (count!=0) {
        AdcValid = AdcRead(3); // Read the analog input channel 2 adcValue
        count--;
        adcValue1+=AdcValid;
    }
    adcValue1 = (int) (adcValue1 / 3.413);

    //Get voltage 2
    count=5;
    adcValue2 =0;
    while (count!=0) {
        AdcValid = AdcRead(2); // Read the analog input channel 2 adcValue
        count--;
        adcValue2+=AdcValid;
    }
    adcValue2 = (int) (adcValue2 / 3.413);

    //Calculate power
    Uint16 current=(adcValue2-adcValue1); // Resistor Used 1 Ohm =>>> Voltage = Current
    power=current*5;

    //Send power
    char OneByte;
    OneByte=(power & 0xFF); // Low Byte of 16 Bit
    SciSend(OneByte);
    OneByte=(power >> 8) & 0xFF; // High Byte of 16 Bit
    SciSend(OneByte);
}

// Sends power and rpm every 1s
Void txData_swi(Void){
    counter++;// Increment counter to track time
    // If 1 second has passed (1000ms), send power and RPM data
    if (counter>1000){
        counter=0;
        sendPower();
        sendRpm();
    }
}
