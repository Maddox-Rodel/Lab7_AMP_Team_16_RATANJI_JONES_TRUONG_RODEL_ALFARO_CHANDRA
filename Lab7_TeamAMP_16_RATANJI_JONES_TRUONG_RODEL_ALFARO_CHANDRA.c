//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>          // strlen() for UARTSendBoth; strncmp() for command lookup (Step 6)
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h" // Lab 5 addition. StepConfigure/Enable(), ADCIntEnable/Clear(), ADCProcessorTrigger(), ADCSequenceDataGet().
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
//*****************************************************************************
//
// LAB 6 -- Right-wall following with PID
//
//   STR: both motors at 99% duty, start the 50 ms timer        (tasks 3, 5)
//   Timer 2A ISR (every 50 ms): trigger the distance reading    (task 6)
//   ADC ISR: right wheel distance -> LED, PID -> motor duty     (tasks 7.1, 7.2)
//   STP: timer off, motors off, LED off
//
//*****************************************************************************
#include "driverlib/pwm.h"      // LAB 6 ADDITION: PWMGenConfigure(), PWMPulseWidthSet(), PWMOutputState()
#include "driverlib/timer.h"    // LAB 6 ADDITION: TimerConfigure(), TimerLoadSet(), TimerEnable()

//*****************************************************************************
// [ADC] Distance sensors (task 4)
//   RIGHT sensor -> PE3 = AIN0 (SS1 steps 0-1), used by the PID
//   FRONT sensor -> PE2 = AIN1 (SS1 steps 2-3), kept for Lab 7
//*****************************************************************************
#define ADC_SEQ             1       // sample sequencer 1 (FIFO holds 4 samples)
#define ADC_NUM_SAMPLES     4       // total samples per trigger
#define SAMPLES_PER_SENSOR  2       // 2 right + 2 front
#define SAMPLE_PERIOD_MS    50      // task 5: 50 ms periodic timer

//*****************************************************************************
// LAB 6 ADDITION
// [WALL] Target distance and LED bands (task 7.1)
//   wheel distance = sensor distance - SENSOR_TO_WHEEL_CM
//*****************************************************************************
#define TARGET_CM           10.0f   // right WHEEL 10 cm from the wall
#define SENSOR_TO_WHEEL_CM  4.0f    // measure on the robot and update
#define GREEN_BAND_CM       1.0f    // |error| <= 1 cm      -> GREEN
#define YELLOW_BAND_CM      2.0f    // 1 < |error| <= 2 cm  -> YELLOW, else RED

//*****************************************************************************
// LAB 6 ADDITION
// [PID] Gains (task 7.2, W6C1 slides 14-15). Tune Kp first, then Ki, then Kd.
//*****************************************************************************
#define KP                  4.0f
#define KI                  0.0f
#define KD                  0.0f
#define I_SUM_LIMIT         50.0f   // caps the error sum to avoid integral windup
#define MAX_ADJUST_PCT      40.0f   // largest PID correction (% duty)

//*****************************************************************************
// LAB 6 ADDITION
// [MOTOR] PWM (task 2: Lab 4 PWM moved from the LED to the motors)
//   PB6 = M0PWM0 -> DRV8835 AENBL  (left speed)
//   PB7 = M0PWM1 -> DRV8835 BENBL  (right speed)
//   PA2          -> DRV8835 APHASE (left direction)
//   PA3          -> DRV8835 BPHASE (right direction)
//*****************************************************************************
#define PWM_FREQ_HZ         10000   // 40 MHz / 10 kHz = 4000 clocks per period
#define BASE_DUTY_PCT       99.0f   // task 3: as fast as possible
#define MAX_DUTY_PCT        99.0f   // pulse width must be less than the period
#define MIN_DUTY_PCT        0.0f

#define LEFT_PHASE_FWD      0       // PA2 level for forward
#define RIGHT_PHASE_FWD     1       // PA3 level for forward (right motor is mirrored)

// [LED] PF1 = red, PF2 = blue, PF3 = green; yellow = red + green
#define LED_ALL     (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
#define LED_RED     (GPIO_PIN_1)
#define LED_GREEN   (GPIO_PIN_3)
#define LED_YELLOW  (GPIO_PIN_1 | GPIO_PIN_3)

//*****************************************************************************
// [GLOBALS] volatile because ISRs write them and other code reads them
// LAB 5 -> LAB 6 EDIT: Lab 5's AdcAverage/DistanceCm replaced by a distance
// per sensor, the wheel distance, the Running flag and the PID state.
//*****************************************************************************
volatile float RightDistanceCm = 0.0f;  // right sensor to wall
volatile float RightWheelCm    = 0.0f;  // right wheel to wall
volatile float FrontDistanceCm = 0.0f;  // front sensor to wall (Lab 7)
volatile bool  Running = false;         // true after STR, false after STP
volatile float ErrSum  = 0.0f;          // PID: sum of past errors (I term)
volatile float ErrPrev = 0.0f;          // PID: previous error (D term)
uint32_t PWMPeriod = 0;                 // PWM clocks per period, set in PWM_init()

// Lab 3 command buffer: filled by UART1IntHandler, read by main
#define CMD_LEN 3
volatile char    CmdBuffer[CMD_LEN + 1];
volatile uint8_t CmdIndex = 0;          // next empty slot in CmdBuffer
volatile bool    CmdReady = false;      // true when 3 letters are waiting

// Command table entry: 3-letter name + the function that runs it
typedef struct
{
    char CmdName[CMD_LEN + 1];
    void (*pfnHandler)(void);
} Command;

//*****************************************************************************
// [ISR] UART0 (Tera Term): echo every character to both terminals
//*****************************************************************************
void
UART0IntHandler(void)
{
    // Clear the active interrupts so the handler doesn't re-enter immediately
    uint32_t ui32Status = ROM_UARTIntStatus(UART0_BASE, true);
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    // Empty the receive FIFO
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        uint8_t ui8Char = ROM_UARTCharGetNonBlocking(UART0_BASE);
        ROM_UARTCharPutNonBlocking(UART0_BASE, ui8Char);
        ROM_UARTCharPutNonBlocking(UART1_BASE, ui8Char);
    }
}

//*****************************************************************************
// [ISR] UART1 (Bluetooth): echo to both terminals, then collect 3-letter
// commands. main() runs the command once CmdReady is set.
//*****************************************************************************
void
UART1IntHandler(void)
{
    uint32_t ui32Status = ROM_UARTIntStatus(UART1_BASE, true);
    ROM_UARTIntClear(UART1_BASE, ui32Status);

    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        uint8_t ui8Char = ROM_UARTCharGetNonBlocking(UART1_BASE);
        ROM_UARTCharPutNonBlocking(UART1_BASE, ui8Char);
        ROM_UARTCharPutNonBlocking(UART0_BASE, ui8Char);

        if((ui8Char == '\r') || (ui8Char == '\n'))
        {
            CmdIndex = 0;               // Enter cancels a partial command
            continue;                   // CR/LF is never stored
        }

        CmdBuffer[CmdIndex++] = (char)ui8Char;
        if(CmdIndex >= CMD_LEN)         // 3rd letter: command complete
        {
            CmdIndex = 0;
            CmdReady = true;
        }
    }
}

//*****************************************************************************
// [UART] Send a string to both terminals.
// Blocking (waits for FIFO space), so call it from main, not from an ISR.
//*****************************************************************************
void
UARTSendBoth(const char *pcString)
{
    const char *p;

    for(p = pcString; *p; p++) ROM_UARTCharPut(UART0_BASE, *p);
    for(p = pcString; *p; p++) ROM_UARTCharPut(UART1_BASE, *p);
}

//*****************************************************************************
// LAB 6 ADDITION
// [MOTOR] Set one motor's duty cycle in %. Clamped to 0-99%; 0% turns the
// output off completely.
//*****************************************************************************
void
SetMotorDuty(uint32_t ui32Out, uint32_t ui32Bit, float fPct)
{
    if(fPct > MAX_DUTY_PCT) fPct = MAX_DUTY_PCT;
    if(fPct < MIN_DUTY_PCT) fPct = MIN_DUTY_PCT;

    if(fPct <= 0.0f)
    {
        PWMOutputState(PWM0_BASE, ui32Bit, false);
        return;
    }

    // % duty -> pulse width in PWM clocks (e.g. 99% of 4000 = 3960)
    uint32_t ui32Width = (uint32_t)((float)PWMPeriod * fPct / 100.0f);
    if(ui32Width < 1) ui32Width = 1;    // width 0 is not allowed

    PWMPulseWidthSet(PWM0_BASE, ui32Out, ui32Width);
    PWMOutputState(PWM0_BASE, ui32Bit, true);
}

// LAB 6 ADDITION: set both motors -- left = PB6 (PWM_OUT_0), right = PB7 (PWM_OUT_1)
void
SetMotors(float fLeftPct, float fRightPct)
{
    SetMotorDuty(PWM_OUT_0, PWM_OUT_0_BIT, fLeftPct);
    SetMotorDuty(PWM_OUT_1, PWM_OUT_1_BIT, fRightPct);
}

//*****************************************************************************
// LAB 6 ADDITION
// [MOTOR] PWM_init (task 2): Lab 4 PWM setup moved from the LED (PF1) to the
// motor driver pins. Outputs stay OFF until STR.
//*****************************************************************************
void
PWM_init(void)
{
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);                 // PWM clock = 40 MHz

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))  {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    // Speed pins: PB6 -> M0PWM0, PB7 -> M0PWM1
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    // Direction pins: PA2 (left), PA3 (right) as GPIO outputs, set to forward
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3,
                 (LEFT_PHASE_FWD  ? GPIO_PIN_2 : 0) |
                 (RIGHT_PHASE_FWD ? GPIO_PIN_3 : 0));

    // Generator 0 drives PWM_OUT_0 and PWM_OUT_1: count-down mode, 10 kHz
    PWMPeriod = SysCtlClockGet() / PWM_FREQ_HZ;
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWMPeriod);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);   // motors off
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

//*****************************************************************************
// LAB 6 ADDITION
// [TIMER] Timer_init (task 5): Timer 2A, periodic, 50 ms.
// Load = 40 MHz / 1000 x 50 - 1 = 1,999,999. Enabled by STR, disabled by STP.
//*****************************************************************************
void
Timer_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)) {}

    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);    // reloads automatically
    TimerLoadSet(TIMER2_BASE, TIMER_A, (SysCtlClockGet() / 1000) * SAMPLE_PERIOD_MS - 1);

    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER2A);
}

//*****************************************************************************
// LAB 6 ADDITION
// [ISR] Timer 2A, every 50 ms: trigger the sensor reading (task 6).
// Replaces the Lab 5 ADCProcessorTrigger() + SysCtlDelay() in main's loop.
//*****************************************************************************
void
Timer2AIntHandler(void)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    ADCProcessorTrigger(ADC0_BASE, ADC_SEQ);
}

//*****************************************************************************
// [ADC] ADC_init (task 4)
// LAB 5 -> LAB 6 EDIT: Lab 5 took 4 samples of the right sensor only. Now
// SS1 takes 2 samples of each sensor (right = CH0, front = CH1), with 4x
// hardware oversampling to reduce noise.
//*****************************************************************************
void
ADC_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))  {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);   // PE3 right, PE2 front
    ADCHardwareOversampleConfigure(ADC0_BASE, 4);               // each sample = average of 4

    ADCSequenceDisable(ADC0_BASE, ADC_SEQ);                     // disable while configuring
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQ, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ, 0, ADC_CTL_CH0);      // right
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ, 1, ADC_CTL_CH0);      // right
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ, 2, ADC_CTL_CH1);      // front
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ, 3, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1);  // front, last step, interrupt
    ADCSequenceEnable(ADC0_BASE, ADC_SEQ);

    ADCIntClear(ADC0_BASE, ADC_SEQ);
    ADCIntEnable(ADC0_BASE, ADC_SEQ);
    IntEnable(INT_ADC0SS1);
}

// W5C2 slide 19: ADC value -> distance in cm (negative results clamped to 0)
float
AdcToCm(uint32_t ui32Adc)
{
    float x = (float)ui32Adc;
    float d = -1.858203e-9f * x * x * x + 1.3214630459e-5f * x * x
              - 0.03356491643966f * x + 35.536853696418802f;
    return (d < 0.0f) ? 0.0f : d;
}

//*****************************************************************************
// [LED] Task 7.1
// LAB 5 -> LAB 6 EDIT: the color now shows how far the wheel is from the
// 10 cm target (|error|), not the raw distance to the wall.
//*****************************************************************************
void
SetLed(float fError)
{
    float fAbs = (fError < 0.0f) ? -fError : fError;

    if(fAbs <= GREEN_BAND_CM)       GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, LED_GREEN);
    else if(fAbs <= YELLOW_BAND_CM) GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, LED_YELLOW);
    else                            GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, LED_RED);
}

//*****************************************************************************
// [ISR] ADC0 SS1 -- sensor data ready (task 7)
// LAB 5 -> LAB 6 EDIT: reads both sensors, converts to the right wheel
// distance, then (while running) sets the LED and runs the PID.
//*****************************************************************************
void
ADC0Seq1IntHandler(void)
{
    uint32_t ui32Value[ADC_NUM_SAMPLES];
    float    fWheel, fErr, fAdj;

    ADCIntClear(ADC0_BASE, ADC_SEQ);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQ, ui32Value);

    // Average the 2 samples of each sensor, convert to cm
    RightDistanceCm = AdcToCm((ui32Value[0] + ui32Value[1]) / 2);
    FrontDistanceCm = AdcToCm((ui32Value[2] + ui32Value[3]) / 2);

    // 7.1: distance from the right WHEEL to the wall
    fWheel = RightDistanceCm - SENSOR_TO_WHEEL_CM;
    if(fWheel < 0.0f) fWheel = 0.0f;
    RightWheelCm = fWheel;

    if(!Running) return;                // stopped: leave LED and motors off

    fErr = TARGET_CM - fWheel;          // > 0 too close, < 0 too far
    SetLed(fErr);

    // 7.2: PID (W6C1 slide 14)
    fAdj = (KP * fErr) + (KD * (fErr - ErrPrev)) + (KI * ErrSum);

    ErrSum += fErr;
    if(ErrSum >  I_SUM_LIMIT) ErrSum =  I_SUM_LIMIT;    // anti-windup
    if(ErrSum < -I_SUM_LIMIT) ErrSum = -I_SUM_LIMIT;
    ErrPrev = fErr;

    if(fAdj >  MAX_ADJUST_PCT) fAdj =  MAX_ADJUST_PCT;
    if(fAdj < -MAX_ADJUST_PCT) fAdj = -MAX_ADJUST_PCT;

    // Add to one wheel, subtract from the other:
    //   too close -> left slower -> turn away from the wall
    //   too far   -> right slower -> turn toward the wall
    // At 99% base speed the faster wheel stays clamped at 99%.
    SetMotors(BASE_DUTY_PCT - fAdj, BASE_DUTY_PCT + fAdj);
}

//*****************************************************************************
// [COMMANDS] Lab 3 command interpreter
// LAB 5 -> LAB 6 EDIT: FWD/BCK/LFT/RGT removed, STR added, STP now stops
// the robot, DST now prints the wheel distance.
//*****************************************************************************

// STR: reset the PID, motors at full speed (task 3), start the timer (task 5)
void
CmdStart(void)
{
    ErrSum  = 0.0f;
    ErrPrev = 0.0f;
    Running = true;
    SetMotors(BASE_DUTY_PCT, BASE_DUTY_PCT);
    TimerEnable(TIMER2_BASE, TIMER_A);
    UARTSendBoth("\r\nSTR: Start -- following the right wall\r\n");
    
}

// STP: timer off, motors off, LED off
void
CmdStop(void)
{
    TimerDisable(TIMER2_BASE, TIMER_A);
    Running = false;
    SetMotors(0.0f, 0.0f);
    GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, 0);
    UARTSendBoth("\r\nSTP: Stop the robot\r\n");
}

// DST: print the right wheel distance in whole cm.
// Note: readings only happen while the timer runs (after STR).
void
CmdDistance(void)
{
    char     Msg[48] = "\r\nDST: Wheel = ";
    char     Num[6];
    uint32_t v = (uint32_t)(RightWheelCm + 0.5f);   // round to nearest cm
    int      n = 0, len = strlen(Msg);

    // Number -> text: digits come out in reverse, then copied back in order
    do { Num[n++] = '0' + (v % 10); v /= 10; } while(v > 0);
    while(n > 0) Msg[len++] = Num[--n];
    Msg[len] = '\0';
    strcat(Msg, " cm\r\n");
    UARTSendBoth(Msg);
}

// Prototype: the table needs CmdHelp's address before it is defined
void CmdHelp(void);

const Command CmdTable[] =
{
    { "STR", CmdStart    },
    { "STP", CmdStop     },
    { "DST", CmdDistance },
    { "HLP", CmdHelp     }
};
#define NUM_COMMANDS (sizeof(CmdTable) / sizeof(CmdTable[0]))

void
CmdHelp(void)
{
    UARTSendBoth("\r\nCommands: STR = start, STP = stop, DST = wheel distance, HLP = help\r\n");
}

//*****************************************************************************
//
// [MAIN] Setup, then run Bluetooth commands forever.
//
// LAB 5 -> LAB 6 changes:
//   NEW:     PWM_init(), Timer_init()
//   CHANGED: while(1) no longer triggers the ADC or waits with SysCtlDelay();
//            Timer 2A triggers the ADC and the ADC ISR runs the LED + PID
//   CHANGED: startup prompt (Lab 5 clear-screen code removed)
//
//*****************************************************************************
int
main(void)
{
    uint32_t i;

    // FPU on before any float math (distance formula + PID in the ADC ISR)
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    // 40 MHz system clock: 16 MHz crystal -> 400 MHz PLL / 2 / 5
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // LED pins PF1-PF3 as GPIO outputs, all off until STR
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_ALL);
    GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, 0);

    // UART0 (PA0/PA1, Tera Term) and UART1 (PB0/PB1, HC-05)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    ROM_IntMasterEnable();              // processor-wide interrupt switch

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // 8-N-1: UART0 at 115200, UART1 at 9600 (HC-05 default)
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    // Receive (RX) and receive-timeout (RT) interrupts for both UARTs
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    ADC_init();         // task 4: right + front sensors

    // ===================== LAB 6 ADDITION =====================
    PWM_init();         // task 2: motor PWM, outputs off until STR
    Timer_init();       // task 5: 50 ms timer, started by STR
    // ===================== LAB 6 ADDITION END =================

    UARTSendBoth("Lab 6: type STR to start, STP to stop, HLP for help\r\n");

    // LAB 5 -> LAB 6 EDIT: ADC trigger + 50 ms delay removed from this loop,
    // so commands (especially STP) are handled immediately.
    while(1)
    {
        if(CmdReady)
        {
            char Cmd[CMD_LEN + 1];
            bool Found = false;

            // Copy the command with interrupts paused, converting to uppercase
            ROM_IntMasterDisable();
            for(i = 0; i < CMD_LEN; i++)
            {
                char c = CmdBuffer[i];
                Cmd[i] = ((c >= 'a') && (c <= 'z')) ? (c - 'a' + 'A') : c;
            }
            Cmd[CMD_LEN] = '\0';
            CmdReady = false;
            ROM_IntMasterEnable();

            // Find the command in the table and run its handler
            for(i = 0; i < NUM_COMMANDS; i++)
            {
                if(strncmp(Cmd, CmdTable[i].CmdName, CMD_LEN) == 0)
                {
                    CmdTable[i].pfnHandler();
                    Found = true;
                    break;
                }
            }

            if(!Found)
            {
                UARTSendBoth("\r\nUnknown command: ");
                UARTSendBoth(Cmd);
                UARTSendBoth("\r\n");
            }
        }
    }
}
