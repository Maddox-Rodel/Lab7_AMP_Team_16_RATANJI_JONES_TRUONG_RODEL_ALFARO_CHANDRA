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
// LAB 7 -- Maze navigation: PID wall following + right turn + U-turn
//
//   Timer 2A ISR (every 30 ms, always on): trigger the FRONT sensor (ADC0 SS2)
//   FRONT ISR (SS2): stopped -> just read the right sensor (for DST/LIV)
//                    front < 8 cm  -> stop, U-turn status = 1, U-turn
//                    U-turning and front < 14 cm -> keep U-turning
//                    U-turning and front >= 14 cm -> forward, status = 0
//                    otherwise -> trigger the RIGHT sensor (ADC0 SS1)
//   RIGHT ISR (SS1): wheel-to-wall > 15 cm (2 readings in a row) -> Right Turn
//                    else -> PID
//   STR: motors forward at base speed, Running = true
//   STP: Running = false, motors off, LED off
//   LIV: live readout of both sensors + mode, on/off
//
// SMOOTH: changes for straighter, gentler wall following
//   1. PID gains tuned for this robot + capped correction
//   2. Deadband: tiny errors (sensor noise) are ignored
//   3. Ramp: the correction changes at most MAX_STEP_PCT per reading
//   4. Low-pass filter on the right wheel distance (removes sensor jitter)
//   5. PidReset() clears all PID state after STR, a U-turn or a right turn
//   6. Intersection needs 2 far readings in a row (one noisy spike won't turn)
//   7. Motor trim + TEST_MODE 3 to make the robot drive straight on its own
//
// LIVE: LIV command prints Wheel / Right / Front / Mode every 200 ms
//
// TUNE: changes after the first LIV log (robot weaved like a snake)
//   1. Sensor is at the wheel edge -> SENSOR_TO_WHEEL_CM = 0
//   2. TARGET_CM = 10 (sensor's reliable range starts at ~10 cm)
//   3. Stronger P (KP 3), bigger max correction (20%), faster ramp (4%)
//   4. D now compares with the error 150 ms ago (5 readings), so it can
//      actually damp the weave; KD = 8
//   5. FILTER_ALPHA 0.6 for a little less lag
//
//*****************************************************************************
#include "driverlib/pwm.h"      // PWMGenConfigure(), PWMPulseWidthSet(), PWMOutputState()
#include "driverlib/timer.h"    // TimerConfigure(), TimerLoadSet(), TimerEnable()

//*****************************************************************************
// [TEST] Steps 3 and 4: 1 = Right Turn alone (drives in a circle),
// 2 = U-Turn alone (spins in place), 3 = straight line (motor trim test).
// Don't type STR while testing. Set back to 0 for the maze.
//*****************************************************************************
#define TEST_MODE           0

//*****************************************************************************
// [ADC] Distance sensors -- each sensor has its own sequencer so the front
// reading can decide whether to read the right one.
//   FRONT sensor -> PE2 = AIN1 -> SS2 (triggered by Timer 2A)
//   RIGHT sensor -> PE3 = AIN0 -> SS1 (triggered by the front ISR)
//*****************************************************************************
#define ADC_SEQ_RIGHT       1       // sample sequencer 1
#define ADC_SEQ_FRONT       2       // sample sequencer 2
#define SAMPLES_PER_SENSOR  2       // 2 samples per sensor, averaged
#define SAMPLE_PERIOD_MS    30      // 30 ms timer (sensor updates every ~38 ms, don't go lower)

// LIVE: print rate for the LIV command (Bluetooth at 9600 baud handles ~5 lines/s easily)
#define STREAM_PERIOD_MS    200
#define STREAM_TICKS        (STREAM_PERIOD_MS / SAMPLE_PERIOD_MS)   // timer ticks between prints

//*****************************************************************************
// [WALL] Distances (all in cm) -- tune these in the maze
//   wheel distance = right sensor distance - SENSOR_TO_WHEEL_CM
//*****************************************************************************
#define TARGET_CM           10.0f   // TUNE: right wheel 10 cm from the wall (was 4)
#define SENSOR_TO_WHEEL_CM  0.0f    // TUNE: sensor is at the edge of the wheel (was 4)
#define INTERSECTION_CM     15.0f   // wheel-to-wall above this = intersection -> right turn
#define INTERSECTION_COUNT  2       // far readings in a row needed before turning
#define FRONT_STOP_CM       8.0f    // front below this -> start U-turn
#define UTURN_EXIT_CM       14.0f   // U-turn ends once front reaches this

//*****************************************************************************
// [PID] Gains. Errors are in cm, corrections in % duty.
//   Still snaking slowly  -> raise KD (try 12)
//   Fast wiggle / jerky   -> lower KD (try 5) or FILTER_ALPHA (try 0.4)
//   Sits 1-2 cm off target -> set motor trim, or KI 0.05
//*****************************************************************************
#define KP                  3.0f    // TUNE: was 1.5 -- % duty per cm of error
#define KI                  0.0f
#define KD                  8.0f    // TUNE: % duty per cm of change over D_WINDOW readings
#define D_WINDOW            5       // TUNE: D compares with the reading 5 x 30 = 150 ms ago
#define I_SUM_LIMIT         50.0f   // caps the error sum to avoid integral windup
#define MAX_ADJUST_PCT      20.0f   // TUNE: was 12 -- largest correction (% duty)
#define DEADBAND_CM         0.5f    // errors smaller than this count as 0 (P/I only)
#define MAX_STEP_PCT        4.0f    // TUNE: was 2 -- correction can change this much per reading
#define FILTER_ALPHA        0.6f    // TUNE: was 0.5 -- 1.0 = no filter, smaller = smoother but slower

//*****************************************************************************
// [MOTOR] PWM
//   PB6 = M0PWM0 -> DRV8835 AENBL  (left speed)
//   PB7 = M0PWM1 -> DRV8835 BENBL  (right speed)
//   PA2          -> DRV8835 APHASE (left direction)
//   PA3          -> DRV8835 BPHASE (right direction)
//*****************************************************************************
#define PWM_FREQ_HZ         10000   // 40 MHz / 10 kHz = 4000 clocks per period
#define BASE_DUTY_PCT       50.0f   // default forward speed
#define MAX_DUTY_PCT        99.0f   // pulse width must be less than the period
#define MIN_DUTY_PCT        0.0f

// Motor trim. If the robot curves in TEST_MODE 3, speed up the slower
// side (curves right -> right motor slow -> RIGHT_TRIM_PCT 2.0f, etc.)
#define LEFT_TRIM_PCT       0.0f
#define RIGHT_TRIM_PCT      0.0f

// Turn speeds -- tune so the turns fit the maze
#define RTURN_LEFT_PCT      50.0f   // right turn: left (outer) wheel fast
#define RTURN_RIGHT_PCT     15.0f   //             right (inner) wheel slow
#define UTURN_PCT           50.0f   // U-turn: both wheels, opposite directions

#define LEFT_PHASE_FWD      0       // PA2 level for forward
#define RIGHT_PHASE_FWD     1       // PA3 level for forward (right motor is mirrored)

// [LED] PF1 = red, PF2 = blue, PF3 = green
#define LED_ALL     (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
#define LED_RED     (GPIO_PIN_1)
#define LED_BLUE    (GPIO_PIN_2)
#define LED_GREEN   (GPIO_PIN_3)

//*****************************************************************************
// [GLOBALS] volatile because ISRs write them and other code reads them
//*****************************************************************************
volatile float RightDistanceCm = 0.0f;  // right sensor to wall
volatile float RightWheelCm    = 0.0f;  // right wheel to wall (raw, shown by DST/LIV)
volatile float FrontDistanceCm = 0.0f;  // front sensor to wall
volatile bool  Running = false;         // true after STR, false after STP
volatile float ErrSum  = 0.0f;          // PID: sum of past errors (I term)
volatile float ErrPrev = 0.0f;          // PID: previous error
volatile float FiltWheel = 0.0f;        // filtered wheel distance used by the PID
volatile float AdjPrev   = 0.0f;        // last correction (for the ramp)
volatile bool  PidFirst  = true;        // next PID call seeds the filter + history
volatile float   ErrHist[D_WINDOW];     // TUNE: past errors, for the D term
volatile uint8_t HistIdx = 0;           // TUNE: next slot in ErrHist
volatile uint8_t FarCount        = 0;   // far readings in a row (intersection check)
volatile uint8_t UTurnStatus     = 0;   // 1 while U-turning
volatile uint8_t RightTurnStatus = 0;   // 1 while turning right
volatile bool     Streaming = false;    // LIVE: true while LIV printing is on
volatile uint32_t TickCount = 0;        // LIVE: counts timer ticks, paces the printing
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
    uint32_t ui32Status = ROM_UARTIntStatus(UART0_BASE, true);
    ROM_UARTIntClear(UART0_BASE, ui32Status);

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

    // % duty -> pulse width in PWM clocks (e.g. 50% of 4000 = 2000)
    uint32_t ui32Width = (uint32_t)((float)PWMPeriod * fPct / 100.0f);
    if(ui32Width < 1) ui32Width = 1;    // width 0 is not allowed

    PWMPulseWidthSet(PWM0_BASE, ui32Out, ui32Width);
    PWMOutputState(PWM0_BASE, ui32Bit, true);
}

// Set both motors -- left = PB6 (PWM_OUT_0), right = PB7 (PWM_OUT_1).
// Trim is added only when a wheel is moving, so 0% still means off.
void
SetMotors(float fLeftPct, float fRightPct)
{
    if(fLeftPct  > 0.0f) fLeftPct  += LEFT_TRIM_PCT;
    if(fRightPct > 0.0f) fRightPct += RIGHT_TRIM_PCT;
    SetMotorDuty(PWM_OUT_0, PWM_OUT_0_BIT, fLeftPct);
    SetMotorDuty(PWM_OUT_1, PWM_OUT_1_BIT, fRightPct);
}

//*****************************************************************************
// [MOTOR] Set each wheel's direction: true = forward, false = reverse.
// Handles the mirrored right motor so callers just think forward/reverse.
//*****************************************************************************
void
SetDirection(bool bLeftFwd, bool bRightFwd)
{
    uint8_t ui8Left  = bLeftFwd  ? LEFT_PHASE_FWD  : !LEFT_PHASE_FWD;
    uint8_t ui8Right = bRightFwd ? RIGHT_PHASE_FWD : !RIGHT_PHASE_FWD;

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3,
                 (ui8Left  ? GPIO_PIN_2 : 0) |
                 (ui8Right ? GPIO_PIN_3 : 0));
}

//*****************************************************************************
// [MOTOR] PWM_init: motor driver pins. Outputs stay OFF until STR.
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
    SetDirection(true, true);

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
// [TIMER] Timer_init: Timer 2A, periodic, SAMPLE_PERIOD_MS.
// Load = 40 MHz / 1000 x 30 - 1 = 1,199,999. Enabled in main and left on.
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
// [ISR] Timer 2A: trigger the FRONT sensor. The front ISR decides whether
// the right sensor gets read.
//*****************************************************************************
void
Timer2AIntHandler(void)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TickCount++;                        // LIVE: main() uses this to time the printouts
    ADCProcessorTrigger(ADC0_BASE, ADC_SEQ_FRONT);
}

//*****************************************************************************
// [ADC] ADC_init
//   SS2 = front (CH1), 2 samples, triggered by the timer
//   SS1 = right (CH0), 2 samples, triggered by the front ISR
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

    // Front sensor: SS2, priority 0
    ADCSequenceDisable(ADC0_BASE, ADC_SEQ_FRONT);
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQ_FRONT, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ_FRONT, 0, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ_FRONT, 1, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1);
    ADCSequenceEnable(ADC0_BASE, ADC_SEQ_FRONT);
    ADCIntClear(ADC0_BASE, ADC_SEQ_FRONT);
    ADCIntEnable(ADC0_BASE, ADC_SEQ_FRONT);
    IntEnable(INT_ADC0SS2);

    // Right sensor: SS1, priority 1
    ADCSequenceDisable(ADC0_BASE, ADC_SEQ_RIGHT);
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQ_RIGHT, ADC_TRIGGER_PROCESSOR, 1);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ_RIGHT, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ_RIGHT, 1, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    ADCSequenceEnable(ADC0_BASE, ADC_SEQ_RIGHT);
    ADCIntClear(ADC0_BASE, ADC_SEQ_RIGHT);
    ADCIntEnable(ADC0_BASE, ADC_SEQ_RIGHT);
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
// PidReset -- clear all PID memory so the next PID call starts fresh.
// Called on STR, at the end of a U-turn and at the end of a right turn, so
// old errors from before the turn don't cause a swerve afterwards.
// (PidFirst also refills the D history on the next PID call.)
//*****************************************************************************
void
PidReset(void)
{
    ErrSum   = 0.0f;
    ErrPrev  = 0.0f;
    AdjPrev  = 0.0f;
    PidFirst = true;
}

//*****************************************************************************
// [PID] Step 2: all PID code moved here from the Lab 6 ADC ISR. GREEN LED.
// filter -> P (with deadband) + D (over 150 ms) -> clamp -> ramp -> motors
//*****************************************************************************
void
PID(float fWheel)
{
    float   fRaw, fErr, fOld, fAdj;
    uint8_t k;

    GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, LED_GREEN);

    // Low-pass filter: each reading moves FiltWheel only part of the way
    // toward the new value, so single noisy readings barely matter.
    if(PidFirst) FiltWheel = fWheel;    // first call after a reset: start at the real value
    else         FiltWheel += FILTER_ALPHA * (fWheel - FiltWheel);

    fRaw = TARGET_CM - FiltWheel;       // > 0 too close, < 0 too far

    // TUNE: first call after a reset -- fill the history so D starts at 0
    if(PidFirst)
    {
        for(k = 0; k < D_WINDOW; k++) ErrHist[k] = fRaw;
        HistIdx  = 0;
        PidFirst = false;
    }

    // TUNE: D compares with the error from D_WINDOW readings ago (150 ms),
    // so slow drifts toward/away from the wall are big enough to matter.
    // Uses the error before the deadband, so D stays smooth.
    fOld = ErrHist[HistIdx];
    ErrHist[HistIdx] = fRaw;
    HistIdx = (HistIdx + 1) % D_WINDOW;

    // Deadband (P and I only): close enough counts as perfect
    fErr = fRaw;
    if((fErr < DEADBAND_CM) && (fErr > -DEADBAND_CM)) fErr = 0.0f;

    fAdj = (KP * fErr) + (KD * (fRaw - fOld)) + (KI * ErrSum);

    ErrSum += fErr;
    if(ErrSum >  I_SUM_LIMIT) ErrSum =  I_SUM_LIMIT;    // anti-windup
    if(ErrSum < -I_SUM_LIMIT) ErrSum = -I_SUM_LIMIT;
    ErrPrev = fErr;

    if(fAdj >  MAX_ADJUST_PCT) fAdj =  MAX_ADJUST_PCT;
    if(fAdj < -MAX_ADJUST_PCT) fAdj = -MAX_ADJUST_PCT;

    // Ramp: the correction can only change MAX_STEP_PCT per reading
    if(fAdj > AdjPrev + MAX_STEP_PCT) fAdj = AdjPrev + MAX_STEP_PCT;
    if(fAdj < AdjPrev - MAX_STEP_PCT) fAdj = AdjPrev - MAX_STEP_PCT;
    AdjPrev = fAdj;

    // too close -> left slower -> turn away from the wall
    // too far   -> right slower -> turn toward the wall
    SetMotors(BASE_DUTY_PCT - fAdj, BASE_DUTY_PCT + fAdj);
}

//*****************************************************************************
// [TURN] Step 3: Right Turn -- both wheels forward, left faster. BLUE LED.
// Tune RTURN_LEFT_PCT / RTURN_RIGHT_PCT so the arc fits the maze.
//*****************************************************************************
void
RightTurn(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, LED_BLUE);
    SetDirection(true, true);
    SetMotors(RTURN_LEFT_PCT, RTURN_RIGHT_PCT);
}

//*****************************************************************************
// [TURN] Step 4: U-Turn -- spin in place, left wheel reverse, right wheel
// forward (turns left). RED LED. Only sets direction + speed; does NOT check
// the front distance. Swap the two booleans to spin the other way.
//*****************************************************************************
void
UTurn(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, LED_RED);
    SetDirection(false, true);
    SetMotors(UTURN_PCT, UTURN_PCT);
}

//*****************************************************************************
// [ISR] Step 5: ADC0 SS2 -- FRONT sensor ready (every 30 ms from Timer 2A)
//*****************************************************************************
void
ADC0Seq2IntHandler(void)
{
    uint32_t ui32Value[4];

    ADCIntClear(ADC0_BASE, ADC_SEQ_FRONT);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQ_FRONT, ui32Value);

    // Calculate the front distance
    FrontDistanceCm = AdcToCm((ui32Value[0] + ui32Value[1]) / SAMPLES_PER_SENSOR);

    if(!Running)
    {
        ADCProcessorTrigger(ADC0_BASE, ADC_SEQ_RIGHT);  // stopped: still read the right sensor for DST/LIV
        return;
    }

    if(FrontDistanceCm < FRONT_STOP_CM)
    {
        // Wall ahead: stop, set U-turn status, start the U-turn
        if(UTurnStatus == 0) SetMotors(0.0f, 0.0f);
        UTurnStatus = 1;
        UTurn();
    }
    else if((UTurnStatus == 1) && (FrontDistanceCm < UTURN_EXIT_CM))
    {
        // Still facing the wall: keep spinning
        UTurn();
    }
    else if(UTurnStatus == 1)
    {
        // U-turn finished: both wheels forward at the default speed
        SetDirection(true, true);
        SetMotors(BASE_DUTY_PCT, BASE_DUTY_PCT);
        UTurnStatus     = 0;
        RightTurnStatus = 0;
        FarCount        = 0;
        PidReset();                     // fresh PID after the U-turn
    }
    else
    {
        // Path ahead is clear: read the right sensor
        ADCProcessorTrigger(ADC0_BASE, ADC_SEQ_RIGHT);
    }
}

//*****************************************************************************
// [ISR] ADC0 SS1 -- RIGHT sensor ready (triggered by the front ISR)
// Only calculates the distance and picks Right Turn or PID.
//*****************************************************************************
void
ADC0Seq1IntHandler(void)
{
    uint32_t ui32Value[4];
    float    fWheel;

    ADCIntClear(ADC0_BASE, ADC_SEQ_RIGHT);
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQ_RIGHT, ui32Value);

    // Calculate the right side distance, then wheel-to-wall
    RightDistanceCm = AdcToCm((ui32Value[0] + ui32Value[1]) / SAMPLES_PER_SENSOR);
    fWheel = RightDistanceCm - SENSOR_TO_WHEEL_CM;
    if(fWheel < 0.0f) fWheel = 0.0f;
    RightWheelCm = fWheel;

    if(!Running || UTurnStatus) return;

    // Count far readings in a row, so one noisy spike can't start a turn
    if(fWheel > INTERSECTION_CM)
    {
        if(FarCount < 255) FarCount++;
    }
    else
    {
        FarCount = 0;
    }

    if(FarCount >= INTERSECTION_COUNT)
    {
        // No wall on the right: intersection -> turn right
        RightTurnStatus = 1;
        RightTurn();
    }
    else
    {
        // Wall found again after a turn: restart the PID cleanly
        if(RightTurnStatus)
        {
            RightTurnStatus = 0;
            PidReset();
        }
        PID(fWheel);
    }
}

//*****************************************************************************
// [COMMANDS] Lab 3 command interpreter
//*****************************************************************************

// STR: reset PID + turn flags, both wheels forward, start driving
void
CmdStart(void)
{
    PidReset();
    UTurnStatus     = 0;
    RightTurnStatus = 0;
    FarCount        = 0;
    SetDirection(true, true);
    SetMotors(BASE_DUTY_PCT, BASE_DUTY_PCT);
    Running = true;
    UARTSendBoth("\r\nSTR: Start -- navigating the maze\r\n");
}

// STP: clear Running first so a pending ADC ISR can't restart the motors.
// The timer keeps running so DST/LIV still show live readings.
void
CmdStop(void)
{
    Running = false;
    SetMotors(0.0f, 0.0f);
    SetDirection(true, true);
    UTurnStatus     = 0;
    RightTurnStatus = 0;
    FarCount        = 0;
    GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, 0);
    UARTSendBoth("\r\nSTP: Stop the robot\r\n");
}

// Add a whole number to the end of a string (rounded to nearest)
void
AppendCm(char *Msg, float fCm)
{
    char     Num[6];
    uint32_t v = (uint32_t)(fCm + 0.5f);
    int      n = 0, len = strlen(Msg);

    // Digits come out in reverse, then copied back in order
    do { Num[n++] = '0' + (v % 10); v /= 10; } while(v > 0);
    while(n > 0) Msg[len++] = Num[--n];
    Msg[len] = '\0';
}

// DST: print the right wheel and front distances in whole cm (one time)
void
CmdDistance(void)
{
    char Msg[64] = "\r\nDST: Wheel = ";

    AppendCm(Msg, RightWheelCm);
    strcat(Msg, " cm, Front = ");
    AppendCm(Msg, FrontDistanceCm);
    strcat(Msg, " cm\r\n");
    UARTSendBoth(Msg);
}

//*****************************************************************************
// LIVE: print one line with both sensors and the current mode.
// Called from main() only (UARTSendBoth blocks, so never from an ISR).
//   Wheel = right wheel to wall, Right = right sensor to wall,
//   Front = front sensor to wall, Mode = STOP / PID / RTURN / UTURN
//*****************************************************************************
void
PrintLive(void)
{
    char Msg[96] = "\r\nLIVE: Wheel = ";

    AppendCm(Msg, RightWheelCm);
    strcat(Msg, " cm, Right = ");
    AppendCm(Msg, RightDistanceCm);
    strcat(Msg, " cm, Front = ");
    AppendCm(Msg, FrontDistanceCm);
    strcat(Msg, " cm, Mode = ");
 
    if(!Running)              strcat(Msg, "STOP");
    else if(UTurnStatus)      strcat(Msg, "UTURN");
    else if(RightTurnStatus)  strcat(Msg, "RTURN");
    else                      strcat(Msg, "PID");

    UARTSendBoth(Msg);
}

// LIVE: LIV turns the live readout on/off
void
CmdLive(void)
{
    Streaming = !Streaming;
    TickCount = 0;
    UARTSendBoth(Streaming ? "\r\nLIV: live readout ON (type LIV to stop)\r\n"
                           : "\r\nLIV: live readout OFF\r\n");
}

// Prototype: the table needs CmdHelp's address before it is defined
void CmdHelp(void);

const Command CmdTable[] =
{
    { "STR", CmdStart    },
    { "STP", CmdStop     },
    { "DST", CmdDistance },
    { "LIV", CmdLive     },
    { "HLP", CmdHelp     }
};
#define NUM_COMMANDS (sizeof(CmdTable) / sizeof(CmdTable[0]))

void
CmdHelp(void)
{
    UARTSendBoth("\r\nCommands: STR = start, STP = stop, DST = distances, LIV = live readout on/off, HLP = help\r\n");
}

//*****************************************************************************
//
// [MAIN] Setup, then run Bluetooth commands forever.
//
//*****************************************************************************
int
main(void)
{
    uint32_t i;

    // FPU on before any float math (distance formula + PID in the ADC ISRs)
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

    ADC_init();         // front (SS2) + right (SS1) sensors
    PWM_init();         // motor PWM, outputs off until STR
    Timer_init();       // 30 ms timer

#if TEST_MODE == 0
    TimerEnable(TIMER2_BASE, TIMER_A);  // sensors always update; STR/STP only start/stop the motors
#endif

    UARTSendBoth("Lab 7: STR = start, STP = stop, DST = distances, LIV = live readout, HLP = help\r\n");

    while(1)
    {
        // ===================== TEST HOOK (steps 3, 4 + trim) =====================
#if TEST_MODE == 1
        RightTurn();                    // should drive in a circle (open area)
#elif TEST_MODE == 2
        UTurn();                        // should spin in place
#elif TEST_MODE == 3
        SetDirection(true, true);       // both wheels same duty -> should go straight
        SetMotors(BASE_DUTY_PCT, BASE_DUTY_PCT);
#endif
        // =========================================================================

        // LIVE: print a readout every STREAM_PERIOD_MS while LIV is on
        if(Streaming && (TickCount >= STREAM_TICKS))
        {
            TickCount = 0;
            PrintLive();
        }

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
