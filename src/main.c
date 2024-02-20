#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>

#include <stdint.h>
#include "notes.h"
#include "song.h"
#include "player.h"

// External resources
extern Song hedwigsTheme; // Declare external reference to the song defined elsewhere
extern Graphics_Image orso8BPP_UNCOMP;

// ADC results buffer
static uint16_t resultsBuffer[2];
static uint16_t previusResultsBuffer[2];

// Graphic library  context
Graphics_Context g_sContext;

// Macros
#define map(x, in_min, in_max, out_min, out_max) \
    ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define delta_percent(value, previus_value) \
    ((value - previus_value) * 100 / JOYSTICK_MAX_VALUE)


// Begin Servo configuration values

// SERVO1 (HORIZONTAL) 2.5
#define SERVO1_MAX 1800
#define SERVO1_MIN 450
#define SERVO1_MID 1100

#define SERVO1_MOVE 100

int servo1Position = SERVO1_MID;

// SERVO2 (VERTICAL) 2.4
// SERVO3 (TRIGGER) 2.6
#define SERVOC_DOWN 1000
#define SERVOC_STOP 1150
#define SERVOC_UP 1200

int servo2Direction = SERVOC_STOP;
int servo3Direction = SERVOC_STOP;

// End Servo Configuration values

// Trigger Timer values
#define TRIGGER_TIMER_MAX_CYCLES 8
uint8_t triggerTimerCycles = 0;

// GameMode switch
uint8_t turretAutomaticMode = false;

// Joystick constants
#define JOYSTICK_TRESHOLD 2500
#define JOYSTICK_CENTER 8192
#define JOYSTICK_MAX_VALUE 16362 // should be 16384

// Timer_A Compare Configuration Parameter  (PWM) for SERVOs
Timer_A_CompareModeConfig compareConfig_PWM = {
    TIMER_A_CAPTURECOMPARE_REGISTER_1,        // Use CCR1
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE, // Disable CCR interrupt
    TIMER_A_OUTPUTMODE_RESET_SET,             // Toggle output but
    SERVO1_MID                                // 1.5 ms pulse width
};

// Timer_A Up Configuration Parameter for SERVOs
Timer_A_UpModeConfig upConfig = {
    TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK = 48 MhZ
    TIMER_A_CLOCKSOURCE_DIVIDER_64,      // SMCLK/64 = 750 KhZ
    15000,                               // 0.02 s * 750 KhZ = 15000 tick period
    TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE, // Disable CCR0 interrupt
    TIMER_A_DO_CLEAR                     // Clear value
};

void _servoInit() {
    // Configures P2.5 to PM_TA0.2 for using Timer PWM to control Servo1
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configures P2.4 to PM_TA0.1 for using Timer PWM to control Servo2
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configures P2.6 to PM_TA0.3 for using Timer PWM to control Servo3
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configuring Timer_A0 for Up Mode and starting
    Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    // Initialize compare registers to generate PWM  for the Servo2 Port
    compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1; // For P2.4
    compareConfig_PWM.compareValue = SERVOC_STOP;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM); // For P2.4

    // Initialize compare registers to generate PWM  for the Servo1 Port 2.5
    compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2; // For P2.5
    compareConfig_PWM.compareValue = SERVO1_MID;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM); // For P2.5

    // Initialize compare registers to generate PWM  for the Servo3 Port 2.6
    compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3; // For P2.6
    compareConfig_PWM.compareValue = SERVOC_STOP;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM); // For P2.6

    // Configure Tmer_A1 for the time measurement of the trigger
    upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; // Enable CCR0 interrupt
    upConfig.timerPeriod *= 200;                                                           // Extend the period
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);


    /// TODO: check this
    // Interrupt_enableSleepOnIsrExit();
    Interrupt_enableInterrupt(INT_TA1_0);
}

void _adcInit() {
    // Configures Pin 6.0 and 4.4 as ADC input for Joystick
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    // Initializing ADC (ADCOSC/64/8)
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

    // Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat) with internal 2.5v reference
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
    ADC14_configureConversionMemory(ADC_MEM0,
        ADC_VREFPOS_AVCC_VREFNEG_VSS,
        ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

    ADC14_configureConversionMemory(ADC_MEM1,
        ADC_VREFPOS_AVCC_VREFNEG_VSS,
        ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

    // Enabling the interrupt when a conversion on channel 1 (end of sequence) is complete and enabling conversions
    ADC14_enableInterrupt(ADC_INT1);

    // Enabling Interrupts for ADC
    Interrupt_enableInterrupt(INT_ADC14);
    Interrupt_enableMaster();

    // Setting up the sample timer to automatically step through the sequence convert.
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    // Triggering the start of the sample
    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();
}

void _graphicsInit() {
    // Initializes display
    Crystalfontz128x128_Init();

    // Set default screen orientation
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    // Initializes graphics context
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);

    Graphics_drawImage(&g_sContext, &orso8BPP_UNCOMP, 0, 0);
}

void _hwInit() {
    // Halting WDT and disabling master interrupts 
    WDT_A_holdTimer();
    Interrupt_disableMaster();

    // Set the core voltage level to VCORE1 
    PCM_setCoreVoltageLevel(PCM_VCORE1);

    // Set 2 flash wait states for Flash bank 0 and 1
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);

    // Initializes Clock System
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    _graphicsInit();
    _adcInit();
    _servoInit();
    _toneInit();
}

// Function to move the servo1 to the desired position
void moveServo1(void) {
    compareConfig_PWM.compareValue = servo1Position;
    compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;  // For P2.5
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
}

// Function to move the servo2 to the desired direction
void moveServo2() {
    compareConfig_PWM.compareValue = servo2Direction;
    compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1; // For P2.4
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
}

// Function to press and release the trigger
void pressTrigger() {
    // Set servo3 towards the trigger
    servo3Direction = SERVOC_DOWN;
    compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3; // For P2.6
    compareConfig_PWM.compareValue = servo3Direction;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);

    // Start the timer to press and release the trigger
    triggerTimerCycles = 1;
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}

int main(void) {
    _hwInit();

    /// TODO: move this where needed (e.g. when the game starts, ends, etc.)
    // note that the MSP432 cannot play music asynchonously
    // play_song(hedwigsTheme);


    while (1) {
        PCM_gotoLPM0();
    }
}

/*
 * Timer A1_0 interrupt service routine
 */
void TA1_0_IRQHandler(void) {
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    switch (triggerTimerCycles) {
    case TRIGGER_TIMER_MAX_CYCLES: // If the timer has reached the maximum cycles release the trigger
        servo3Direction = SERVOC_UP;
        compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3; // For P2.6
        compareConfig_PWM.compareValue = servo3Direction;
        Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
        // Start the timer again
        triggerTimerCycles--;
        Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        break;
    case 0: // If the timer has reached the minimum cycles stop the trigger
        servo3Direction = SERVOC_STOP;
        compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3; // For P2.6
        compareConfig_PWM.compareValue = servo3Direction;
        Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
        break;
    default:
        // If the servo is pressing the trigger increase the triggerTimerCycles otherwise decrease it
        triggerTimerCycles = triggerTimerCycles + (servo3Direction == SERVOC_DOWN ? 1 : -1);
        Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE); // Start the timer again
        break;
    }
}

/*
 * This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer
*/
void ADC14_IRQHandler(void) {
    uint64_t status;

    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);

    // ADC_MEM1 conversion completed
    if (status & ADC_INT1) {
        // Store ADC14 conversion results
        resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
        resultsBuffer[1] = ADC14_getResult(ADC_MEM1);

        // Determine if JoyStick bottom button is pressed
        int bottomButtonPressed = 0;
        if (!(P3IN & GPIO_PIN5)) {
            bottomButtonPressed = 1;
        }

        // Determine if JoyStick top button is pressed
        int topButtonPressed = 0;
        if (!(P5IN & GPIO_PIN1)) {
            topButtonPressed = 1;
        }

        if (!turretAutomaticMode) {
            // If the joystick is moved on the x-axis change the servo1 position
            if (resultsBuffer[0] > JOYSTICK_CENTER + JOYSTICK_TRESHOLD && servo1Position > SERVO1_MIN) {
                servo1Position = servo1Position - SERVO1_MOVE;
            }
            else if (resultsBuffer[0] < JOYSTICK_CENTER - JOYSTICK_TRESHOLD && servo1Position < SERVO1_MAX) {
                servo1Position = servo1Position + SERVO1_MOVE;
            }
            moveServo1();

            // If the joystick is moved on the y-axis change the servo2 movement direction
            if (resultsBuffer[1] > JOYSTICK_CENTER + JOYSTICK_TRESHOLD) {
                servo2Direction = SERVOC_UP;
            }
            else if (resultsBuffer[1] < JOYSTICK_CENTER - JOYSTICK_TRESHOLD) {
                servo2Direction = SERVOC_DOWN;
            }
            else if (servo2Direction != SERVOC_STOP) {
                // If the joystick is not moved stop the servo2
                servo2Direction = SERVOC_STOP;
            }
            moveServo2();

            // If the trigger button is pressed and servo3 isn't moving press the trigger
            if (bottomButtonPressed && servo3Direction == SERVOC_STOP) {
                pressTrigger();
            }
        }

        // if the delta is big enough, redraw the image. This is to avoid flickering5
        if (delta_percent(resultsBuffer[0], previusResultsBuffer[0]) > 7 || previusResultsBuffer[0] == 0 ||
            delta_percent(resultsBuffer[1], previusResultsBuffer[1]) > 7 || previusResultsBuffer[1] == 0) {
            previusResultsBuffer[0] = resultsBuffer[0];
            previusResultsBuffer[1] = resultsBuffer[1];

            // draw sight (circle + cross)

            // map the joystick values to the screen
            uint16_t x_offset = map(resultsBuffer[0], 0, JOYSTICK_MAX_VALUE, 0, 128);
            uint16_t y_offset = 128 - map(resultsBuffer[1], 0, JOYSTICK_MAX_VALUE, 0, 128); // invert y axis

            // redraw the image
            Graphics_drawImage(&g_sContext, &orso8BPP_UNCOMP, 0, 0);

            // draw cross
            // horizontal lines
            Graphics_drawLineH(&g_sContext, x_offset - 20, x_offset + 20, y_offset - 1);
            Graphics_drawLineH(&g_sContext, x_offset - 20, x_offset + 20, y_offset);
            Graphics_drawLineH(&g_sContext, x_offset - 20, x_offset + 20, y_offset + 1);

            // vertical lines
            Graphics_drawLineV(&g_sContext, x_offset - 1, y_offset - 20, y_offset + 20);
            Graphics_drawLineV(&g_sContext, x_offset, y_offset - 20, y_offset + 20);
            Graphics_drawLineV(&g_sContext, x_offset + 1, y_offset - 20, y_offset + 20);

            // draw circles
            Graphics_drawCircle(&g_sContext, x_offset, y_offset, 15);
            Graphics_drawCircle(&g_sContext, x_offset, y_offset, 16);
        }
    }
}