#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>

#include <stdint.h>
#include "notes.h"

// Define SMCLK frequency if not already defined
#ifndef SMCLK_FREQUENCY
#define SMCLK_FREQUENCY 48000000 // 48 MHz
#endif


const uint16_t joystick_max_value = 16362; // should be 16384

/* Graphic library context */
Graphics_Context g_sContext;

extern Graphics_Image orso8BPP_UNCOMP;

/* ADC results buffer */
static uint16_t resultsBuffer[2];
static uint16_t previusResultsBuffer[2];


/* Timer_A Compare Configuration Parameter  (PWM) */
Timer_A_CompareModeConfig compareConfig_PWM = {
        TIMER_A_CAPTURECOMPARE_REGISTER_4,          // Use CCR4
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_SET,              // Toggle output but
        0                                        // 25% Duty Cycle initially
};

// /* Timer_A Up Configuration Parameter */
// const Timer_A_UpModeConfig upConfig = {
//         TIMER_A_CLOCKSOURCE_SMCLK,               // SMCLK = 3 MhZ
//         TIMER_A_CLOCKSOURCE_DIVIDER_12,         // SMCLK/12 = 250 KhZ
//         20000,                                  // 40 ms tick period
//         TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
//         TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
//         TIMER_A_DO_CLEAR                        // Clear value
// };


void _buzzerInit() {

    // Configure GPIO pin for PWM output (P2.7 associated with TA0.4)
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    // Stop the timer during configuration
    Timer_A_stopTimer(TIMER_A0_BASE);

    // Timer_A PWM Configuration
    Timer_A_PWMConfig pwmConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,               // Use SMCLK as source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,           // Divider: 1
        0,                                       // Period will be set by playFrequency
        TIMER_A_CAPTURECOMPARE_REGISTER_4,       // Use CCR4 for PWM channel
        TIMER_A_OUTPUTMODE_RESET_SET,            // PWM output mode: reset/set
        0                                        // Duty cycle will be set by playFrequency
    };

    // Initialize PWM output (but don't start it yet)
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);




    /* Configures P2.7 to PM_TA0.4 for using Timer PWM to control the buzzer */
    // GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7,
        // GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring Timer_A0 for Up Mode and starting */
    // Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
    // Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    /* Initialize compare registers to generate PWM */
    //Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM); // For P2.7
}

void _adcInit() {
    /* Configures Pin 6.0 and 4.4 as ADC input */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8) */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
         * with internal 2.5v reference */
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
    ADC14_configureConversionMemory(ADC_MEM0,
        ADC_VREFPOS_AVCC_VREFNEG_VSS,
        ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

    ADC14_configureConversionMemory(ADC_MEM1,
        ADC_VREFPOS_AVCC_VREFNEG_VSS,
        ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

/* Enabling the interrupt when a conversion on channel 1 (end of sequence)
 *  is complete and enabling conversions */
    ADC14_enableInterrupt(ADC_INT1);

    /* Enabling Interrupts */
    Interrupt_enableInterrupt(INT_ADC14);
    Interrupt_enableMaster();

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();
}

void _graphicsInit() {
    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);

    Graphics_drawImage(&g_sContext, &orso8BPP_UNCOMP, 0, 0);
}

void _hwInit() {
    /* Halting WDT and disabling master interrupts */
    WDT_A_holdTimer();
    Interrupt_disableMaster();

    /* Set the core voltage level to VCORE1 */
    PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set 2 flash wait states for Flash bank 0 and 1*/
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializes Clock System */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    _graphicsInit();
    _adcInit();
    _buzzerInit();
}



uint16_t map_value(uint16_t value, uint16_t from_min, uint16_t from_max, uint16_t to_min, uint16_t to_max) {
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}

uint16_t percent_delta(uint16_t value, uint16_t previus_value) {
    return (value - previus_value) * 100 / joystick_max_value;
}



void playFrequency(uint32_t frequency, uint32_t durationMs) {
    uint32_t period = SMCLK_FREQUENCY / frequency;
    uint32_t dutyCycle = period / 2; // 50% duty cycle for square wave

    // Update PWM period for desired frequency
    Timer_A_PWMConfig pwmConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        period,
        TIMER_A_CAPTURECOMPARE_REGISTER_4,
        TIMER_A_OUTPUTMODE_RESET_SET,
        dutyCycle
    };
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    // Delay for the duration of the note
    __delay_cycles((SMCLK_FREQUENCY / 1000) * durationMs);

    // Stop PWM to silence the buzzer
    Timer_A_stopTimer(TIMER_A0_BASE);
}

int main(void) {
    _hwInit();


    while (1) {
        playFrequency(NOTE_C4, 500);
        playFrequency(NOTE_D4, 500);
        playFrequency(NOTE_E4, 500);
        playFrequency(NOTE_F4, 500);
        playFrequency(NOTE_G4, 500);
        playFrequency(NOTE_A4, 500);
        playFrequency(NOTE_B4, 500);
        playFrequency(NOTE_C5, 500);
        playFrequency(NOTE_C4, 500);
    }
}


/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void) {
    uint64_t status;

    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);

    /* ADC_MEM1 conversion completed */
    if (status & ADC_INT1) {
        /* Store ADC14 conversion results */
        resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
        resultsBuffer[1] = ADC14_getResult(ADC_MEM1);

        // if the delta is huge, redraw the image. This is to avoid flickering5
        if (percent_delta(resultsBuffer[0], previusResultsBuffer[0]) > 7 || previusResultsBuffer[0] == 0 ||
            percent_delta(resultsBuffer[1], previusResultsBuffer[1]) > 7 || previusResultsBuffer[1] == 0) {
            previusResultsBuffer[0] = resultsBuffer[0];
            previusResultsBuffer[1] = resultsBuffer[1];


            // draw sight (circle + cross)

            // map the joystick values to the screen
            uint16_t x_offset = map_value(resultsBuffer[0], 0, joystick_max_value, 0, 128);
            uint16_t y_offset = 128 - map_value(resultsBuffer[1], 0, joystick_max_value, 0, 128); // invert y axis

            // redraw the image
            Graphics_drawImage(&g_sContext, &orso8BPP_UNCOMP, 0, 0);

            // draw cross
            // horizontal
            Graphics_drawLineH(&g_sContext, x_offset - 20, x_offset + 20, y_offset - 1);
            Graphics_drawLineH(&g_sContext, x_offset - 20, x_offset + 20, y_offset);
            Graphics_drawLineH(&g_sContext, x_offset - 20, x_offset + 20, y_offset + 1);

            // vertical
            Graphics_drawLineV(&g_sContext, x_offset - 1, y_offset - 20, y_offset + 20);
            Graphics_drawLineV(&g_sContext, x_offset, y_offset - 20, y_offset + 20);
            Graphics_drawLineV(&g_sContext, x_offset + 1, y_offset - 20, y_offset + 20);

            // draw circle
            Graphics_drawCircle(&g_sContext, x_offset, y_offset, 15);
            Graphics_drawCircle(&g_sContext, x_offset, y_offset, 16);

        }
    }
}