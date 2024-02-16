#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>

/* Graphic library context */
Graphics_Context g_sContext;

/* ADC results buffer */
static uint16_t resultsBuffer[2];

#define map(x, in_min, in_max, out_min, out_max) \
    ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define SERVO1_MAX 1800
#define SERVO1_MIN 450
#define SERVO1_MID 1100

#define SERVO1_MOVE 10

int servo1Position = SERVO1_MID;


#define JOYSTICK_TRESHOLD 4000
#define JOYSTICK_CENTER 8192

/* Timer_A Compare Configuration Parameter  (PWM) */
Timer_A_CompareModeConfig compareConfig_PWM = {
    TIMER_A_CAPTURECOMPARE_REGISTER_2,        // Use CCR2
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE, // Disable CCR interrupt
    TIMER_A_OUTPUTMODE_RESET_SET,             // Toggle output but
    SERVO1_MID                                // 1.5 ms pulse width
};

/* Timer_A Up Configuration Parameter */
const Timer_A_UpModeConfig upConfig = {
    TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK = 48 MhZ
    TIMER_A_CLOCKSOURCE_DIVIDER_64,      // SMCLK/64 = 750 KhZ
    15000,                               // 0.02 s * 750 KhZ = 15000 tick period
    TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE, // Disable CCR0 interrupt
    TIMER_A_DO_CLEAR                     // Clear value
};

void _servoInit()
{
    // Configures P2.5 to PM_TA0.2 for using Timer PWM to control Servo1
    // GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configures P2.6 to PM_TA0.3 for using Timer PWM to control Servo1
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configuring Timer_A0 for Up Mode and starting
    Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    // Initialize compare registers to generate PWM  for the Servo1 Port
    compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM); // For P2.6

    // // Initialize compare registers to generate PWM  for the Servo1 Port 2.5
    // compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
    // Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM); // For P2.5
}

void _adcInit()
{
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

void _graphicsInit()
{
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

}

void _hwInit()
{
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
    _servoInit();
}


char debugPrintString[20];

/*
 * Main function
 */
int main(void)
{
    _hwInit();

    while (1)
    {
        PCM_gotoLPM0();
    }
}


/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);

    /* ADC_MEM1 conversion completed */
    if (status & ADC_INT1)
    {
        /* Store ADC14 conversion results */
        resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
        resultsBuffer[1] = ADC14_getResult(ADC_MEM1);

        sprintf(debugPrintString, "Servo: %d", servo1Position);


        Graphics_drawStringCentered(&g_sContext,
                                    debugPrintString,
                                    AUTO_STRING_LENGTH,
                                    64,
                                    30,
                                    OPAQUE_TEXT);

        if (resultsBuffer[0] > JOYSTICK_CENTER + JOYSTICK_TRESHOLD && servo1Position > SERVO1_MIN)
        {
            servo1Position = servo1Position - SERVO1_MOVE;
            compareConfig_PWM.compareValue = servo1Position;
            Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
        }
        else if (resultsBuffer[0] < JOYSTICK_CENTER - JOYSTICK_TRESHOLD && servo1Position < SERVO1_MAX)
        {
            servo1Position = servo1Position + SERVO1_MOVE;
            compareConfig_PWM.compareValue = servo1Position;
            Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
        }



        char string[10];
        sprintf(string, "X: %5d", resultsBuffer[0]);
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    8,
                                    64,
                                    50,
                                    OPAQUE_TEXT);

        sprintf(string, "Y: %5d", resultsBuffer[1]);
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    8,
                                    64,
                                    70,
                                    OPAQUE_TEXT);

        // Determine if JoyStick button is pressed
        int buttonPressed = 0;
        if (!(P4IN & GPIO_PIN1))
        {
            buttonPressed = 1;
        }

        sprintf(string, "Button: %d", buttonPressed);
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    AUTO_STRING_LENGTH,
                                    64,
                                    90,
                                    OPAQUE_TEXT);
    }
}
