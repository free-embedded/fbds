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

#define TRIGGER_TIMER_MAX_CYCLES 10
uint8_t triggerTimerCycles = 0;

int turretAutomaticMode = true;

#define JOYSTICK_TRESHOLD 2500
#define JOYSTICK_CENTER 8192

// Timer_A_1 variables
#define TA1_NO_MOVE 0
#define TA1_AUTO_MOVE 1
#define TA1_TRIGGER 2

uint8_t TA1_State = TA1_NO_MOVE;

/* Timer_A Compare Configuration Parameter  (PWM) */
Timer_A_CompareModeConfig compareConfig_PWM = {
	TIMER_A_CAPTURECOMPARE_REGISTER_1, // Use CCR1
	TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
	// Disable CCR interrupt
	TIMER_A_OUTPUTMODE_RESET_SET, // Toggle output but
	SERVO1_MID					  // 1.5 ms pulse width
};

/* Timer_A Up Configuration Parameter */
Timer_A_UpModeConfig upConfig = {
	TIMER_A_CLOCKSOURCE_SMCLK,		// SMCLK = 48 MhZ
	TIMER_A_CLOCKSOURCE_DIVIDER_64, // SMCLK/64 = 750 KhZ
	15000,							// 0.02 s * 750 KhZ = 15000 tick period
	TIMER_A_TAIE_INTERRUPT_DISABLE, // Disable Timer interrupt
	TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
	// Disable CCR0 interrupt
	TIMER_A_DO_CLEAR // Clear value
};

/* UART Config - 115200 baud rate */
/* 0 left, 1 right, 2 up, 3 down */
const eUSCI_UART_ConfigV1 uartConfig =
	{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
		26,								// BRDIV = 13
		0,								// UCxBRF = 0
		37,								// UCxBRS = 37
		EUSCI_A_UART_NO_PARITY,
		// No Parity
		EUSCI_A_UART_LSB_FIRST,						   // MSB First
		EUSCI_A_UART_ONE_STOP_BIT,					   // One stop bit
		EUSCI_A_UART_MODE,							   // UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION, // Oversampling
		EUSCI_A_UART_8_BIT_LEN						   // 8 bit data length
};

void _uartInit()
{
	/* Selecting P3.2 and P3.3 in UART mode */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
											   GPIO_PIN2 | GPIO_PIN3,
											   GPIO_PRIMARY_MODULE_FUNCTION);

	/* Setting DCO to 24MHz (upping Vcore) -> CPU operates at 24 MHz!*/
	FlashCtl_setWaitState(FLASH_BANK0, 1);
	FlashCtl_setWaitState(FLASH_BANK1, 1);
	PCM_setCoreVoltageLevel(PCM_VCORE1);
	// CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);

	/* Configuring UART Module */
	UART_initModule(EUSCI_A2_BASE, &uartConfig);

	/* Enable UART module */
	UART_enableModule(EUSCI_A2_BASE);

	/* Enabling interrupts */
	UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
	Interrupt_enableInterrupt(INT_EUSCIA2);
	Interrupt_enableSleepOnIsrExit();
}

void _servoInit(void)
{
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

	// Initialize compare registers to generate PWM  for the Servo1 Port 2.6
	compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3; // For P2.6
	compareConfig_PWM.compareValue = SERVOC_STOP;
	Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM); // For P2.6

	// Configure Tmer_A1 for the time measurement
	upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; // Enable CCR0 interrupt
	upConfig.timerPeriod *= 66;															   // 1 second
	Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);

	// Interrupt_enableSleepOnIsrExit();
	Interrupt_enableInterrupt(INT_TA1_0);
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
									ADC_INPUT_A15,
									ADC_NONDIFFERENTIAL_INPUTS);

	ADC14_configureConversionMemory(ADC_MEM1,
									ADC_VREFPOS_AVCC_VREFNEG_VSS,
									ADC_INPUT_A9,
									ADC_NONDIFFERENTIAL_INPUTS);

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
	_uartInit();
}

void moveServo1(void)
{
	compareConfig_PWM.compareValue = servo1Position;
	compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2; // For P2.5
	Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
}

void changeServo2Direction()
{
	compareConfig_PWM.compareValue = servo2Direction;
	compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1; // For P2.4
	Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
}

void pressTrigger()
{
	TA1_State = TA1_TRIGGER;
	// Servo3 is releasing the trigger
	servo3Direction = SERVOC_DOWN;
	compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3; // For P2.6
	compareConfig_PWM.compareValue = servo3Direction;
	Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);

	// Start the timer to press the trigger
	triggerTimerCycles = 1;
	Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
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

/*
 * Timer A1_0 interrupt service routine
 */
void TA1_0_IRQHandler(void)
{
	Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
	if (TA1_State == TA1_TRIGGER)
	{
		switch (triggerTimerCycles)
		{
		case TRIGGER_TIMER_MAX_CYCLES: // If the timer has reached the maximum cycles release the trigger
			servo3Direction = SERVOC_UP;
			compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3; // For P2.6
			compareConfig_PWM.compareValue = servo3Direction;
			Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
			// Start the timer again
			triggerTimerCycles--;
			Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
			break;
		case 0:
			servo3Direction = SERVOC_STOP;										   // If the timer has reached the minimum cycles stop the trigger
			compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3; // For P2.6
			compareConfig_PWM.compareValue = servo3Direction;
			Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
			TA1_State = TA1_NO_MOVE;
			break;
		default:
			// If the servo is pressing the trigger increase the triggerTimerCycles otherwise decrease it
			triggerTimerCycles = triggerTimerCycles + (servo3Direction == SERVOC_DOWN ? 1 : -1);
			// Start the timer again
			Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
			break;
		}
	}
	else if (TA1_State == TA1_AUTO_MOVE)
	{
		servo2Direction = SERVOC_STOP;
		changeServo2Direction();
		// Start the timer again
		// Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
		TA1_State = TA1_NO_MOVE;
	}
}

/*
 * This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer
 */
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

		// Determine if JoyStick bottom button is pressed
		int bottomButtonPressed = 0;
		if (!(P3IN & GPIO_PIN5))
		{
			bottomButtonPressed = 1;
		}

		// Determine if JoyStick top button is pressed
		int topButtonPressed = 0;
		if (!(P5IN & GPIO_PIN1))
		{
			turretAutomaticMode = false;
		}

		if (!turretAutomaticMode)
		{
			// If the joystick is moved on the x-axis change the servo1 position
			if (resultsBuffer[0] > JOYSTICK_CENTER + JOYSTICK_TRESHOLD && servo1Position > SERVO1_MIN)
			{
				servo1Position = servo1Position - SERVO1_MOVE;
				moveServo1();
			}
			else if (resultsBuffer[0] < JOYSTICK_CENTER - JOYSTICK_TRESHOLD && servo1Position < SERVO1_MAX)
			{
				servo1Position = servo1Position + SERVO1_MOVE;
				moveServo1();
			}

			// If the joystick is moved on the y-axis change the servo2 movement direction
			if (resultsBuffer[1] > JOYSTICK_CENTER + JOYSTICK_TRESHOLD)
			{
				servo2Direction = SERVOC_UP;
				changeServo2Direction();
			}
			else if (resultsBuffer[1] < JOYSTICK_CENTER - JOYSTICK_TRESHOLD)
			{
				servo2Direction = SERVOC_DOWN;
				changeServo2Direction();
				TA1_State = TA1_AUTO_MOVE;
				Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
			}
			else if (servo2Direction != SERVOC_STOP)
			{
				// If the joystick is not moved stop the servo2
				servo2Direction = SERVOC_STOP;
				changeServo2Direction();
			}

			// If one of the buttons is pressed and servo3 is still change the servo3 movement direction
			if (bottomButtonPressed && servo3Direction == SERVOC_STOP)
			{
				pressTrigger();
			}
		}

		Graphics_drawStringCentered(&g_sContext,
									debugPrintString,
									AUTO_STRING_LENGTH,
									64,
									30,
									OPAQUE_TEXT);

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

		sprintf(string, "Top Button: %d", topButtonPressed);
		Graphics_drawStringCentered(&g_sContext,
									(int8_t *)string,
									AUTO_STRING_LENGTH,
									64,
									90,
									OPAQUE_TEXT);

		sprintf(string, "Bottom Button: %d", bottomButtonPressed);
		Graphics_drawStringCentered(&g_sContext,
									(int8_t *)string,
									AUTO_STRING_LENGTH,
									64,
									110,
									OPAQUE_TEXT);
	}
}

int RXData;
/* EUSCI A0 UART ISR - Echos data back to PC host */
void EUSCIA2_IRQHandler(void)
{
	uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

	if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG && turretAutomaticMode)
	{
		RXData = UART_receiveData(EUSCI_A2_BASE);

		char string[10];
		sprintf(string, "UART: %d", RXData);
		Graphics_drawStringCentered(&g_sContext,
									(int8_t *)string,
									AUTO_STRING_LENGTH,
									64,
									10,
									OPAQUE_TEXT);

		servo2Direction = SERVOC_STOP;
		changeServo2Direction();

		switch (RXData)
		{
		// left
		case 1:
			if (servo1Position > SERVO1_MIN)
			{
				servo1Position = servo1Position - 20;
				moveServo1();
			}
			break;
		// right
		case 0:
			if (servo1Position < SERVO1_MAX)
			{
				servo1Position = servo1Position + 20;
				moveServo1();
			}
			break;
		// up
		case 3:
			if (TA1_State == TA1_TRIGGER)
				break;
			servo2Direction = SERVOC_UP;
			changeServo2Direction();
			TA1_State = TA1_State;
			Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
			break;
		// down
		case 2:
			if (TA1_State == TA1_TRIGGER)
				break;
			servo2Direction = SERVOC_DOWN;
			changeServo2Direction();
			TA1_State = TA1_AUTO_MOVE;
			Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
			break;
		// centered
		case 4:
			if (TA1_State == TA1_NO_MOVE)
				break;
			pressTrigger();
			break;
		}

		Interrupt_disableSleepOnIsrExit();
	}
}
