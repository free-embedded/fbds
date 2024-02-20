#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include "LcdDriver/HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.h"
#include <stdio.h>

#include "notes.h"
#include "song.h"
#include "player.h"

#define ADAPT_TO_48MHZ 14

/* Timer_A Compare Configuration Parameter  (PWM) */
Timer_A_CompareModeConfig compareConfig_PWM_buzzer = {
        TIMER_A_CAPTURECOMPARE_REGISTER_4,          // Use CCR3
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_SET,              // Toggle output but
        61500,                                      // 25% Duty Cycle initially
};

/* Timer_A Up Configuration Parameter */
Timer_A_UpModeConfig upConfig_buzzer = {
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK = 3 MhZ
        TIMER_A_CLOCKSOURCE_DIVIDER_12,         // SMCLK/12 = 250 KhZ
        62500,                                  // 40 ms tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

void noTone() {
    /* Configuring Timer_A0 for Up Mode and starting */
    upConfig_buzzer.timerPeriod = 10000 * ADAPT_TO_48MHZ;
    Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig_buzzer);

    /* Initialize compare registers to generate PWM */
    compareConfig_PWM_buzzer
        .compareValue = 20000 * ADAPT_TO_48MHZ;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM_buzzer
    ); // For P2.7

    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

void tone(int frequencyHz) {
    /* Configuring Timer_A0 for Up Mode and starting */
    upConfig_buzzer.timerPeriod = 250000 / frequencyHz * ADAPT_TO_48MHZ;
    Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig_buzzer);

    /* Initialize compare registers to generate PWM */
    compareConfig_PWM_buzzer
        .compareValue = 125000 / frequencyHz * ADAPT_TO_48MHZ;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM_buzzer
    ); // For P2.7

    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

void _toneInit() {
    /* Configures P2.7 to PM_TA0.4 for using Timer PWM to control the buzzer */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7,
        GPIO_PRIMARY_MODULE_FUNCTION);

    noTone();
}

void delay(int time) {
    //__delay_cycles(200 * time);
    int i;
    for (i = 0;i < 200 * ADAPT_TO_48MHZ * time;++i);
}

void play_song(Song song) {

    // sizeof gives the number of bytes, each int value is composed of two bytes (ADAPT_TO_48MHZ bits)
    // there are two values per note (pitch and duration), so for each note there are four bytes
    // int notes = sizeof(song.melody) / sizeof(song.melody[0]) / 2;

    // this calculates the duration of a whole note in ms
    int wholenote = (60000 * 4) / song.tempo;

    int divider = 0, noteDuration = 0;



    // iterate over the notes of the melody.
    // Remember, the array is twice the number of notes (notes + durations)
    int thisNote;
    for (thisNote = 0; thisNote < song.notes * 2; thisNote = thisNote + 2) {
      // calculates the duration of each note
        divider = song.melody[thisNote + 1];
        if (divider > 0) {
          // regular note, just proceed
            noteDuration = (wholenote) / divider;
        }
        else if (divider < 0) {
       // dotted notes are represented with negative durations!!
            noteDuration = (wholenote) / abs(divider);
            noteDuration *= 1.5; // increases the duration in half for dotted notes
        }

        // we only play the note for 90% of the duration, leaving 10% as a pause
        tone(song.melody[thisNote]);

        // Wait for the specief duration before playing the next note.
        delay(noteDuration * 0.9);

        // stop the waveform generation before the next note.
        noTone();

        // Wait for the specief duration before playing the next note.
        delay(noteDuration * 0.1);
    }
}