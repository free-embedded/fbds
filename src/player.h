#include "song.h"
#ifndef PLAYER_H
#define PLAYER_H

int play_song(Song song);
void noTone();
void tone(int frequencyHz);
void _toneInit();
void delay(int time);
#endif