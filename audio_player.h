#ifndef AUDIO_PLAYER_H
#define AUDIO_PLAYER_H

#include <stdint.h>

void Audio_Init(void);
void Audio_Store(const uint8_t* data, uint32_t size);
void Audio_Play(void);
void Audio_Stop(void);
void Audio_TimerCallback(void);

#endif