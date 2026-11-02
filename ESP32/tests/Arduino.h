#pragma once
#include <stdint.h>
inline unsigned long millis(){static unsigned long t=0;return ++t;}
inline void delay(unsigned long){}
inline void delayMicroseconds(unsigned int){}
