#pragma once
#include <stdint.h>
class TwoWire {public:
 bool short_read=false;uint8_t error=0,byte=15;
 void beginTransmission(uint8_t){} void write(uint8_t){}
 uint8_t endTransmission(){return error;}
 uint8_t requestFrom(uint8_t,uint8_t n){return short_read?n-1:n;}
 int read(){return byte;}
};extern TwoWire Wire;
