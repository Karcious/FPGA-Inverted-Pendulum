#include <cassert>
#include <cstdio>
#define private public
#include "VL53L1X.h"
#undef private
TwoWire Wire;
int main(){VL53L1X s;
assert(s.readReg16Bit(0)==0x0f0f && !s.io_error);
Wire.short_read=true;s.io_error=0;assert(s.readReg16Bit(0)==0 && s.io_error==5);
s.io_error=0;assert(s.read(false)==0 && s.io_error==5 && s.ranging_data.range_status==VL53L1X::None);
Wire.short_read=false;Wire.error=2;s.io_error=0;assert(s.readReg(VL53L1X::SOFT_RESET)==0 && s.io_error==2);
Wire.error=0;s.io_error=0;s.fast_osc_frequency=0;assert(!s.setMeasurementTimingBudget(50000) && s.io_error==6);
s.io_error=0;s.fast_osc_frequency=1000;Wire.byte=255;assert(!s.setMeasurementTimingBudget(50000) && s.io_error==6);
s.io_error=0;Wire.byte=15;assert(s.setMeasurementTimingBudget(50000) && !s.io_error);
printf("PASS: normal I2C, short reads, NACK, zero oscillator, invalid period, valid timing\n");}
