#include <Wire.h>
#include "VL53L1X.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
TwoWire secondBus(1);
VL53L1X sensors[2];
bool present[2]={false,false};
uint16_t distanceMM[2]={0,0};
uint8_t statusCode[2]={255,255};
uint32_t measured[2]={0,0},sequence=0,lastSend=0;
uint32_t retryAt[2]={0,0};

#define FIRST_USES_PHYSICAL_RIGHT 0
#if FIRST_USES_PHYSICAL_RIGHT
const int shutPins[2]={33,32};
static TwoWire* const busFor[2]={&secondBus,&Wire};
#else
const int shutPins[2]={32,33};
static TwoWire* const busFor[2]={&Wire,&secondBus};
#endif

#define LEFT_ONLY_TEST 0

#define POWER_CYCLE_TEST 0
volatile bool connected=false,reAdvertise=false;
BLECharacteristic *tx;
struct __attribute__((packed)) Packet {
 char magic[2];uint8_t version,present;uint32_t seq,ms;uint16_t left,right;uint8_t ls,rs;uint16_t reserved;
};
static_assert(sizeof(Packet)==20,"Packet must fit default BLE payload");
class Callbacks:public BLEServerCallbacks {
 void onConnect(BLEServer *s,esp_ble_gatts_cb_param_t *p) override {
  connected=true;s->updateConnParams(p->connect.remote_bda,12,24,0,400);
 }
 void onDisconnect(BLEServer*) override {connected=false;reAdvertise=true;}
};
bool initSensor(int i,TwoWire &bus){
 statusCode[i]=255;distanceMM[i]=0;
 bus.beginTransmission(0x29);uint8_t ack=bus.endTransmission();
 Serial.printf("SENSOR%d ACK=%u\n",i,ack);if(ack!=0)return false;
 sensors[i]=VL53L1X();
 sensors[i].setBus(&bus);sensors[i].setTimeout(500);
 Serial.printf("SENSOR%d MODEL=%04X\n",i,sensors[i].readReg16Bit(VL53L1X::IDENTIFICATION__MODEL_ID));

 Serial.printf("SENSOR%d PRE=0x%02X\n",i,sensors[i].readReg(VL53L1X::FIRMWARE__SYSTEM_STATUS));
 if(!sensors[i].init()){

  uint8_t io=sensors[i].io_error;uint16_t reg=sensors[i].error_reg;
  uint8_t sys=sensors[i].readReg(VL53L1X::FIRMWARE__SYSTEM_STATUS);
  Serial.printf("SENSOR%d INIT FAILED io=%u reg=0x%04X (5=short read,6=invalid timing) systat=0x%02X\n",i,io,reg,sys);
  return false;
 }
 if(!sensors[i].setDistanceMode(VL53L1X::Long)||!sensors[i].setMeasurementTimingBudget(50000)){Serial.printf("SENSOR%d TIMING FAILED io=%u\n",i,sensors[i].io_error);return false;}
 sensors[i].startContinuous(60);
 delay(120);

 uint8_t md=sensors[i].readReg(VL53L1X::SYSTEM__MODE_START);
 uint8_t gp=sensors[i].readReg(VL53L1X::GPIO__TIO_HV_STATUS);
 if(md!=0x40){Serial.printf("SENSOR%d START LOST mode=0x%02X gpio=0x%02X (chip reloaded its defaults)\n",i,md,gp);return false;}
 return sensors[i].io_error==0;
}
void setup(){
 Serial.begin(115200);
 Serial.println("TOF-BLE v1.1 / protocol v1 / guarded I2C and independent BLE");

 for(int i=0;i<2;i++){pinMode(shutPins[i],OUTPUT);digitalWrite(shutPins[i],LOW);}
 delay(20);
#if LEFT_ONLY_TEST
 digitalWrite(shutPins[0],HIGH);
#else
 for(int i=0;i<2;i++){digitalWrite(shutPins[i],HIGH);delay(5);}
#endif
 delay(300);
 Wire.begin(21,22,100000);Wire.setTimeOut(50);
 secondBus.begin(25,26,100000);secondBus.setTimeOut(50);
#if POWER_CYCLE_TEST
 for(int i=0;i<2;i++){
  int fail=0,failBoot=0;
  for(int n=0;n<50;n++){
   digitalWrite(shutPins[i],LOW);delay(20);
   digitalWrite(shutPins[i],HIGH);delay(60);
   sensors[i]=VL53L1X();
   sensors[i].setBus(busFor[i]);sensors[i].setTimeout(50);
   sensors[i].io_error=0;
   uint16_t m=sensors[i].readReg16Bit(VL53L1X::IDENTIFICATION__MODEL_ID);
   if(sensors[i].io_error||m!=0xEACC)fail++;
   else if((sensors[i].readReg(VL53L1X::FIRMWARE__SYSTEM_STATUS)&1)==0)failBoot++;
  }
  Serial.printf("POWER CYCLE SENSOR%d: %d/50 no answer, %d/50 not booted\n",i,fail,failBoot);
 }
#endif

 BLEDevice::init("FPGA-ESP32-TOF");
 auto server=BLEDevice::createServer();server->setCallbacks(new Callbacks());
 auto service=server->createService("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
 tx=service->createCharacteristic("6e400003-b5a3-f393-e0a9-e50e24dcca9e",BLECharacteristic::PROPERTY_NOTIFY);
 tx->addDescriptor(new BLE2902());service->start();
 auto adv=BLEDevice::getAdvertising();adv->addServiceUUID(service->getUUID());adv->setScanResponse(true);adv->start();
 Serial.println("BLE READY; initializing sensors independently");
 present[0]=initSensor(0,*busFor[0]);
#if LEFT_ONLY_TEST
 present[1]=false;
#else
 present[1]=initSensor(1,*busFor[1]);
#endif
 measured[0]=measured[1]=millis();retryAt[0]=retryAt[1]=millis();
 Serial.printf("TOF READY left=%d right=%d\n",present[0],present[1]);
}
void loop(){
 uint32_t now=millis();
 if(reAdvertise){reAdvertise=false;BLEDevice::startAdvertising();}
 for(int i=0;i<2;i++){
#if LEFT_ONLY_TEST
  if(i==1)continue;
#endif
  if(!present[i]){
   if(now-retryAt[i]>=3000){

    pinMode(shutPins[i],OUTPUT);digitalWrite(shutPins[i],LOW);delay(20);
    digitalWrite(shutPins[i],HIGH);delay(50);
    present[i]=initSensor(i,*busFor[i]);retryAt[i]=millis();measured[i]=millis();
    if(present[i])Serial.printf("SENSOR%d RECOVERED\n",i);
   }
   continue;
  }
  sensors[i].io_error=0;
  bool ready=sensors[i].dataReady();
  if(sensors[i].io_error!=0)statusCode[i]=254;
  else if(ready){
   distanceMM[i]=sensors[i].read(false);
   statusCode[i]=(sensors[i].io_error==0&&!sensors[i].timeoutOccurred()) ? (uint8_t)sensors[i].ranging_data.range_status : 254;
   if(sensors[i].io_error==0)measured[i]=now;
  }
  if(now-measured[i]>250)statusCode[i]=254;
  if(now-measured[i]>1000){
   uint8_t io=sensors[i].io_error;
   uint8_t g=sensors[i].readReg(VL53L1X::GPIO__TIO_HV_STATUS);
   uint8_t r=sensors[i].readReg(VL53L1X::RESULT__RANGE_STATUS);
   uint8_t md=sensors[i].readReg(VL53L1X::SYSTEM__MODE_START);
   uint8_t sy=sensors[i].readReg(VL53L1X::FIRMWARE__SYSTEM_STATUS);
   Serial.printf("SENSOR%d STALE io=%u gpio=0x%02X rs=0x%02X mode=0x%02X systat=0x%02X; retrying\n",i,io,g,r,md,sy);
   present[i]=false;retryAt[i]=now;
  }
 }
 if(now-lastSend>=60){
  lastSend=now;
  Packet p={{'T','F'},1,(uint8_t)(present[0]|(present[1]<<1)),sequence++,now,distanceMM[0],distanceMM[1],statusCode[0],statusCode[1],0};
  if(connected){tx->setValue((uint8_t*)&p,sizeof(p));tx->notify();}
  Serial.printf("%lu,%u,%u,%u,%u\n",now,p.left,p.right,p.ls,p.rs);
 }
 delay(3);
}
