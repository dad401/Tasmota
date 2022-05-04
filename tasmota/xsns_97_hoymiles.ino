/*
  xsns_97_hoymiles.ino - read Hoymiles inverters via nrf24l01 support for Tasmota

  Copyright (C) 2022  Marcus Roscher
  
  Made possible by the great work of Lukas P. (lumapu), Hubi and other supporters at https://www.mikrocontroller.net/topic/525778 and 
  https://github.com/grindylow/ahoy
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifdef USE_SPI
#ifdef USE_NRF24
#ifdef USE_SUNRISE
#ifdef USE_HOYMILES

#define XSNS_97             97

#include <RF24.h>
#include <RF24_config.h>

//-------------------------------------
// CONFIGURATION - COMPILE TIME
//-------------------------------------
#define PACKET_BUFFER_SIZE      30
#define MAX_NUM_INVERTERS       3
#define MAX_NAME_LENGTH         16
#define MAX_RF_PAYLOAD_SIZE     64
#define DEFAULT_RECV_CHANNEL    3
#define DBG_CMD_LIST_LEN        7
#define SEND_INTERVAL_MS        1000  // interval for sending messages to inverter (ToDo: use SensorXY of tasmota for configuration)
#define SUN_OFFSET              15    // add this time to sunset or substract it at sunrise (safety margin) - (ToDo: use SensorXY of tasmota for configuration)

//#define CHANNEL_HOP // switch between channels or use static channel to send // (ToDo: use SensorXY of tasmota for configuration)

//-------------------------------------
typedef struct {
    uint8_t sendCh;
    uint8_t packet[MAX_RF_PAYLOAD_SIZE];
} packet_t;

#include "CircularBuffer.h"
#include "hmSystem.h"
#include <Ticker.h>

typedef CircularBuffer<packet_t, PACKET_BUFFER_SIZE> BufferType;
typedef HmRadio<BufferType> RadioType;
typedef HmSystem<RadioType, BufferType, MAX_NUM_INVERTERS, float> HmSystemType;

HmSystemType *mSys;
Ticker *mSendTicker;

bool mFlagSend;
bool NtpSet;
bool mIsDay;
uint32_t mTimestamp;
//uint32_t mChannelStat[4];
uint32_t mRecCnt;
uint64_t invSerial;
uint32_t mCmds[DBG_CMD_LIST_LEN+1];

float value;
char value_chr[FLOATSZ];
uint8_t pos;

//memset(mCmds, 0, sizeof(uint32_t)*DBG_CMD_LIST_LEN);
//memset(mChannelStat, 0, sizeof(uint32_t) * 4);

const char HTTP_SNS_HOYMILES[] PROGMEM = "{s}%s {m}%s %s{e}";

const char *invName = "HM-400";         // change (ToDo: use SensorXY of tasmota for configuration)
const char *WRSerial = "11217310xxyy";  // change (ToDo: use SensorXY of tasmota for configuration)
uint8_t invType = INV_TYPE_HM400;       // change (ToDo: use SensorXY of tasmota for configuration)


/******************************* INIT ***********************************************************/
void AHOYinit(void) {
  if (PinUsed(GPIO_NRF24_CS) && PinUsed(GPIO_NRF24_DC) && PinUsed(GPIO_NRF24_IRQ) && TasmotaGlobal.spi_enabled) {
       
    mFlagSend = false;  
    mIsDay = false;     
    NtpSet = false;    
    
    mSys = new HmSystemType(); 
    mSendTicker = new Ticker();
    
    mTimestamp  = UtcTime();
    invSerial = Serial2u64(WRSerial); // (ToDo: use SensorXY of tasmota for configuration)

    mSys->addInverter(invName, invSerial, invType);
    AddLog(LOG_LEVEL_INFO,PSTR("add inverter: %s - SN: %s - type: %d"), invName, WRSerial, invType);

    // add ticker for sending messages every SEND_INTERVAL
    mSendTicker->attach_ms(SEND_INTERVAL_MS, &sendTicker);

    // init radio with CircularBuffer
    mSys->setup();
  
    attachInterrupt(digitalPinToInterrupt(Pin(GPIO_NRF24_IRQ)), handleIntr, FALLING);
    AddLog(LOG_LEVEL_INFO, PSTR("HOYMILES init completed"));
  }
}


/******************************* LOOPS ***********************************************************/
void AHOY_EVERY_50_MSECOND(void) {
    if(!mSys->BufCtrl.empty()) {
        uint8_t len, rptCnt;
        packet_t *p = mSys->BufCtrl.getBack();
        //mSys->Radio.dumpBuf("RAW ", p->packet, MAX_RF_PAYLOAD_SIZE);

        if(mSys->Radio.checkCrc(p->packet, &len, &rptCnt)) {
            //AddLog(LOG_LEVEL_INFO,PSTR("received packet with correct crc"));
            // process buffer only on first occurrence
            if((0 != len) && (0 == rptCnt)) {
                uint8_t *cmd = &p->packet[11];
                //Serial.println("CMD " + String(*cmd, HEX));
                //mSys->Radio.dumpBuf("Payload ", p->packet, len);

                inverter_t *iv = mSys->findInverter(&p->packet[3]);
                if(NULL != iv) {
                    for(uint8_t i = 0; i < iv->listLen; i++) {
                        if(iv->assign[i].cmdId == *cmd)
                            mSys->addValue(iv, i, &p->packet[11]);
                            //Serial.print("Packet: ");
                            //dumpData(&p->packet[0], len);
                    }
                }

                if(*cmd == 0x01)      mCmds[0]++;
                else if(*cmd == 0x02) mCmds[1]++;
                else if(*cmd == 0x03) mCmds[2]++;
                else if(*cmd == 0x81) mCmds[3]++;
                else if(*cmd == 0x82) mCmds[4]++;
                else if(*cmd == 0x83) mCmds[5]++;
                else if(*cmd == 0x84) mCmds[6]++;
                else                  mCmds[7]++;

                /*if(p->sendCh == 23)      mChannelStat[0]++;
                else if(p->sendCh == 40) mChannelStat[1]++;
                else if(p->sendCh == 61) mChannelStat[2]++;
                else                     mChannelStat[3]++;*/
            }
        }
        mSys->BufCtrl.popBack();
    }

    // only send packet at ticker interval (mFlagSend) and if it`s day (mIsDay) ... and time is set
    if(mFlagSend && mIsDay) {
        mFlagSend = false;
        inverter_t *inv;
        for(uint8_t i = 0; i < MAX_NUM_INVERTERS; i ++) {
            inv = mSys->getInverterByPos(i);
            if(NULL != inv) {
                //AddLog(LOG_LEVEL_INFO,PSTR("send packet"));
                mSys->Radio.sendTimePacket(inv->radioId.u64, mTimestamp);
                //Serial.print("Sent packet with time: ");
                //Serial.println(mTimestamp);
                yield();
                //delay(100);
            }
        }
    }
    // Serial debug
    char topic[30], val[15];
    for(uint8_t id = 0; id < mSys->getNumInverters(); id++) {
        inverter_t *iv = mSys->getInverterByPos(id);
        if(NULL != iv) {
            for(uint8_t i = 0; i < iv->listLen; i++) {
                if(0.0f != mSys->getValue(iv, i)) {
                    snprintf(topic, 30, "%s/ch%d/%s", iv->name, iv->assign[i].ch, mSys->getFieldName(iv, i));
                    snprintf(val, 15, "%.3f %s", mSys->getValue(iv, i), mSys->getUnit(iv, i));
                    //DPRINTLN(String(topic) + ": " + String(val));
                    //Serial.println(String(topic) + ": " + String(val));
                }
                yield();
            }
        }
    }
}

void AHOY_EVERY_SECOND(void) {
  // update time for sending packets
  mTimestamp  = UtcTime();
  if (NtpSet) { 
    // do not check sunset/rise every second
    if (mTimestamp % 60 == 0) 
      mIsDay = isDay();
  } else {
   // simple check if time is set 
  if (mTimestamp > 1651263056) {
      NtpSet = true;
  }
  }
}

/******************************* WEB / MQTT ***********************************************************/
void AHOYShow(bool json)
{
  if(json) {
    ; // TODO: MQTT stuff
#ifdef USE_WEBSERVER
  } else {

  inverter_t *iv = mSys->getInverterByPos(0);

  if (mIsDay) {
    WSContentSend_P(PSTR("{s}%s "), iv->name);
    WSContentSend_P(PSTR("Cmd01: %d Cmd82: %d"), mCmds[0], mCmds[4]);
    value = mSys->getValue(iv, 7); 
    dtostrfd(value, Settings->flag2.wattage_resolution, value_chr);
    WSContentSend_P(HTTP_SNS_HOYMILES, D_JSON_POWERUSAGE, value_chr, D_UNIT_WATT);
    value = mSys->getValue(iv, 5); 
    dtostrfd(value, Settings->flag2.voltage_resolution, value_chr);
    WSContentSend_P(HTTP_SNS_HOYMILES, D_JSON_VOLTAGE, value_chr, D_UNIT_VOLT);
    value = mSys->getValue(iv, 0); 
    dtostrfd(value, Settings->flag2.voltage_resolution, value_chr);
    WSContentSend_P(HTTP_SNS_HOYMILES, PSTR("U_PV1"), value_chr, D_UNIT_VOLT);
  } else {
    WSContentSend_P(PSTR("{s}%s{e}"), iv->name);
    WSContentSend_P(PSTR("it is nighttime!"));
  }
  

  /*  
    for(uint8_t id = 0; id < mSys->getNumInverters(); id++) {
      inverter_t *iv = mSys->getInverterByPos(id);
      if(NULL != iv) {
          WSContentSend_P(PSTR("%s"), iv->name);
          switch(iv->type) {
              case INV_TYPE_HM600:  
                break;
              case INV_TYPE_HM1200:
                break;
              case INV_TYPE_HM400:
              default:              
                pos = (mSys->getPosByChField(iv, 0, FLD_PAC));
                if(0.0f != mSys->getValue(iv, pos)) {
                  value = mSys->getValue(iv, pos); 
                  dtostrfd(value, Settings->flag2.wattage_resolution, value_chr);
                  WSContentSend_P(HTTP_SNS_HOYMILES, D_JSON_POWERUSAGE, value_chr, D_UNIT_WATT);
                //}
                pos = (mSys->getPosByChField(iv, 0, FLD_UAC));
                //if(0.0f != mSys->getValue(iv, pos)) {
                  value = mSys->getValue(iv, pos); 
                  dtostrfd(value, Settings->flag2.voltage_resolution, value_chr);
                  WSContentSend_P(HTTP_SNS_HOYMILES, D_JSON_VOLTAGE, value_chr, D_UNIT_VOLT);
                //}
                pos = (mSys->getPosByChField(iv, 0, FLD_F));
                //if(0.0f != mSys->getValue(iv, pos)) {
                  value = mSys->getValue(iv, pos); 
                  dtostrfd(value, 2, value_chr);
                  WSContentSend_P(HTTP_SNS_HOYMILES, D_JSON_FREQUENCY, value_chr, PSTR("Hz"));
                //}
                pos = (mSys->getPosByChField(iv, 0, FLD_T));
                //if(0.0f != mSys->getValue(iv, pos)) {
                  value = mSys->getValue(iv, pos); 
                  dtostrfd(value, Settings->flag2.temperature_resolution, value_chr);
                  WSContentSend_P(HTTP_SNS_HOYMILES, D_JSON_TEMPERATURE, value_chr, D_UNIT_DEGREE);
                //}
                break;
          }
      }
      
    }
    */
  /*
      // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
      //WSContentSend_PD check for decimal separator (e.g. changes . to , in string)
      //WSContentSend_P(PSTR("{s}%s"), serial.c_str());
      WSContentSend_P(PSTR("%s"), iv->name);
      WSContentSend_PD(HTTP_SNS_VOLTAGE, voltage_chr);
      WSContentSend_PD(HTTP_SNS_POWER, power_chr);
      WSContentSend_P(PSTR("Cmd01: %d Cmd82: %d"), mCmds[0], mCmds[4]);
      WSContentSend_Temp(iv->name, hm_temp);
      WSContentButton(10,1);
  */
#endif // USE_WEBSERVER
  }
}

/******************************* routines ***********************************************************/
//ICACHE_RAM_ATTR void handleIntr(void) {
void IRAM_ATTR handleIntr(void) {
    //ISRcheck = true;
    mSys->Radio.handleIntr();
}

void sendTicker(void) {
    mFlagSend = true;
}

bool isDay() {
  TIME_T now;
  uint8_t min_rise;
  uint8_t hour_rise;
  uint8_t min_set;
  uint8_t hour_set;
  uint32_t currtime;

  // get local time 
  BreakTime(LocalTime(), now);
  currtime = now.hour * 60 + now.minute;

  // calculate sunset / sunrise using tasmotas SUNRISE function
  DuskTillDawn(&hour_rise, &min_rise, &hour_set, &min_set);

  // SUN_OFFSET before sunrise or after sunset
  if ((currtime > (hour_rise * 60 + min_rise - SUN_OFFSET)) && (currtime < (hour_set * 60 + min_set + SUN_OFFSET))) {
    //AddLog(LOG_LEVEL_INFO,PSTR("its daytime"));
    return true;
  } else {
    //AddLog(LOG_LEVEL_INFO,PSTR("its nighttime"));
    return false;
  }
}

uint64_t Serial2u64(const char *val) {
  char tmp[3] = {0};
  uint64_t ret = 0ULL;
  uint64_t u64;
  for(uint8_t i = 0; i < 6; i++) {
      tmp[0] = val[i*2];
      tmp[1] = val[i*2 + 1];
      if((tmp[0] == '\0') || (tmp[1] == '\0'))
          break;
      u64 = strtol(tmp, NULL, 16);
      ret |= (u64 << ((5-i) << 3));
  }
  return ret;
}
String uint64ToString(uint64_t input, uint8_t base) {
  String result = "";
  
  do {
    char c = input % base;
    input /= base;

    if (c < 10)
      c +='0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns97(uint8_t function)
{
  bool result = false;
  if (PinUsed(GPIO_NRF24_IRQ)) {
    switch (function) {
      case FUNC_INIT:
        AHOYinit();
        break;
      case FUNC_EVERY_50_MSECOND:
        AHOY_EVERY_50_MSECOND();
        break;
      case FUNC_EVERY_SECOND:
        AHOY_EVERY_SECOND();
        break;
      case FUNC_COMMAND:
        //result = NRFCmd();
        break;
      case FUNC_JSON_APPEND:
        //AHOYShow(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        AHOYShow(0);
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_HOYMILES
#endif  // USE_SUNRISE
#endif  // USE_NRF24
#endif  // USE_SPI