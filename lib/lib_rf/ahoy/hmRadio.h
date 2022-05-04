#ifndef __RADIO_H__
#define __RADIO_H__

#include <RF24.h>
#include <RF24_config.h>
#include "crc.h"


#define DTU_RADIO_ID            ((uint64_t)0x1234567801ULL)
#define DUMMY_RADIO_ID          ((uint64_t)0xDEADBEEF01ULL)


const char* const rf24AmpPower[] = {"MIN", "LOW", "HIGH", "MAX"};



//-----------------------------------------------------------------------------
// MACROS
//-----------------------------------------------------------------------------
#define CP_U32_LittleEndian(buf, v) ({ \
    uint8_t *b = buf; \
    b[0] = ((v >> 24) & 0xff); \
    b[1] = ((v >> 16) & 0xff); \
    b[2] = ((v >>  8) & 0xff); \
    b[3] = ((v      ) & 0xff); \
})

#define CP_U32_BigEndian(buf, v) ({ \
    uint8_t *b = buf; \
    b[3] = ((v >> 24) & 0xff); \
    b[2] = ((v >> 16) & 0xff); \
    b[1] = ((v >>  8) & 0xff); \
    b[0] = ((v      ) & 0xff); \
})

#define BIT_CNT(x)  ((x)<<3)


//-----------------------------------------------------------------------------
// HM Radio class
//-----------------------------------------------------------------------------
template <class BUFFER, uint64_t DTU_ID=DTU_RADIO_ID>
class HmRadio {
    public:
        HmRadio() {
            mChanOut[0] = 23;
            mChanOut[1] = 40;
            mChanOut[2] = 61;
            mChanOut[3] = 75;
            mChanIdx = 1;

            calcDtuCrc();

            AmplifierPower = 3;
            mSendCnt       = 0;
        }
        ~HmRadio() {}

        void setup(BUFFER *ctrl) {
            //DPRINTLN("HmRadio::setup, pins: " + String(pinCs) + ", " + String(pinCe) + ", " + String(pinIrq));
            pinMode(Pin(GPIO_NRF24_IRQ), INPUT_PULLUP);

            mBufCtrl = ctrl;

            //NRF24radio.begin(pinCe, pinCs);
            NRF24radio.setAutoAck(false);
            NRF24radio.setRetries(0, 0);

            NRF24radio.setChannel(DEFAULT_RECV_CHANNEL);
            NRF24radio.setDataRate(RF24_250KBPS);
            NRF24radio.disableCRC();
            NRF24radio.setAutoAck(false);
            NRF24radio.setPayloadSize(MAX_RF_PAYLOAD_SIZE);
            NRF24radio.setAddressWidth(5);
            NRF24radio.openReadingPipe(1, DTU_RADIO_ID);

            // enable only receiving interrupts
            NRF24radio.maskIRQ(true, true, false);

            //DPRINTLN("RF24 Amp Pwr: RF24_PA_" + String(rf24AmpPower[AmplifierPower]));
            NRF24radio.setPALevel(AmplifierPower & 0x03);
            NRF24radio.startListening();

            //DPRINTLN("Radio Config:");
            //NRF24radio.printPrettyDetails(); // not supported in tasmota NRF24 lib
            NRF24radio.printDetails();

            mSendChannel = getDefaultChannel();

            /* already checked in xdrv_33_nrf24l01.ino
            if(!NRF24.isChipConnected()) {
                DPRINTLN("WARNING! your NRF24 module can't be reached, check the wiring");
            }
            */
        }

        void handleIntr(void) {
            uint8_t pipe, len;
            packet_t *p;

            noInterrupts();
            while(NRF24radio.available(&pipe)) {
                if(!mBufCtrl->full()) {
                    p = mBufCtrl->getFront();
                    memset(p->packet, 0xcc, MAX_RF_PAYLOAD_SIZE);
                    p->sendCh = mSendChannel;
                    len = NRF24radio.getPayloadSize();
                    if(len > MAX_RF_PAYLOAD_SIZE)
                        len = MAX_RF_PAYLOAD_SIZE;

                    NRF24radio.read(p->packet, len);
                    mBufCtrl->pushFront(p);
                }
                else {
                    bool tx_ok, tx_fail, rx_ready;
                    NRF24radio.whatHappened(tx_ok, tx_fail, rx_ready); // reset interrupt status
                    NRF24radio.flush_rx(); // drop the packet
                }
            }
            interrupts();
        }

        uint8_t getDefaultChannel(void) {
            return mChanOut[2];
        }
        uint8_t getLastChannel(void) {
            return mChanOut[mChanIdx];
        }

        uint8_t getNxtChannel(void) {
            if(++mChanIdx >= 4)
                mChanIdx = 0;
            return mChanOut[mChanIdx];
        }

        void sendTimePacket(uint64_t invId, uint32_t ts) {
            sendCmdPacket(invId, 0x15, 0x80, false);
            mSendBuf[10] = 0x0b; // cid
            mSendBuf[11] = 0x00;
            CP_U32_LittleEndian(&mSendBuf[12], ts);
            mSendBuf[19] = 0x05;

            uint16_t crc = AHOY_crc16(&mSendBuf[10], 14);
            mSendBuf[24] = (crc >> 8) & 0xff;
            mSendBuf[25] = (crc     ) & 0xff;
            mSendBuf[26] = AHOY_crc8(mSendBuf, 26);

            sendPacket(invId, mSendBuf, 27);
        }

        void sendCmdPacket(uint64_t invId, uint8_t mid, uint8_t cmd, bool calcCrc = true) {
            memset(mSendBuf, 0, MAX_RF_PAYLOAD_SIZE);
            mSendBuf[0] = mid; // message id
            CP_U32_BigEndian(&mSendBuf[1], (invId  >> 8));
            CP_U32_BigEndian(&mSendBuf[5], (DTU_ID >> 8));
            mSendBuf[9]  = cmd;
            if(calcCrc) {
                mSendBuf[10] = AHOY_crc8(mSendBuf, 10);
                sendPacket(invId, mSendBuf, 11);
            }
        }

        bool checkCrc(uint8_t buf[], uint8_t *len, uint8_t *rptCnt) {
            *len = (buf[0] >> 2);
            for (int16_t i = MAX_RF_PAYLOAD_SIZE - 1; i >= 0; i--) {
                buf[i] = ((buf[i] >> 7) | ((i > 0) ? (buf[i-1] << 1) : 0x00));
            }
            uint16_t crc = AHOY_crc16nrf24(buf, BIT_CNT(*len + 2), 7, mDtuIdCrc);

            bool valid = (crc == ((buf[*len+2] << 8) | (buf[*len+3])));

            if(valid) {
                if(mLastCrc == crc)
                    *rptCnt = (++mRptCnt);
                else {
                    mRptCnt = 0;
                    *rptCnt = 0;
                    mLastCrc = crc;
                }
            }

            return valid;
        }

        /*
        void dumpBuf(const char *info, uint8_t buf[], uint8_t len) {
            DPRINT(String(info));
            for(uint8_t i = 0; i < len; i++) {
                if(buf[i] < 10)
                    DPRINT("0");
                DHEX(buf[i]);
                DPRINT(" ");
            }
            DPRINTLN("");
        }
        */
        /*
        bool isChipConnected(void) {
            return mNrf24.isChipConnected();
        }
        */

        uint8_t AmplifierPower;

    private:
        void sendPacket(uint64_t invId, uint8_t buf[], uint8_t len) {
            //DPRINTLN("sent packet: #" + String(mSendCnt));
            //dumpBuf("SEN ", buf, len);

            noInterrupts();
            NRF24radio.stopListening();

        #ifdef CHANNEL_HOP
            mSendChannel = getNxtChannel();
        #else
            mSendChannel = getDefaultChannel();
        #endif
            NRF24radio.setChannel(mSendChannel);
            //DPRINTLN("CH: " + String(mSendChannel));

            NRF24radio.openWritingPipe(invId); // TODO: deprecated
            NRF24radio.setCRCLength(RF24_CRC_16);
            NRF24radio.enableDynamicPayloads();
            NRF24radio.setAutoAck(true);
            NRF24radio.setRetries(3, 15);

            NRF24radio.write(buf, len);

            // Try to avoid zero payload acks (has no effect)
            NRF24radio.openWritingPipe(DUMMY_RADIO_ID); // TODO: why dummy radio id?, deprecated

            NRF24radio.setAutoAck(false);
            NRF24radio.setRetries(0, 0);
            NRF24radio.disableDynamicPayloads();
            NRF24radio.setCRCLength(RF24_CRC_DISABLED);

            NRF24radio.setChannel(DEFAULT_RECV_CHANNEL);
            NRF24radio.startListening();

            interrupts();
            mSendCnt++;
        }

        void calcDtuCrc(void) {
            uint64_t addr = DTU_RADIO_ID;
            uint8_t tmp[5];
            for(int8_t i = 4; i >= 0; i--) {
                tmp[i] = addr;
                addr >>= 8;
            }
            mDtuIdCrc = AHOY_crc16nrf24(tmp, BIT_CNT(5));
        }

        uint8_t mChanOut[4];
        uint8_t mChanIdx;
        uint16_t mDtuIdCrc;
        uint16_t mLastCrc;
        uint8_t mRptCnt;

        uint8_t mSendChannel;
        BUFFER *mBufCtrl;
        uint32_t mSendCnt;
        uint8_t mSendBuf[MAX_RF_PAYLOAD_SIZE];
};

#endif /*__RADIO_H__*/
