/*
 * DWM1000_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_Tag_H_
#define DWM1000_Tag_H_

#include <vertx.h>
#include <Hardware.h>
#include <DWM1000.h>
#include <DWM1000_Message.h>


#define ANCHOR_EXPIRE_TIME 10000
class RemoteAnchor
{
public:
    uint16_t _address;
    uint64_t _expires;
    uint8_t _sequence;
    int32_t _x;
    int32_t _y;
    uint32_t _distance;
    
    RemoteAnchor(){
        _address=0;
    }
    
    
    bool expired() {
        return Sys::millis()>_expires;
    }
    
    void update(uint8_t sequence) {
        _expires = Sys::millis()+ANCHOR_EXPIRE_TIME;
        if ( sequence > (_sequence+1)) INFO(" dropped %d frames from 0x%X",sequence-_sequence-1,_address);
        _sequence=sequence;
    }

    void update(BlinkMsg& blinkMsg) {
        uint8_t sequence = blinkMsg.sequence;
        _expires = Sys::millis()+ANCHOR_EXPIRE_TIME;
        if ( sequence > (_sequence+1)) INFO(" dropped %d frames from 0x%X",sequence-_sequence-1,_address);
        little_endian(_x,blinkMsg.x);
        little_endian(_y,blinkMsg.y);
        little_endian(_distance,blinkMsg.distance);
        _sequence=sequence;
    }
    
    RemoteAnchor(uint16_t address,uint8_t sequence) {
        _address=address;
        _expires = Sys::millis()+ANCHOR_EXPIRE_TIME;
        _sequence = sequence;
    }
    
    void remove(){
        _address=0;
    }
    
};

class DWM1000_Tag: public VerticleCoRoutine,public DWM1000
{
    uint32_t _count;
    static DWM1000_Tag* _tag;
    uint32_t _interrupts;
    uint32_t _polls;
    bool interrupt_detected;
    uint32_t _resps;
    uint32_t _blinks;
    uint32_t _finals;
    uint32_t _frame_len;
    BlinkMsg _blinkMsg;
    PollMsg _pollMsg;
    RespMsg _respMsg;
    FinalMsg _finalMsg;
    DwmMsg _dwmMsg;
    Str _anchors;
    uint32_t _anchorIndex;
    uint32_t _anchorMax;
    Str _panAddress;
    uint8_t _rxdSequence;
    
    typedef enum  {
        RCV_ANY=H("RCV_ANY"),
        RCV_RESP=H("RCV_RESP"),
        RCV_FINAL=H("SND_FINAL")
    } State;
    uint16_t _currentAnchor;
    State _state;
    Timeout _pollTimer;
public:
    DWM1000_Tag(const char* name,Spi& spi,DigitalIn& irq,DigitalOut& reset);
     ~DWM1000_Tag();
    void mode(uint32_t m);
    void start();
    void loop();
//    void resetChip();
    void initSpi();
    static void my_dwt_isr(void *);
    bool isInterruptDetected();
    bool clearInterrupt();

   void run();

    int sendFinalMsg();
    int sendPollMsg();
    static void rxcallback(const  dwt_callback_data_t* event) ;
    static void txcallback(const  dwt_callback_data_t* event) ;
    void FSM(const dwt_callback_data_t* signal);
    void onDWEvent(const dwt_callback_data_t* signal);
    FrameType readMsg(const dwt_callback_data_t* signal);
    void updateAnchors(uint16_t address,uint8_t sequence);
    void updateAnchors(BlinkMsg& blinkMsg);
    void expireAnchors();
    bool pollAnchors();
    void handleBlinkMsg();
    void handleRespMsg();
private:

};

#endif /* DWM1000_Tag_H_ */
