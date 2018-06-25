/*
 * DWM1000_Anchor_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_Anchor_H_
#define DWM1000_Anchor_H_

#include <vertx.h>
#include <Hardware.h>
#include <DWM1000_Message.h>
#include <DWM1000.h>

class DWM1000_Anchor: public VerticleTask,public DWM1000
{
    uint32_t _count;
    uint32_t _interrupts;
    uint32_t _polls;
    uint32_t _finals;
    uint32_t _blinks;
    uint32_t _resps;
    uint32_t _errs;
    uint32_t _missed;

    uint32_t _interruptDelay;


    uint8_t _lastSequence;
    static DWM1000_Anchor* _anchor;
//   enum { WAIT_POLL, WAIT_FINAL } _state;
    bool interrupt_detected ;
    uint32_t _frame_len;
    PollMsg _pollMsg;
    RespMsg _respMsg;
    FinalMsg _finalMsg;
    BlinkMsg _blinkMsg;
    DwmMsg _dwmMsg;
    Str _panAddress;
    Cbor _irqEvent;
    bool _hasIrqEvent;
    float _distance;
//   uint32_t _distanceInCm;
    uint8_t _blinkSequence;
    typedef  enum { RCV_ANY=H("RCV_ANY"),
                    RCV_POLL=H("RCV_POLL"),
                    RCV_FINAL=H("RCV_FINAL")
                  } State;
    State _state;
    Timeout _blinkTimer;
    PropertyReference<float>* _distanceProp;
    Str _role;

public:
    uint64_t _interruptStart;
    DWM1000_Anchor(const char* name,Spi& spi,DigitalIn& irq,DigitalOut& reset);
    ~DWM1000_Anchor();
    void mode(uint32_t m);
    void start();
    void run();
    void init();

    void sendReply();
    void calcFinalMsg();
    int sendRespMsg();

    void update(uint16_t srcAddress,uint8_t sequence);
    static void rxcallback(const  dwt_callback_data_t* event) ;
    static void txcallback(const  dwt_callback_data_t* event) ;

    void FSM(const dwt_callback_data_t* signal);
    void onDWEvent(const  dwt_callback_data_t* event);
    FrameType readMsg(const dwt_callback_data_t* signal);
    void sendBlinkMsg();
    void handleFinalMsg();
};

#endif /* DWM1000_Anchor_Tag_H_ */
