/*
 *  StratoDIB.h
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  
 *  This file declares an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as the overarching class
 *  for the FLOATS Data Interface Board, or DIB.
 */

#ifndef StratoDIB_H
#define StratoDIB_H

#include "StratoCore.h"
#include "DIBHardware.h"
#include "DIBBufferGuard.h"

#define INSTRUMENT      FLOATS

// number of loops before a flag becomes stale and is reset
#define FLAG_STALE      2

// todo: update naming to be more unique (ie. ACT_ prefix)
enum ScheduleAction_t : uint8_t {
    NO_ACTION = NO_SCHEDULED_ACTION,
    SEND_IMR,
    RESEND_SAFETY,
    RESEND_MCB_LP,
    COMMAND_REEL_OUT,
    COMMAND_REEL_IN,
    COMMAND_MOTION_STOP,
    START_FTR,
    STOP_FTR,
    NUM_ACTIONS
};

class StratoDIB : public StratoCore {
public:
    StratoDIB();
    ~StratoDIB() { };

    // called before the loop begins
    void InstrumentSetup();

    // called at the end of each loop
    void InstrumentLoop();

    // called from serialEvent2 in main
    void TakeMCBByte(uint8_t new_byte);

    void RunMCBRouter();

private:
    // XML reader/writer for MCB comms
    XMLWriter_v4 mcbTX;
    XMLReader_v3 mcbRX;

    // tracks number of MCB messages that need processing
    uint8_t waiting_mcb_messages;

    // flags for MCB state
    bool mcb_low_power;
    bool mcb_motion_finished;

    // telecommand values
    float deploy_length;
    float retract_length;

    // Mode functions (implemented in unique source files)
    void StandbyMode();
    void FlightMode();
    void LowPowerMode();
    void SafetyMode();
    void EndOfFlightMode();

    // Telcommand handler - returns ack/nak
    bool TCHandler(Telecommand_t telecommand);

    // Action handler for scheduled actions
    void ActionHandler(uint8_t action);

    // Safely check and clear action flags
    bool CheckAction(uint8_t action);

    // Correctly set an action flag
    void SetAction(uint8_t action);

    // Monitor the action flags and clear old ones
    void WatchFlags();

    ActionFlag_t action_flags[NUM_ACTIONS] = {{0}}; // initialize all flags to false

    void FTRon();
    void FTRoff();

    uint16_t ftr_on_time = 300; // 5 min
    uint16_t ftr_cycle_time = 1500; // 25 min
};

#endif /* StratoDIB_H */