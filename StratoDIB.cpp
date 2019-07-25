/*
 *  StratoDIB.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  
 *  This file implements an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as the overarching class
 *  for the FLOATS Data Interface Board, or DIB.
 */

#include "StratoDIB.h"

StratoDIB::StratoDIB()
    : StratoCore(&ZEPHYR_SERIAL, INSTRUMENT)
    , mcbTX(&MCB_SERIAL, &Serial)
    , mcbRX(&MCB_SERIAL, &Serial, DIB) // note: MCB expects DIB due to hard-coded Reader/Writer functions
{
    mcbTX.setDevId("DIB"); // note: MCB expects DIB due to hard-coded Reader/Writer functions
    waiting_mcb_messages = 0;
    mcb_low_power = false;
    mcb_motion_finished = false;
}

void StratoDIB::InstrumentSetup()
{
    // for RS232 transceiver
    pinMode(FORCEOFF_232, OUTPUT);
    pinMode(FORCEON_232, OUTPUT);
    digitalWrite(FORCEOFF_232, HIGH);
    digitalWrite(FORCEON_232, HIGH);

    // safe pin required by Zephyr
    pinMode(SAFE_PIN, OUTPUT);
    digitalWrite(SAFE_PIN, LOW);

    // FTR power pin (HIGH = OFF)
    pinMode(FTR_POWER, OUTPUT);
    digitalWrite(FTR_POWER, HIGH);
}

void StratoDIB::InstrumentLoop()
{
    WatchFlags();
}

// The telecommand handler must return ACK/NAK
bool StratoDIB::TCHandler(Telecommand_t telecommand)
{
    String dbg_msg = "";

    switch (telecommand) {
    case FTRONTIME:
        ftr_on_time = dibParam.ftrOnTime;
        dbg_msg = "Set on time to ";
        dbg_msg += String(ftr_on_time);
        ZephyrLogFine(dbg_msg.c_str());
        break;
    case FTRCYCLETIME:
        ftr_cycle_time = dibParam.ftrCycleTime;
        dbg_msg = "Set cycle time to ";
        dbg_msg += String(ftr_cycle_time);
        ZephyrLogFine(dbg_msg.c_str());
        break;
    default:
        log_error("Unknown TC received");
        // error case here
        break;
    }
    return true;
}

void StratoDIB::ActionHandler(uint8_t action)
{
    // for safety, ensure index doesn't exceed array size
    if (action >= NUM_ACTIONS) {
        log_error("Out of bounds action flag access");
        return;
    }

    // set the flag and reset the stale count
    action_flags[action].flag_value = true;
    action_flags[action].stale_count = 0;
}

bool StratoDIB::CheckAction(uint8_t action)
{
    // for safety, ensure index doesn't exceed array size
    if (action >= NUM_ACTIONS) {
        log_error("Out of bounds action flag access");
        return false;
    }

    // check and clear the flag if it is set, return the value
    if (action_flags[action].flag_value) {
        action_flags[action].flag_value = false;
        action_flags[action].stale_count = 0;
        return true;
    } else {
        return false;
    }
}

void StratoDIB::SetAction(uint8_t action)
{
    action_flags[action].flag_value = true;
    action_flags[action].stale_count = 0;
}

void StratoDIB::WatchFlags()
{
    // monitor for and clear stale flags
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (action_flags[i].flag_value) {
            action_flags[i].stale_count++;
            if (action_flags[i].stale_count >= FLAG_STALE) {
                action_flags[i].flag_value = false;
                action_flags[i].stale_count = 0;
            }
        }
    }
}

void StratoDIB::TakeMCBByte(uint8_t new_byte)
{
    mcbRX.putChar(new_byte);
    if (new_byte == 3) { // EOF char
        waiting_mcb_messages++;
    }
}

void StratoDIB::RunMCBRouter()
{
    if (waiting_mcb_messages) {
        waiting_mcb_messages--;
        mcbRX.igetNew();

        if (mcbRX.dataValid()) {
            switch (mcb_message) {
            case LOW_POWER_ACK:
                log_nominal("MCB in low power");
                mcb_low_power = true;
                break;
            case MOTION_FINISHED:
                log_nominal("MCB motion finished");
                mcb_motion_finished = true;
                break;
            default:
                log_error("Unknown MCB message received");
                break;
            }
        }
    }
}

void StratoDIB::FTRon()
{
    digitalWrite(FTR_POWER, LOW);
}

void StratoDIB::FTRoff()
{
    digitalWrite(FTR_POWER, HIGH);
}