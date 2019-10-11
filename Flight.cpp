/*
 *  Flight.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  This file implements the MCBOATS flight mode.
 */

#include "StratoDIB.h"

// Flight mode states for FTR
enum FLStatesFTR_t : uint8_t {
    FTR_ENTRY = MODE_ENTRY,

    // before anything else
    FTR_GPS_WAIT,

    // FTR operation states
    FTR_IDLE,

    // ----------------------------------------------------
    // Define FTR states here
    // ----------------------------------------------------

    // general off-nominal states in response to following landing states
    FTR_ERROR_LOOP,
    FTR_SHUTDOWN_LOOP,

    // 253 - 255, defined in StratoCore
    FTR_ERROR_LANDING = MODE_ERROR,
    FTR_SHUTDOWN_LANDING = MODE_SHUTDOWN,
    FTR_EXIT = MODE_EXIT
};

// Flight mode states for MCB
enum FLStatesMCB_t : uint8_t {
    MCB_ENTRY = MODE_ENTRY,

    // before anything else
    MCB_GPS_WAIT,

    // MCB motion states
    MCB_IDLE,
    MCB_START_MOTION,
    MCB_VERIFY_MOTION,
    MCB_MONITOR_MOTION,
    MCB_TM_ACK,

    // general off-nominal states in response to following landing states
    MCB_ERROR_LOOP,
    MCB_SHUTDOWN_LOOP,

    // 253 - 255, defined in StratoCore
    MCB_ERROR_LANDING = MODE_ERROR,
    MCB_SHUTDOWN_LANDING = MODE_SHUTDOWN,
    MCB_EXIT = MODE_EXIT
};

// this function is called at the defined rate
//  * when flight mode is entered, it will start in MCB_ENTRY state
//  * it is then up to this function to change state as needed by updating the inst_substate variable
//  * on each loop, whichever substate is set will be perfomed
//  * when the mode is changed by the Zephyr, MCB_EXIT will automatically be set
//  * it is up to the MCB_EXIT logic perform any actions for leaving flight mode
void StratoDIB::FlightMode()
{
    if (FTR_SUBMODE == flight_submode) {
        FlightFTR();
    } else {
        FlightMCB();
    }
}

void StratoDIB::FlightFTR()
{
    switch (inst_substate) {
    case FTR_ENTRY:
        // perform setup
        log_nominal("Entering FTR");
        inst_substate = FTR_GPS_WAIT;
        break;
    case FTR_GPS_WAIT:
        // wait for the first GPS message from Zephyr to set the time before moving on
        log_debug("Waiting on GPS time");
        if (time_valid) {
            inst_substate = FTR_IDLE;
        }
        break;
    case FTR_IDLE:
        log_debug("FTR Idle");
        break;

    // ----------------------------------------------------
    // Implement FTR states here
    // ----------------------------------------------------

    case FTR_ERROR_LANDING:
        log_error("Landed in flight error");
        inst_substate = FTR_ERROR_LOOP;
        break;
    case FTR_ERROR_LOOP:
        log_debug("FTR error loop");

        if (CheckAction(EXIT_ERROR_STATE)) {
            log_nominal("Leaving flight error loop");
            inst_substate = FTR_ENTRY;
        }
        break;
    case FTR_SHUTDOWN_LANDING:
        // prep for shutdown
        log_nominal("Shutdown warning received in FTR");
        inst_substate = FTR_SHUTDOWN_LOOP;
        break;
    case FTR_SHUTDOWN_LOOP:
        break;
    case FTR_EXIT:
        // perform cleanup
        log_nominal("Exiting Flight from FTR");
        break;
    default:
        log_error("Unknown flight FTR state");
        break;
    }
}

void StratoDIB::FlightMCB()
{
    switch (inst_substate) {
    case MCB_ENTRY:
        // perform setup
        log_nominal("Entering MCB");
        inst_substate = MCB_GPS_WAIT;
        break;
    case MCB_GPS_WAIT:
        // wait for the first GPS message from Zephyr to set the time before moving on
        log_debug("Waiting on GPS time");
        if (time_valid) {
            inst_substate = MCB_IDLE;
        }
        break;
    case MCB_IDLE:
        log_debug("MCB Idle");
        if (CheckAction(COMMAND_REEL_IN)) {
            mcb_motion = MOTION_REEL_IN;
            inst_substate = MCB_START_MOTION;
            resend_attempted = false;
        } else if (CheckAction(COMMAND_REEL_OUT)) {
            mcb_motion = MOTION_REEL_OUT;
            inst_substate = MCB_START_MOTION;
            resend_attempted = false;
        } else if (CheckAction(COMMAND_DOCK)) {
            mcb_motion = MOTION_DOCK;
            inst_substate = MCB_START_MOTION;
            resend_attempted = false;
        }
        break;
    case MCB_START_MOTION:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion commanded while motion ongoing");
            inst_substate = MCB_ERROR_LANDING;
        }

        if (StartMCBMotion()) {
            inst_substate = MCB_VERIFY_MOTION;
            scheduler.AddAction(RESEND_MOTION_COMMAND, MCB_RESEND_TIMEOUT);
        } else {
            ZephyrLogWarn("Motion start error");
            inst_substate = MCB_ERROR_LANDING;
        }
        break;
    case MCB_VERIFY_MOTION:
        if (mcb_motion_ongoing) { // set in the Ack handler
            log_nominal("MCB commanded motion");
            inst_substate = MCB_MONITOR_MOTION;
        }

        if (CheckAction(RESEND_MOTION_COMMAND)) {
            if (!resend_attempted) {
                resend_attempted = true;
                inst_substate = MCB_START_MOTION;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("MCB never confirmed motion");
                inst_substate = MCB_ERROR_LANDING;
            }
        }
        break;
    case MCB_MONITOR_MOTION:
        if (CheckAction(COMMAND_MOTION_STOP)) {
            // todo: verification of motion stop
            ZephyrLogFine("Commanded motion stop");
            inst_substate = MCB_IDLE;
            break;
        }

        if (!mcb_motion_ongoing) {
            SendMCBTM(FINE, "Finished commanded motion");
            inst_substate = MCB_TM_ACK;
            scheduler.AddAction(RESEND_TM, 10);
        }
        break;
    case MCB_TM_ACK:
        if (ACK == TM_ack_flag) {
            inst_substate = MCB_IDLE;
        } else if (NAK == TM_ack_flag || CheckAction(RESEND_TM)) {
            // attempt one resend
            zephyrTX.TM();
            inst_substate = MCB_IDLE;
        }
        break;
    case MCB_ERROR_LANDING:
        log_error("Landed in flight error");
        mcb_motion_ongoing = false;
        mcb_motion = NO_MOTION;
        resend_attempted = false;
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        scheduler.AddAction(RESEND_MCB_LP, MCB_RESEND_TIMEOUT);
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        inst_substate = MCB_ERROR_LOOP;
        break;
    case MCB_ERROR_LOOP:
        log_debug("MCB error loop");
        if (!mcb_low_power && CheckAction(RESEND_MCB_LP)) {
            scheduler.AddAction(RESEND_MCB_LP, MCB_RESEND_TIMEOUT);
            mcbComm.TX_ASCII(MCB_GO_LOW_POWER); // just constantly send
        }

        if (CheckAction(EXIT_ERROR_STATE)) {
            log_nominal("Leaving flight error loop");
            inst_substate = MCB_ENTRY;
        }
        break;
    case MCB_SHUTDOWN_LANDING:
        // prep for shutdown
        log_nominal("Shutdown warning received in MCB");
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        inst_substate = MCB_SHUTDOWN_LOOP;
        break;
    case MCB_SHUTDOWN_LOOP:
        break;
    case MCB_EXIT:
        // perform cleanup
        log_nominal("Exiting Flight from MCB");
        break;
    default:
        log_error("Unknown flight MCB state");
        break;
    }
}