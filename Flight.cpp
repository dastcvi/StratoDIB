/*
 *  Flight.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  
 *  This file implements the FLOATS flight mode.
 */

#include "StratoDIB.h"

enum FLStates_t : uint8_t {
    FL_ENTRY = MODE_ENTRY,
    
    // add any desired states between entry and shutdown
    FL_IDLE,
    FL_GPS_WAIT,
    FL_FTR_START,
    FL_FTR_MEASURE,
    FL_FTR_END,
    FL_ERROR_LANDING,
    FL_ERROR_LOOP,
    
    FL_SHUTDOWN = MODE_SHUTDOWN,
    FL_EXIT = MODE_EXIT
};

char log_array[101] = {0};

// this function is called at the defined rate
//  * when flight mode is entered, it will start in FL_ENTRY state
//  * it is then up to this function to change state as needed by updating the inst_substate variable
//  * on each loop, whichever substate is set will be perfomed
//  * when the mode is changed by the Zephyr, FL_EXIT will automatically be set
//  * it is up to the FL_EXIT logic perform any actions for leaving flight mode
void StratoDIB::FlightMode()
{
    // todo: draw out flight mode state machine
    switch (inst_substate) {
    case FL_ENTRY:
        // perform setup
        log_nominal("Entering FL");
        inst_substate = FL_GPS_WAIT;
        break;
    case FL_GPS_WAIT:
        // wait for the first GPS message from Zephyr to set the time before moving on
        log_debug("Waiting on GPS time");
        if (time_valid) {
            SetAction(START_FTR);
            inst_substate = FL_IDLE;
        }
        break;
    case FL_IDLE:
        if (CheckAction(START_FTR)) {
            inst_substate = FL_FTR_START;
        }
        break;
    case FL_FTR_START:
        log_nominal("Starting FTR");
        FTRon();
        scheduler.AddAction(STOP_FTR, ftr_on_time);
        scheduler.AddAction(START_FTR, ftr_cycle_time);
        inst_substate = FL_FTR_MEASURE;
        break;
    case FL_FTR_MEASURE:
        if (CheckAction(STOP_FTR)) {
            inst_substate = FL_FTR_END;
        }
        break;
    case FL_FTR_END:
        log_nominal("Stopping FTR");
        FTRoff();
        inst_substate = FL_IDLE;
        break;
    case FL_ERROR_LANDING:
        // generic error state for flight mode to go to if any error is detected
        // this state can make sure the ground is informed, and go to the error looop to wait for ground intervention
        // before setting this substate, a ZephyrLogCrit should be sent
        inst_substate = FL_ERROR_LOOP;
        break;
    case FL_ERROR_LOOP:
        // wait for ground
        // todo: add exit condition
        break;
    case FL_SHUTDOWN:
        // prep for shutdown
        log_nominal("Shutdown warning received in FL");
        break;
    case FL_EXIT:
        // perform cleanup
        FTRoff();
        log_nominal("Exiting FL");
        break;
    default:
        // todo: throw error
        ZephyrLogCrit("Unknown substate in FL");
        inst_substate = FL_ERROR_LANDING;
        break;
    }
}