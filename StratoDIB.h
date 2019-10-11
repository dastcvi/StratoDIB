/*
 *  StratoDIB.h
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  This file declares an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as the overarching class
 *  for the FLOATS Data Interface Board, or DIB.
 */

#ifndef STRATODIB_H
#define STRATODIB_H

#include "StratoCore.h"
#include "DIBHardware.h"
#include "DIBBufferGuard.h"
#include "MCBComm.h"
#include "PUComm.h"

#define INSTRUMENT      FLOATS

// number of loops before a flag becomes stale and is reset
#define FLAG_STALE      2

#define MCB_RESEND_TIMEOUT      10
#define ZEPHYR_RESEND_TIMEOUT   60

#define LOG_ARRAY_SIZE  101

// todo: update naming to be more unique (ie. ACT_ prefix)
enum ScheduleAction_t : uint8_t {
    NO_ACTION = NO_SCHEDULED_ACTION,

    // scheduled actions
    SEND_IMR,
    RESEND_SAFETY,
    RESEND_MCB_LP,
    RESEND_MOTION_COMMAND,
    RESEND_TM,
    EXIT_ERROR_STATE,

    // internal command actions
    COMMAND_REEL_OUT,
    COMMAND_REEL_IN,
    COMMAND_DOCK,
    COMMAND_MOTION_STOP,

    // used for tracking
    NUM_ACTIONS
};

enum MCBMotion_t : uint8_t {
    NO_MOTION,
    MOTION_REEL_IN,
    MOTION_REEL_OUT,
    MOTION_DOCK,
    MOTION_UNDOCK
};

enum FlightSubMode_t : uint8_t {
    FTR_SUBMODE = 0,
    MCB_SUBMODE = 1
};

class StratoDIB : public StratoCore {
public:
    StratoDIB();
    ~StratoDIB() { };

    // called before the main loop begins
    void InstrumentSetup();

    // called at the end of each main loop
    void InstrumentLoop();

    // called in each main loop
    void RunMCBRouter();

private:
    // internal serial interface object for the MCB
    MCBComm mcbComm;

    // Mode functions (implemented in unique source files)
    void StandbyMode();
    void FlightMode();
    void LowPowerMode();
    void SafetyMode();
    void EndOfFlightMode();

    // Sub-modes for DIB FlightMode
    void FlightFTR();
    void FlightMCB();
    FlightSubMode_t flight_submode = FTR_SUBMODE; // reboot default

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

    // Handle messages from the MCB
    void HandleMCBASCII();
    void HandleMCBAck();
    void HandleMCBBin();
    uint8_t binary_mcb[50];

    // Start any type of MCB motion
    bool StartMCBMotion();

    // Add an MCB motion TM packet to the binary TM buffer
    void AddMCBTM();

    // Set variables and TM buffer after a profile starts
    void NoteProfileStart();

    // Send a telemetry packet with MCB binary info
    void SendMCBTM(StateFlag_t state_flag, String message);

    ActionFlag_t action_flags[NUM_ACTIONS] = {{0}}; // initialize all flags to false

    // flags for MCB state tracking
    bool mcb_low_power = false;
    bool mcb_motion_ongoing = false;

    // uint32_t start time of the current profile in millis
    uint32_t profile_start = 0;

    // tracks the current type of motion
    MCBMotion_t mcb_motion = NO_MOTION;

    // tracks if a resend of any message has already been attempted
    bool resend_attempted = false;

    // current profile parameters
    float deploy_length = 0.0f;
    float retract_length = 0.0f;
    float dock_length = 0.0f;
    float deploy_velocity = 80.0f;
    float retract_velocity = 80.0f;
    float dock_velocity = 20.0f;

    // array of error values for MCB motion fault
    uint16_t motion_fault[8] = {0};

    // keep a statically allocated array for creating up to 100 char TM state messages
    char log_array[LOG_ARRAY_SIZE] = {0};
};

#endif /* STRATODIB_H */