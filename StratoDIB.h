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
#include "EFUComm.h"
#include <LTC2983Manager.h>
#include <FTR3000.h>
#include <Ethernet.h> 


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

    //FTR actions
    HOUSEKEEPING,
    IDLE_EXIT,
    POWERON_FTR,
    CONFIGURE_FTR,
    FTR_SCAN,
    CHECK_FTR_STATUS,
    START_EFU,
    LISTEN_EFU,
    BUILD_TELEM,
    SEND_TELEM,
    
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

enum FTRStatus_t : uint8_t {
    ENTERSTAT,
    FTR_READY,
    FTR_NOTREADY,
    FTR_ERROR
};

enum FTRMeasureType_t : uint8_t {
    BURST,
    AVERAGE,
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

    void RunEFURouter();

private:
    // instances
    MCBComm mcbComm;
    FTR ftr;
    LTC2983Manager ltcManager;
    EFUComm efucomm;
    EthernetClient client;

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

    //Hardware Operation functions
    void FTR_On(); //switches FTR power on
    void FTR_Off(); //switches FTR power off
    void FiberSwitch_EFU(); //Fiber switch actuated to EFU side
    void FiberSwitch_FTR(); //Fiber switch actuacted to FTR side
    void resetLtcSpi(); //SPI0 reset for use with LTC2983
    void LTCSetup(); //Sets up LTC channels
    void ReadFullTemps(); //gets full temperature string
    void ReadVoltages(); //get voltages
    void FTRStatusReport(uint8_t); //sets ftr_status variable based on FTR status byte retrieved with FTR3000 library

    void EFUWatch(); //Sets EFU ready flag when predetermined minute is reached

    //Timing Variables
    uint16_t Measure_Period = 10*60; //10 minutes nominally
    uint16_t HK_Loop = 120; //number of seconds between idle HK data retreival
    uint16_t Idle_Period = 5*60; //Should be opposite duty cycle of measure period minus Start_EFU_Period telemetry period
    uint16_t Stat_Limit = 300/20; //number of FTR status requests before timeout and FTR3000 reset (nominally 5mins/20second requests = 15)
    
    int Status_Loop = 20; // number of seconds between FTR status requests
    int Scan_Loop = 120; //number of seconds per scan. FTR3000 scan time hardset to 2minutes.
    int Stat_Counter = 0; //number of times status is requested prior to entering measurement state or resetting FTR3000
    int EFU_Loop = 15; //number of seconds between EFU retrieval attempts during EFU telemetry state
    int EFU_Counter = 0;
    int Scan_Counter = 0; //number of 2 minute FTR3000 scans attempted
    uint8_t EthernetCount = 0; 
    int Burst_Counter = 0;
    uint8_t Burst_Limit = 8; //8, 2min scans = 16 minutes
    
    //Operational Flags
    bool EFU_Ready = false; 
    bool EFU_Received = false;
    bool EnterMeasure = 0;
    
    //State variables
    uint8_t ftr_status;
    uint8_t measure_type = AVERAGE;

    //Data Variables
    float FOTS1Therm;
    float FOTS2Therm;
    float DC_DC_Therm;
    float SpareTherm;
    float RTD1;
    float RTD2;
    float V_Zephyr;
    float V_3v3;
    float V_5TX;
    float V_12FTR;

    //FTR Arrays
    uint16_t RamanLength = 1850;
    byte RamanBin[17000]; //Binary array that contains raw FTR scan
    uint16_t Stokes[2200]; //instant stokes scan from RamanBin
    uint16_t Astokes[2200]; //instant antistokes scan from RamanBin
    uint16_t StokesElements[2200]; //number of good scans per array point for stokes averaging
    uint16_t AstokesElements[2200]; //number of good scans per array point for antistokes averaging
    float StokesAvg[2200]; //stokes CO-add values that will be averaged using elements array and passed as TM
    float AStokesAvg[2200]; //antistokes Co-add values that will be averaging using the elements array and passed as TM
    uint16_t Stokes_Counter = 0;
    uint16_t Astokes_Counter = 0;
    

    //Route and Handle messages from EFUComm
    void HandleEFUBin();
    void AddEFUTM();
    uint8_t bin_rx[2048];

    //Handle FTR cans and telemetry
    void HandleFTRBin();
    void XMLHeader();

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