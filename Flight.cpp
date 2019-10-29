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
    FTR_ENTER_IDLE, //Fiber switch to EFU, turn off FTR, configure LTC chip
    FTR_IDLE, //FLOATS is between measurements and listening for EFU errors
    FTR_WARMUP, //Warmup state for FTR3000 (checks FTR status byte)
    FTR_MEASURE, //Measurement operations 
    FTR_EFU, //FLOATS is waiting to recieve EFU telemetry based on a synced time
    FTR_SEND_TELEMETRY,

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
            inst_substate = FTR_ENTER_IDLE;
            EFU_Ready = 0;
        }
        break;

    case FTR_EFU:
        log_debug("Enter EFU State");
        if((CheckAction(LISTEN_EFU)) && (EFU_Received == 0)){// (EFU_Counter <= 60/EFU_Loop)) {
            log_debug("Listening for EFU");
            FTR_Off();
            FiberSwitch_EFU();
            EFU_Counter++;
            scheduler.AddAction(LISTEN_EFU, EFU_Loop);
        }

        if((EFU_Received) && (EFU_Ready == 0)){
            log_debug("EFU received and going into GPS wait");
            inst_substate = FTR_GPS_WAIT;
        }

        /*if(SerialComm gives recieved flag){
            inst_substate = FTR_GPS_WAIT;
        } */

        break;    

    case FTR_ENTER_IDLE:
        log_debug("FTR Enter Idle");
        FTR_Off();
        FiberSwitch_EFU(); 
        resetLtcSpi();
        inst_substate = FTR_IDLE;
        scheduler.AddAction(IDLE_HOUSEKEEPING, Idle_HK_Loop);
        scheduler.AddAction(IDLE_EXIT, Idle_Period);
        //Setup LTC2983

        if (EFU_Ready){
            SetAction(LISTEN_EFU);
            EFU_Counter = 0;
            inst_substate = FTR_EFU;
        }

        break;

    case FTR_IDLE:
        log_debug("FTR Idle");

        if (EFU_Ready){
            //finish idle HK and telemetry, then:
            SetAction(LISTEN_EFU); //goes into EFU substate and starts EFU comms
            EFU_Counter = 0;
            inst_substate = FTR_EFU;
        }

        if (CheckAction(IDLE_HOUSEKEEPING)){

            //Get housekeeping during idle
            //temperature and voltages
            //populate telemetry

            scheduler.AddAction(IDLE_HOUSEKEEPING, Idle_HK_Loop);
           
        }

        if(CheckAction(IDLE_EXIT)){ //could also use a START MEASURE enum here

            inst_substate = FTR_WARMUP; 
            scheduler.AddAction(INITIALIZE_FTR, 2);
            scheduler.AddAction(CHECK_FTR_STATUS, Status_Loop);
            log_debug("Exit Idle");
        }



        break;

    case FTR_WARMUP:

        log_debug("Enter Warmup State");

        if (EFU_Ready){
            SetAction(LISTEN_EFU); //goes into EFU substate and starts EFU comms
            EFU_Counter = 0;
            inst_substate = FTR_EFU;
        }
    

        if(CheckAction(INITIALIZE_FTR)){
            log_debug("FTR Initialized");

            FTR_On();
            //ftr.reset();

        }

        if(CheckAction(CHECK_FTR_STATUS)){
            log_debug("Checking Status");


           //to do: get status

        //     switch(ftr_status){
        //     case FTR_READY: //if status byte shows data ready
        //         log_nominal("stat ready");
        //         Stat_Counter = 0;
        //         scheduler.AddAction(FTR_SCAN, Scan_Loop);
        //         scheduler.AddAction(SEND_TELEM, Measure_Period);
        //         EnterMeasure = 0; 
        //         Scan_Counter = 0;
        //         inst_substate = FTR_MEASURE;
        //         break;

        //     case FTR_NOTREADY: //if status byte doesn't show date ready
        //         scheduler.AddAction(CHECK_FTR_STATUS,Status_Loop);
        //         Stat_Counter ++;
                
        //         if(Stat_Counter >= Stat_Limit) //if counter shows FTR status byte as stale
        //             log_error("stat timeout")
        //             scheduler.AddAction(INITIALIZE_FTR, 2);
        //         break;

        //     case FTR_ERROR: //if status byte is out of bounds
        //         log_error("stat error");
        //         scheduler.AddAction(INITIALIZE_FTR, 2);
        //         break;    

        //     default:
        //         log_error("stat unknown") //if there isn't communication with FTR
        //         scheduler.AddAction(INITIALIZE_FTR, 2);
        //         break; 
        //    }   

        }

        break;

    case FTR_MEASURE:

        log_debug("Enter Measure State");

        if (EFU_Ready){

            //finish measurement and TM then:

            SetAction(LISTEN_EFU); //goes into EFU substate and starts EFU comms
            EFU_Counter = 0;
            inst_substate = FTR_EFU;
        }

        if(EnterMeasure == 0){

            log_debug("First Scan");
            //get and handle first scan
            //get and handle HK data
            Scan_Counter ++;
            EnterMeasure = 1;
            
        }

        if(CheckAction(FTR_SCAN)){
            
            //get and handle scan
            //get and handle HK data
            Scan_Counter ++;
            scheduler.AddAction(FTR_SCAN, Scan_Loop);

        }


        break;    
    case FTR_ERROR_LANDING:
        log_error("Landed in flight error");
        FTR_Off();
        FiberSwitch_EFU();
        inst_substate = FTR_ERROR_LOOP;
        break;
    case FTR_ERROR_LOOP:
        log_debug("FTR error loop");
        FTR_Off();
        FiberSwitch_EFU();

        if (CheckAction(EXIT_ERROR_STATE)) {
            log_nominal("Leaving flight error loop");
            inst_substate = FTR_ENTRY;
        }
        break;
    case FTR_SHUTDOWN_LANDING:
        // prep for shutdown
        log_nominal("Shutdown warning received in FTR");
        FTR_Off();
        FiberSwitch_EFU();
        inst_substate = FTR_SHUTDOWN_LOOP;
        break;
    case FTR_SHUTDOWN_LOOP:
        FTR_Off();
        FiberSwitch_EFU();
        break;
    case FTR_EXIT:
        // perform cleanup
        log_nominal("Exiting Flight from FTR");
        FTR_Off();
        FiberSwitch_EFU();
        break;
    default:
        log_error("Unknown flight FTR state");
        FTR_Off();
        FiberSwitch_EFU();
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