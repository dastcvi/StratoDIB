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
        if((CheckAction(LISTEN_EFU)) && (EFU_Received == 0)){ //if Listen EFU loop is scheduled and a EFU telemetry packet hasn't been successfully received
            log_debug("Listening for EFU");
            //EFUrouter is always running in Main loop and will only make EFU comms when in this configuration
            FTR_Off();
            FiberSwitch_EFU();
            scheduler.AddAction(LISTEN_EFU, EFU_Loop);
        }

        if((EFU_Received) && (EFU_Ready == 0)){ //if the EFU packet was received successfully with EFUrouter and EFUWatch minute expires
            log_debug("EFU received and going into GPS wait");
            inst_substate = FTR_GPS_WAIT;
        }

        if(EFU_Ready==0){ //if EFUWatch expires but EFU telemetry was not received
            log_debug("EFU not received, EFU telem timeout");
            inst_substate = FTR_GPS_WAIT;
        }
        break;   

    case FTR_ENTER_IDLE:
        log_debug("FTR Enter Idle");
        FTR_Off();
        FiberSwitch_EFU(); 
        inst_substate = FTR_IDLE;
        SetAction(HOUSEKEEPING);
        scheduler.AddAction(IDLE_EXIT, Idle_Period);
        
        if (EFU_Ready){
            SetAction(LISTEN_EFU);
            inst_substate = FTR_EFU;
        }
        break;

    case FTR_IDLE:
        log_debug("FTR Idle");

        if (EFU_Ready){
            SetAction(LISTEN_EFU); //goes into EFU substate and starts EFU comms
            inst_substate = FTR_EFU;
        }

        if (CheckAction(HOUSEKEEPING)){
            log_debug("Sending Housekeeping");
            XMLHeader();
            scheduler.AddAction(HOUSEKEEPING, HK_Loop);
        }

        if(CheckAction(IDLE_EXIT)){ //could also use a START MEASURE enum here

            inst_substate = FTR_WARMUP;
            SetAction(POWERON_FTR);
            log_debug("Exit Idle");
        }



        break;

    case FTR_WARMUP:

        log_debug("Enter Warmup State");

        if (EFU_Ready){
            SetAction(LISTEN_EFU); //goes into EFU substate and starts EFU comms
            inst_substate = FTR_EFU;
        }

        if (CheckAction(HOUSEKEEPING)){
            log_debug("Sending Housekeeping");
            XMLHeader();
            scheduler.AddAction(HOUSEKEEPING, HK_Loop);
        }

        if(CheckAction(POWERON_FTR)){
            log_debug("FTR powered on and fiber switched");
            FTR_Off();
            delay(100);
            FTR_On();
            FiberSwitch_FTR();
            ftr.resetFtrSpi();
            scheduler.AddAction(CONFIGURE_FTR, 6);  
        }
    
        if(CheckAction(CONFIGURE_FTR)){
            log_debug("Configuring FTR");
            if(ftr.start()){ 
                scheduler.AddAction(CHECK_FTR_STATUS, Status_Loop); 
                Stat_Counter = 0;
            }   
        }

        if(CheckAction(CHECK_FTR_STATUS)){
            log_debug("Checking Status");
            FTRStatusReport(ftr.status());
            
            switch(ftr_status){
            case FTR_READY: //if status byte shows data ready
                log_nominal("stat ready");
                SetAction(FTR_SCAN);
                scheduler.AddAction(SEND_TELEM, Measure_Period);
                inst_substate = FTR_MEASURE;
                Stokes_Counter = 0;
                Astokes_Counter = 0;
                break;

            case FTR_NOTREADY: //if status byte doesn't show data ready
                scheduler.AddAction(CHECK_FTR_STATUS,Status_Loop);
                Stat_Counter ++;
                
                if(Stat_Counter >= Stat_Limit) //if counter shows FTR status byte as stale
                    log_error("stat timeout");
                    SetAction(POWERON_FTR);
                break;

            case FTR_ERROR: //if status byte is out of bounds
                log_error("stat error");
                SetAction(POWERON_FTR);
                break;    

            default:
                log_error("stat unknown"); //if there isn't communication with FTR
                SetAction(POWERON_FTR);
                break; 
           }   

        }
        break;

    case FTR_MEASURE:

        log_debug("Enter Measure State");

        if (EFU_Ready){

            XMLHeader();
            ftr.RamanAverage(StokesElements, StokesAvg);
            zephyrTX.addTm(Stokes_Counter);
            zephyrTX.addTm(StokesAvg, 7200);
            ftr.RamanAverage(AstokesElements,AStokesAvg);
            zephyrTX.addTm(Astokes_Counter);
            zephyrTX.addTm(AStokesAvg, 7200);
            zephyrTX.TM();
            ftr.ClearArray(StokesAvg);
            ftr.ClearArray(AStokesAvg);

            SetAction(LISTEN_EFU); //goes into EFU substate and starts EFU comms
            inst_substate = FTR_EFU;
        }

        if (CheckAction(HOUSEKEEPING)){
            log_debug("Sending Housekeeping");
            XMLHeader();
            scheduler.AddAction(HOUSEKEEPING, HK_Loop);
        }

        if(CheckAction(FTR_SCAN)){
            
            ftr.resetFtrSpi();
            ftr.readRaman(RamanBin);
            ftr.BintoArray(RamanBin, Stokes, Astokes);
            ftr.ClearArray(RamanBin);
            Stokes_Counter += ftr.RamanCoAdd(StokesElements, Stokes, StokesAvg);
            ftr.ClearArray(Stokes);
            Astokes_Counter += ftr.RamanCoAdd(AstokesElements, Astokes, AStokesAvg);
            ftr.ClearArray(Astokes);

            scheduler.AddAction(FTR_SCAN, Scan_Loop);
        }

        if(CheckAction(SEND_TELEM)){

            XMLHeader();
            ftr.RamanAverage(StokesElements, StokesAvg);
            zephyrTX.addTm(Stokes_Counter);
            zephyrTX.addTm(StokesAvg, 7200);
            ftr.RamanAverage(AstokesElements,AStokesAvg);
            zephyrTX.addTm(Astokes_Counter);
            zephyrTX.addTm(AStokesAvg, 7200);
            zephyrTX.TM();
            ftr.ClearArray(StokesAvg);
            ftr.ClearArray(AStokesAvg);

            inst_substate = FTR_ENTRY;    

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