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
#include "Serialize.h"



StratoDIB::StratoDIB()
    : StratoCore(&ZEPHYR_SERIAL, INSTRUMENT)
    , mcbComm(&MCB_SERIAL)
    , ftr(&client, &Serial)
    , ltcManager(LTC_TEMP_CS_PIN, LTC_TEMP_RESET_PIN, THERM_SENSE_CH, RTD_SENSE_CH)
    , efucomm(&Serial3)
{
}

// --------------------------------------------------------
// General instrument functions
// --------------------------------------------------------

// note serial setup occurs in main arduino file
void StratoDIB::InstrumentSetup()
{

    SPI.begin(); //SPI0 for DIB rev B and C

    // for RS232 transceiver
    pinMode(FORCEOFF_232, OUTPUT);
    pinMode(FORCEON_232, OUTPUT);
    digitalWrite(FORCEOFF_232, HIGH);
    digitalWrite(FORCEON_232, HIGH);

    // safe pin required by Zephyr
    pinMode(SAFE_PIN, OUTPUT);
    digitalWrite(SAFE_PIN, LOW);

    //for fiber optic switch
    pinMode(Switch2_EFU, OUTPUT);
    digitalWrite(Switch2_EFU,LOW); //only momentary HIGH needed to activate

    pinMode(Switch2_FTR, OUTPUT);
    digitalWrite(Switch2_FTR,LOW); //only momentary HIGH needed to activate

    pinMode(SwitchStatus_EFU, INPUT); //HIGH/LOW signal for switch state
    pinMode(SwitchStatus_FTR, INPUT); //HIGH/LOW signal for switch state

    //FTR
    pinMode(FTR_POWER, OUTPUT);
    FTR_Off();

    //WIZIO
    pinMode(WizReset, OUTPUT);
    pinMode(WizCS, OUTPUT);
    digitalWrite(WizCS, HIGH); //deselects Wiz820io from SPI0

    //LTC2983
    pinMode(LTC_TEMP_RESET_PIN,OUTPUT);
    pinMode(LTC_TEMP_CS_PIN,OUTPUT);
    ltcManager.channel_assignments[FOTS1_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[FOTS2_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[DCDC_THERM_CH]   = THERMISTOR_44006;
    ltcManager.channel_assignments[SPARE_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[OAT_PRT1_RTD_CH] = RTD_PT_100;
    ltcManager.channel_assignments[OAT_PRT2_RTD_CH] = RTD_PT_100;
    ltcManager.InitializeAndConfigure();
    digitalWrite(LTC_TEMP_RESET_PIN, HIGH); //deselects LTC from SPI0

      // ADC Setup
    pinMode(VMON_3V3,INPUT);
    pinMode(VMON_12V,INPUT);
    pinMode(VMON_5V,INPUT);
    pinMode(VMON_15V,INPUT);
    analogReference(EXTERNAL);

    efucomm.AssignBinaryRXBuffer(bin_rx, 2048);
    mcbComm.AssignBinaryRXBuffer(binary_mcb, 50);
}

void StratoDIB::InstrumentLoop()
{
    WatchFlags();
    EFUWatch();
}



// --------------------------------------------------------
// Telecommand handler
// --------------------------------------------------------

// The telecommand handler must return ACK/NAK
bool StratoDIB::TCHandler(Telecommand_t telecommand)
{
    String dbg_msg = "";

    switch (telecommand) {
    // MCB Telecommands -----------------------------------
    case DEPLOYx:
        deploy_length = mcbParam.deployLen;
        SetAction(COMMAND_REEL_OUT); // will be ignored if wrong mode
        break;
    case DEPLOYv:
        deploy_velocity = mcbParam.deployVel;
        break;
    case DEPLOYa:
        mcbComm.TX_Out_Acc(mcbParam.deployAcc); // todo: verification + ack
        break;
    case RETRACTx:
        retract_length = mcbParam.retractLen;
        SetAction(COMMAND_REEL_IN); // will be ignored if wrong mode
        break;
    case RETRACTv:
        retract_velocity = mcbParam.retractVel;
        break;
    case RETRACTa:
        mcbComm.TX_In_Acc(mcbParam.retractAcc); // todo: verification + ack
        break;
    case DOCKx:
        dock_length = mcbParam.dockLen;
        SetAction(COMMAND_DOCK); // will be ignored if wrong mode
        break;
    case DOCKv:
        dock_velocity = mcbParam.dockVel;
        break;
    case DOCKa:
        mcbComm.TX_Dock_Acc(mcbParam.dockAcc); // todo: verification + ack
        break;
    case FULLRETRACT:
        // todo: determine implementation
        break;
    case CANCELMOTION:
        mcbComm.TX_ASCII(MCB_CANCEL_MOTION); // no matter what, attempt to send (irrespective of mode)
        SetAction(COMMAND_MOTION_STOP);
        break;
    case ZEROREEL:
        mcbComm.TX_ASCII(MCB_ZERO_REEL); // todo: verification + ack
        break;
    case TEMPLIMITS:
        if (!mcbComm.TX_Temp_Limits(mcbParam.tempLimits[0],mcbParam.tempLimits[1],mcbParam.tempLimits[2],mcbParam.tempLimits[3],mcbParam.tempLimits[4],mcbParam.tempLimits[5])) {
            ZephyrLogWarn("Error sending temperature limits to MCB");
        }
        break;
    case TORQUELIMITS:
        if (!mcbComm.TX_Torque_Limits(mcbParam.torqueLimits[0],mcbParam.torqueLimits[1])) {
            ZephyrLogWarn("Error sending torque limits to MCB");
        }
        break;
    case CURRLIMITS:
        if (!mcbComm.TX_Curr_Limits(mcbParam.currLimits[0],mcbParam.currLimits[1])) {
            ZephyrLogWarn("Error sending curr limits to MCB");
        }
        break;
    // Non-MCB Telecommands -------------------------------
    case GOFTRFLIGHT:
        flight_submode = FTR_SUBMODE;
        ZephyrLogFine("Set flight sub-mode to FTR");
        break;
    case GOMCBFLIGHT:
        flight_submode = MCB_SUBMODE;
        ZephyrLogFine("Set flight sub-mode to MCB");
        break;
    case FTRONTIME:
        Measure_Period = dibParam.ftrOnTime;
        break;
    case FTRCYCLETIME:
        Idle_Period = dibParam.ftrCycleTime;
        break;
    case SETDIBHKPERIOD:
        HK_Loop = dibParam.hkPeriod;
        break;
    case FTRSTATUSLIMIT:
        Stat_Limit = dibParam.statusLimit;
        break;
    case RAMANLEN:
        RamanLength = dibParam.ramanScanLength;
        break;
    case SETMEASURETYPE:
        measure_type = dibParam.ftrMeasureType;
        Burst_Limit = dibParam.ftrBurstLim;
        break;
    case EXITERROR:
        SetAction(EXIT_ERROR_STATE);
        break;
    //Raman length telecommand needed

    default:
        log_error("Unknown TC received");
        // error case here
        break;
    }
    return true;
}

// --------------------------------------------------------
// Action handler and action flag helper functions
// --------------------------------------------------------

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

// --------------------------------------------------------
// MCB Message Router + Handlers
// --------------------------------------------------------

void StratoDIB::RunMCBRouter()
{
    SerialMessage_t rx_msg = mcbComm.RX();

    while (NO_MESSAGE != rx_msg) {
        if (ASCII_MESSAGE == rx_msg) {
            HandleMCBASCII();
        } else if (ACK_MESSAGE == rx_msg) {
            HandleMCBAck();
        } else if (BIN_MESSAGE == rx_msg) {
            HandleMCBBin();
        } else {
            log_error("Unknown message type from MCB");
        }

        rx_msg = mcbComm.RX();
    }
}

void StratoDIB::HandleMCBASCII()
{
    switch (mcbComm.ascii_rx.msg_id) {
    case MCB_MOTION_FINISHED:
        log_nominal("MCB motion finished"); // state machine will report to Zephyr
        mcb_motion_ongoing = false;
        break;
    case MCB_ERROR:
        if (mcbComm.RX_Error(log_array, LOG_ARRAY_SIZE)) {
            ZephyrLogCrit(log_array);
            inst_substate = MODE_ERROR;
        }
        break;
    case MCB_MOTION_FAULT:
        if (mcbComm.RX_Motion_Fault(motion_fault, motion_fault+1, motion_fault+2, motion_fault+3,
                                    motion_fault+4, motion_fault+5, motion_fault+6, motion_fault+7)) {
            mcb_motion_ongoing = false;
            snprintf(log_array, LOG_ARRAY_SIZE, "MCB Fault: %x,%x,%x,%x,%x,%x,%x,%x", motion_fault[0], motion_fault[1],
                     motion_fault[2], motion_fault[3], motion_fault[4], motion_fault[5], motion_fault[6], motion_fault[7]);
            SendMCBTM(CRIT, log_array);
            inst_substate = MODE_ERROR;
        } else {
            mcb_motion_ongoing = false;
            SendMCBTM(CRIT, "MCB Fault: error receiving parameters");
            inst_substate = MODE_ERROR;
        }
        break;
    default:
        log_error("Unknown MCB ASCII message received");
        break;
    }
}

void StratoDIB::HandleMCBAck()
{
    switch (mcbComm.ack_id) {
    case MCB_GO_LOW_POWER:
        log_nominal("MCB in low power");
        mcb_low_power = true;
        break;
    case MCB_REEL_IN:
        if (MOTION_REEL_IN == mcb_motion) NoteProfileStart();
        break;
    case MCB_REEL_OUT:
        if (MOTION_REEL_OUT == mcb_motion) NoteProfileStart();
        break;
    case MCB_DOCK:
        if (MOTION_DOCK == mcb_motion) NoteProfileStart();
        break;
    case MCB_IN_ACC:
        ZephyrLogFine("MCB acked retract acc");
        break;
    case MCB_OUT_ACC:
        ZephyrLogFine("MCB acked deploy acc");
        break;
    case MCB_DOCK_ACC:
        ZephyrLogFine("MCB acked dock acc");
        break;
    case MCB_ZERO_REEL:
        ZephyrLogFine("MCB acked zero reel");
        break;
    case MCB_TEMP_LIMITS:
        ZephyrLogFine("MCB acked temp limits");
        break;
    case MCB_TORQUE_LIMITS:
        ZephyrLogFine("MCB acked torque limits");
        break;
    case MCB_CURR_LIMITS:
        ZephyrLogFine("MCB acked curr limits");
        break;
    default:
        log_error("Unknown MCB ack received");
        break;
    }
}

void StratoDIB::HandleMCBBin()
{
    float reel_pos = 0;
    uint16_t reel_pos_index = 21; // todo: don't hard-code this

    switch (mcbComm.binary_rx.bin_id) {
    case MCB_MOTION_TM:
        if (BufferGetFloat(&reel_pos, mcbComm.binary_rx.bin_buffer, mcbComm.binary_rx.bin_length, &reel_pos_index)) {
            snprintf(log_array, 101, "Reel position: %ld", (int32_t) reel_pos);
            log_nominal(log_array);
        } else {
            log_nominal("Recieved MCB bin: unable to read position");
        }
        AddMCBTM();
        break;
    default:
        log_error("Unknown MCB bin received");
    }
}

// --------------------------------------------------------
// Profile helpers
// --------------------------------------------------------

bool StratoDIB::StartMCBMotion()
{
    bool success = false;

    switch (mcb_motion) {
    case MOTION_REEL_IN:
        snprintf(log_array, LOG_ARRAY_SIZE, "Retracting %0.1f revs", retract_length);
        success = mcbComm.TX_Reel_In(retract_length, retract_velocity); // todo: verification
        break;
    case MOTION_REEL_OUT:
        snprintf(log_array, LOG_ARRAY_SIZE, "Deploying %0.1f revs", deploy_length);
        success = mcbComm.TX_Reel_Out(deploy_length, deploy_velocity); // todo: verification
        break;
    case MOTION_DOCK:
        snprintf(log_array, LOG_ARRAY_SIZE, "Docking %0.1f revs", dock_length);
        success = mcbComm.TX_Dock(dock_length, dock_velocity); // todo: verification
        break;
    case MOTION_UNDOCK:
    default:
        mcb_motion = NO_MOTION;
        log_error("Unknown motion type to start");
        return false;
    }

    ZephyrLogFine(log_array);

    return success;
}

void StratoDIB::AddMCBTM()
{
    // make sure it's the correct size
    if (mcbComm.binary_rx.bin_length != MOTION_TM_SIZE) {
        log_error("invalid motion TM size");
        return;
    }

    // sync byte
    if (!zephyrTX.addTm((uint8_t) 0xA5)) {
        log_error("unable to add sync byte to MCB TM buffer");
        return;
    }

    // tenths of seconds since start
    if (!zephyrTX.addTm((uint16_t) ((millis() - profile_start) / 100))) {
        log_error("unable to add seconds bytes to MCB TM buffer");
        return;
    }

    // add each byte of data to the message
    for (int i = 0; i < MOTION_TM_SIZE; i++) {
        if (!zephyrTX.addTm(mcbComm.binary_rx.bin_buffer[i])) {
            log_error("unable to add data byte to MCB TM buffer");
            return;
        }
    }
}

void StratoDIB::NoteProfileStart()
{
    mcb_motion_ongoing = true;
    profile_start = millis();
    zephyrTX.clearTm(); // empty the TM buffer for incoming MCB motion data

    // MCB TM Header
    zephyrTX.addTm((uint32_t) now()); // as a header, add the current seconds since epoch
    // add to header: profile type, auto vs. manual, auto trigger?
}

void StratoDIB::SendMCBTM(StateFlag_t state_flag, String message)
{
    // use only the first flag to report the motion
    zephyrTX.setStateDetails(1, message);
    zephyrTX.setStateFlagValue(1, state_flag);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    if (!WriteFileTM("MCB")) {
        log_error("Unable to write MCB TM to SD file");
    }
}

// --------------------------------------------------------
// Hardware Operation Functions
// --------------------------------------------------------

void StratoDIB::FTR_On(){
    digitalWrite(FTR_POWER, LOW);
    log_debug("FTR Powered On");
}

void StratoDIB::FTR_Off(){
    digitalWrite(FTR_POWER,HIGH);
    log_debug("FTR Powered Off");
}

void StratoDIB::FiberSwitch_EFU(){
     uint16_t count = 0;
        digitalWrite(Switch2_EFU,HIGH);
        while(digitalRead(SwitchStatus_EFU)){
          delay(1);
          count++;
          if(count >= 700){
                //To do: Send Error message for timeout
                log_debug("Fiber Switch to EFU NACK");
                log_error("EFU Switch NACK");
            break;
          }
        }
        digitalWrite(Switch2_EFU,LOW);

}

void StratoDIB::FiberSwitch_FTR(){
     uint16_t count = 0;
        digitalWrite(Switch2_FTR,HIGH);
        while(digitalRead(SwitchStatus_EFU)){
          delay(1);
          count++;
          if(count >= 700){
                //To do: Send Error message for timeout
                log_debug("Fiber Switch to FTR NACK");
                log_error("FTR Switch NACK");
            break;
          }
        }
        digitalWrite(Switch2_FTR,LOW);
}

void StratoDIB::FTRStatusReport(uint8_t status){

    if((status == 0x27) || (status == 0x00) || (status == 0x07)){
        ftr_status = FTR_NOTREADY;
    }

    else if((status == 0x57) || (status == 0x67) || (status == 0x17)) {
        ftr_status = FTR_READY;
    }

    else {
        ftr_status = FTR_ERROR;
    }
}

void  StratoDIB::resetLtcSpi() {
        SPI0_SR |= SPI_DISABLE;
        SPI0_CTAR0 = 0x38004005;
        SPI0_SR &= ~(SPI_DISABLE);
}

void  StratoDIB::LTCSetup(){

    log_debug("LTC Configured");
    ltcManager.channel_assignments[FOTS1_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[FOTS2_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[DCDC_THERM_CH]   = THERMISTOR_44006;
    ltcManager.channel_assignments[SPARE_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[OAT_PRT1_RTD_CH] = RTD_PT_100;
    ltcManager.channel_assignments[OAT_PRT2_RTD_CH] = RTD_PT_100;
    ltcManager.InitializeAndConfigure();
    digitalWrite(LTC_TEMP_RESET_PIN, HIGH);

    ltcManager.connect();
}

void StratoDIB::ReadFullTemps() {
  resetLtcSpi();
  digitalWrite(14, HIGH);
  noInterrupts();
  ltcManager.WakeUp();
  uint16_t ret = ltcManager.CheckStatusReg();

  if ((ret == 0) || (ret == 0xFF)) {
    //Serial.println("Error reading status register, resetting LTC");
    ltcManager.channel_assignments[FOTS1_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[FOTS2_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[DCDC_THERM_CH]   = THERMISTOR_44006;
    ltcManager.channel_assignments[SPARE_THERM_CH]  = THERMISTOR_44006;
    ltcManager.channel_assignments[OAT_PRT1_RTD_CH] = RTD_PT_100;
    ltcManager.channel_assignments[OAT_PRT2_RTD_CH] = RTD_PT_100;
    ltcManager.InitializeAndConfigure();
    ret = ltcManager.CheckStatusReg();
    if ((ret == 0 || ret == 0xFF)) {
      log_error("LTC Reset failed");
      //return 1;
    }
  }

  FOTS1Therm = ltcManager.MeasureChannel(FOTS1_THERM_CH);
  FOTS2Therm = ltcManager.MeasureChannel(FOTS2_THERM_CH);
  DC_DC_Therm = ltcManager.MeasureChannel(DCDC_THERM_CH);
  SpareTherm = ltcManager.MeasureChannel(SPARE_THERM_CH);
  RTD1 = ltcManager.MeasureChannel(OAT_PRT1_RTD_CH);
  RTD2 = ltcManager.MeasureChannel(OAT_PRT2_RTD_CH);
  interrupts();
  //return 0;
}

void StratoDIB::ReadVoltages(){

  float val = analogRead(VMON_15V);
  //V_Zephyr = (val*(3.0/1023.0))*((10000.0+995.0)/995.0);
  V_Zephyr = (val*(3.3/1023.0))*((10000.0+1100.0)/1100.0);
  val = analogRead(VMON_3V3);
  V_3v3 = (val*(3.3/1023.0))*((10000.0+10000.0)/10000.0);
  val = analogRead(VMON_5V);
  V_5TX = (val*(3.3/1023.0))*((10000.0+1100.0)/1100.0);
  val = analogRead(VMON_12V);
  V_12FTR = (val*(3.3/1023.0))*((10000.0+1100.0)/1100.0);
}

// --------------------------------------------------------
// EFu Message Router + Handlers
// --------------------------------------------------------


void StratoDIB::EFUWatch(){

    if(minute()==46){
        EFU_Ready = true;
    }

    else{
        EFU_Ready = false;
    }
}

void StratoDIB::RunEFURouter(){

    SerialMessage_t EFU_msg = efucomm.RX();

    while (NO_MESSAGE != EFU_msg) {
        if (ASCII_MESSAGE == EFU_msg) {

        } else if (ACK_MESSAGE == EFU_msg) {

        } else if (BIN_MESSAGE == EFU_msg) {
            HandleEFUBin();
        } else {
            log_error("Unknown message type from EFU");
        }

        EFU_msg = efucomm.RX();
    }

}

void StratoDIB:: HandleEFUBin(){

    //if(efucomm.binary_rx.checksum_valid){ // to do: possible problem with EFU side checksum

        switch (efucomm.binary_rx.bin_id) {
        case EFU_DATA_RECORD:
            AddEFUTM();
            break;
        default:
            log_error("Unknown EFU bin received");
        }

        EFU_Received = 1;
        log_nominal("EFU bin handled and sent via TM");

        //if checksum is good send TM packet
        zephyrTX.setStateFlagValue(1, FINE);
        zephyrTX.setStateDetails(1, "EFU1"); //to do: add FLOATS HK here
        zephyrTX.setStateFlagValue(2, FINE);
        zephyrTX.setStateDetails(2, "EFU2"); //to do: add FLOATS HK here
        zephyrTX.TM();
//}


}

void StratoDIB::AddEFUTM()
{
   // add each byte of data to the message
    for (int i = 0; i < efucomm.binary_rx.bin_length; i++) {
        if (!zephyrTX.addTm(efucomm.binary_rx.bin_buffer[i])) {
            log_error("unable to add data byte to EFU TM buffer");
            return;
        }
    }
}


// ------------------------------------------------------
// Handle FTR3000 Data
//-------------------------------------------------------

void StratoDIB::XMLHeader(){

    ReadFullTemps();
    ReadVoltages();

    //float unixtime = now();

    String Message = "";
    bool flag1 = true;
    bool flag2 = true;

    /* Check the values for the TM message header */
    if ((DC_DC_Therm > 60.0) || (DC_DC_Therm < -30.0))
        flag1 = false;
    if ((FOTS1Therm > 60.0) || (FOTS1Therm < -30.0))
        flag1 = false;
    if ((FOTS2Therm > 60.0) || (FOTS2Therm < -30.0))
        flag1 = false;


    /*Check Voltages are in range */
    if ((V_Zephyr > 19.0) || (V_Zephyr < 12.0))
        flag2 = false;
    if((V_12FTR>13.0) || (V_12FTR<11.5))
        flag2 = false;

    //First Field
    if (flag1) {
        zephyrTX.setStateFlagValue(1, FINE);
    } else {
        zephyrTX.setStateFlagValue(1, WARN);
    }

    Message.concat(now());
    Message.concat(',');
    Message.concat(FOTS1Therm);
    Message.concat(',');
    Message.concat(FOTS2Therm);
    Message.concat(',');
    Message.concat(DC_DC_Therm);
    Message.concat(',');
    Message.concat(SpareTherm);
    Message.concat(',');
    Message.concat(RTD1);
    Message.concat(',');
    Message.concat(RTD2);
    zephyrTX.setStateDetails(1, Message);
    Message = "";

    // Second Field
    if (flag2) {
        zephyrTX.setStateFlagValue(2, FINE);
    } else {
        zephyrTX.setStateFlagValue(2, WARN);
    }

    Message.concat(V_Zephyr);
    Message.concat(',');
    Message.concat(V_3v3);
    Message.concat(',');
    Message.concat(V_5TX);
    Message.concat(',');
    Message.concat(V_12FTR);
    zephyrTX.setStateDetails(2, Message);
    Message = "";


}

