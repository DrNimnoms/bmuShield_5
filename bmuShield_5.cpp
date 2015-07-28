/************************************
REVISION HISTORY
$Revision: 1000 $
$Date: 2015-06-10 

Copyright (c) 2015, General Atomics (GA)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright 2015 General Atomics (GA)
***********************************************************/

/*!******************************************************************************************************
* bmuShield_5_2.cpp
* Created by Nima Ghods
* Data: 6/10/2015
* setsup miscellaneous functions for BMUshielv5.X
*********************************************************************************************************/

#include <stdint.h>
#include <Arduino.h>
#include <Metro.h>
#include "bmuShield_5.h"

/*!******************************************************************************************************************
  bmes() constructur
  initializes all the internal bmes values
 ******************************************************************************************************************/
bmuShield::bmuShield()
{
	// bmu state
	myMode = SYS_OFF;
	myFlag = 0;
	myFlagOverride = 0;

  // BMU measurement Variables
	myVoltage = 0;           // total half-string voltage read from ADC
	for(int i=1;i<5;i++) myPressure[i]= 0;          // pressure sensor reading
  myPressCount= 0;
	myPresRate = 0;          // filtered pressure rate
  myMaxPressRate = 0;
	for(int i=1;i<5;i++) myPressureExt[i] =0;       // external pressure sensor reading
	myPresExtRate =0;       // filtered external pressure rate
  myMaxPressExtRate = 0;
	myCur0 =0.3;           // offset value read from the LEM sensor
	myCurrent =0;           // value read from the LEM sensor
	myFwLeak =0;          // front leak sensor
	myBwLeak =0;          // back leak sensor
	myRelay1fb =0;   // contactor 1 feedback
	myRelay2fb =0;   // contactor 2 feedback

	// BMU FLAG LIMITS
    myPresRateHigh = 0.15;    //High pressure rate limit
    myPresHighLimit = 2.5;    //High pressure limit
    myPresLowLimit = 0.5;     //Low Pressure limit
    myInCurLimit = 2.0;      //current in limit during ON mode
    myHighChargeCur = 92.0;     //high current in limit in CHARGE mode
    myLowChargeCur = -2.0;      //current out limit in CHARGE mode
    myInOutCurLimit = 1.0;     // current in or out limit during OFF or BALANCE mode
    myVolBmuMismatch = 3.0;       //voltage mismatch limit between calculated and measured total voltage by BMU
    myTimeOutLimit = 8.0;	// the hours in charge or balance before timing out
    myVolLowBalAlarm  = 3.7;   // the voltage at which the system will not go in to balancing mode
    myBalRecLimit = 3.9;      // minimum voltage limit for recommending balancing
    myBalRecVol = 0.050;       // voltage difference at which balancing will be recommended
    myVolTolerance = 0.003;   // the max voltage difference that the virtual cells will have at the end of balancing
    myDoneCur = 4.45;         //the current at which the charging is called done
    myChg2Vol=0;

  // BMU actuators Variables
	myRelayDelay = false; // a bool to saprate the relays turning on 
	myRelayOn =false;   // relays 1 nd 2
	myExtDO1 =false;   // external digatal output 1
	myExtDO2 =false;   // external digatal output 2

}

/*!******************************************************************************************************************
  \brief set the limits for the bmu

 ******************************************************************************************************************/
void bmuShield::set_limits(float limits[14])
{
  	//?? note put in limit checks for saftey
	// set BMU FLAG LIMITS
    myPresRateHigh = limits[0];    //High pressure rate limit
    myPresHighLimit = limits[1];    //High pressure limit
    myPresLowLimit = limits[2];     //Low Pressure limit
    myInCurLimit = limits[3];      //current in limit during ON mode
    myHighChargeCur = limits[4];     //high current in limit in CHARGE mode
    myLowChargeCur = limits[5];      //current out limit in CHARGE mode
    myInOutCurLimit = limits[6];     // current in or out limit during OFF or BALANCE mode
    myVolBmuMismatch = limits[7];       //voltage mismatch limit between calculated and measured total voltage by BMU
    myTimeOutLimit = limits[8];	// the hours in charge or balance before timing out
    myVolLowBalAlarm  = limits[9];   // the voltage at which the system will not go in to balancing mode
    myBalRecLimit = limits[10];      // minimum voltage limit for recommending balancing
    myBalRecVol = limits[11];       // voltage difference at which balancing will be recommended
    myVolTolerance = limits[12];   // the max voltage difference that the virtual cells will have at the end of balancing
    myDoneCur = limits[13];         //the current at which the charging is called done
}

/*!******************************************************************************************************************
 \brief Measures and calculates data on the BMU shield
 ******************************************************************************************************************/
 void bmuShield::meas_bmuShield(void){
  // measure
  // myVoltage=avgADC(tVolInPin,3)*STRING_VOL_CONST;          // read voltage value
  myFwLeak=!digitalRead(frontWPin);             // read front leak sensor
  myBwLeak=!digitalRead(backWPin);              //read back leak sensor
  myPressCount = (myPressCount+1)%5;
  myPressure[myPressCount]=avgADC(presInPin,3)*PRESSURE_CONST-PRESSURE_OFFSET;  // read pressure from on board sensor
  int lastSecPres= (myPressCount+1)%5;
  myPresRate=(myPressure[myPressCount]-myPressure[lastSecPres]);          //calculate pressure rate
  if(abs(myPresRate)>abs(myMaxPressRate)) myMaxPressRate = myPresRate;
  // myPresRate= biquad_filter(biPresrate,myPresRate);      // get filtered pressure rate
  myPressureExt[myPressCount]=avgADC(presInExtPin,3)*EXT_PRESSURE_CONST-EXT_PRESSURE_OFFSET;   //get external pressure
  myPresExtRate=(myPressureExt[myPressCount]-myPressureExt[lastSecPres]);       //calculate externalpressure rate
  if(abs(myPresExtRate)>abs(myMaxPressExtRate)) myMaxPressExtRate = myPresExtRate;
  // myPresExtRate= biquad_filter(biPresrateExt, myPresExtRate); // get filtered external pressure rate
  float curOffset= avgADC(cur0InPin,3);             //read current offset from LEM sensor
  myRelay1fb=!digitalRead(relay1fbPin);     // read feedback from relay 1
  myRelay2fb=!digitalRead(relay2fbPin);    // read feedback from relay 2
  myCurrent=-(avgADC(curInPin,3)-curOffset)*CUR_CONST;     //read current sensor
  if(!myRelay1fb || !myRelay2fb) myCur0 = (1.0-ALPHA_CUR)*myCur0 + ALPHA_CUR *myCurrent;
  myCurrent = myCurrent - myCur0;

  batStateCal();

 }

/*!******************************************************************************************************************
 \brief actuate relays on the BMU shield
 ******************************************************************************************************************/
 void bmuShield::act_bmuShield(void){
 
  // actuate
  if(myRelayOn) set_relay_on(); 
  else set_relay_off();

  if(myExtDO1) digitalWrite(extDO1Pin, HIGH);
  else digitalWrite(extDO1Pin, LOW);
  if(myExtDO2) digitalWrite(extDO2Pin, HIGH);
  else digitalWrite(extDO2Pin, LOW);
 }


/*!******************************************************************************************************************
 \brief gets the status of the contactor number 2

 @return bool status, the contactor on the bmu
 ******************************************************************************************************************/
void bmuShield::set_flags(){ 

	// update mode time
  time_update();

	if(myFwLeak) myFlag |= 1;  // water leak in the front of the string
	if(myBwLeak) myFlag |= (1<<1);  // water leak in the back of the string
	// if delayed on time is off AND relays are suppose to be on AND either relay is off THEN contactor stuck open
	if(!myRelayDelay && myRelayOn && (!myRelay1fb || !myRelay2fb)) myFlag |= (1<<2); 

	// if the relays are suppose to be off AND either relay is on THEN contactors stuck closed
	if(!myRelayOn && (myRelay1fb || myRelay2fb)) myFlag |= (1<<3); 
	// if(max(abs(myPresRate),abs(myPresExtRate)) > myPresRateHigh) myFlag |= (1<<4); // set pressure rate flag
	// if(max(myPressure[myPressCount],myPressureExt[myPressCount]) > myPresHighLimit || min(myPressure[myPressCount],myPressureExt[myPressCount]) < myPresLowLimit) myFlag |= (1<<5); // set pressure out of bound flag
	if(abs(myPresRate) > myPresRateHigh) myFlag |= (1<<4); // set pressure rate flag
  if(myPressure[myPressCount] > myPresHighLimit || myPressure[myPressCount] < myPresLowLimit) myFlag |= (1<<5); // set pressure out of bound flag
  
  // set overall voltage mismatch
  // if(abs(myBmeSum - myVoltage) > myVolBmuMismatch){
  //   if(myBmuMismatchTimer.check()) myFlag |= (1<<6); 
  // }
  else myBmuMismatchTimer.reset();
  
	if(myMode == SYS_ON && myCurrent > myInCurLimit) myFlag |= (1<<7); // set On current out of bound
	if(myMode == CHARGE && (myCurrent > myHighChargeCur || myCurrent < myLowChargeCur)) myFlag |= (1<<8);  // set Charge current out of bound
 	if((!myRelay1fb || !myRelay2fb) && abs(myCurrent) > myInOutCurLimit) myFlag |= (1<<9);  // set off or balance current out of limit 
 	if((myMode==CHARGE || myMode==BALANCE) && (myModeTime.hours + myModeTime.minutes/60.0) >= myTimeOutLimit) myFlag |= (1<<10); // set time out flag
 	
  if((myMode == SYS_OFF || myMode == BALANCE) && myBmeMin < myVolLowBalAlarm) myFlag |= (1<<11); // set low balance voltage alarm //	
  if(myMode!=BALANCE && (myBmeMax-myBmeMin) > myBalRecVol && myBmeMin > myBalRecLimit) myFlag |= (1<<12); // set balance recommended flag
 	if(myMode==BALANCE && (myBmeMax-myBmeMin) < myVolTolerance){
    if(myBalDoneTimer.check()) myFlag |= (1<<13); // set balancing Done flag
  }
  else myBalDoneTimer.reset();
 	
 	// set charging Done flag if myCurrent is lower than myDoneCur for two minutes
 	if(myMode==CHARGE && myCurrent < myDoneCur && myBmeMax >= myChg2Vol-0.002){
    if(myChargeDoneTimer.check()) myFlag |= (1<<14); 
  }
 	else myChargeDoneTimer.reset(); // reset charge time
}

/*!******************************************************************************************************************
 \brief gets the status of the contactor number 2

 @return bool status, the contactor on the bmu
 ******************************************************************************************************************/
void bmuShield::reset_flags(){ 
	myFlag = 0;
	myFlagOverride = 0;
}



/*------------------------------------------------------------------------------
 * void set_mode(void))
 * sets the contactors on or off
 *----------------------------------------------------------------------------*/
void bmuShield::set_mode(Mode bmuMode){
  if(myMode != (bmuMode & 0x03)){
  	time_reset();
  	myMode = bmuMode;
  } 
  if(myMode == SYS_ON || myMode == CHARGE) myRelayOn= true;
  else{
  	myRelayOn = false;
  	if(myRelay1fb | myRelay2fb) set_relay_off(); //to turn off the bmu ASAP if contactors are on
  }
}
/***********************************************//**
 \brief set bmu charging voltage
*************************************************/
void bmuShield::set_chg2vol(float chg2vol){
	if(chg2vol>=0 && chg2vol<4.2){
		myChg2Vol= chg2vol;
	}
}



/***********************************************//**
 \brief set bmu override flags 
*************************************************/
void bmuShield::set_flag_over(uint16_t priority2Flag){
	
	myFlagOverride |= (myFlag & priority2Flag);
	
}

/***********************************************//**
 \brief set sum voltage of bme voltages
*************************************************/
void bmuShield::set_bme_sum(float bmeSum){
  myBmeSum = bmeSum;
}

/***********************************************//**
 \brief set min voltage of bme voltages
*************************************************/
void bmuShield::set_bme_min(float minVol){
  myBmeMin = minVol;
}

/***********************************************//**
 \brief set max voltage of bme voltages
*************************************************/
void bmuShield::set_bme_max(float maxVol){
  myBmeMax = maxVol;
}

/*------------------------------------------------------------------------------
 * void setRelays(void))
 * sets external digital outputs on or off
 *----------------------------------------------------------------------------*/
uint8_t bmuShield::set_extDO(uint8_t do_num, bool ext_do_on){
  if(do_num==1){
  	myExtDO1= ext_do_on;
  	return 0;
  }
  else if(do_num==2){
  	myExtDO2= ext_do_on;
  	return 0;
  }
  else return -1;
}

/*!******************************************************************************************************************
 \brief for sending bmu data in byte form

@param[out] uint8_t data_out[],  the bmu flag in byte form
 ******************************************************************************************************************/
void bmuShield::data_bmu(uint8_t data_out[22]){ 
	int idx=0;

	int2byte.asInt=float2int(mySoc);
	for(int k=0;k<2;k++) data_out[k+idx]=int2byte.asBytes[k];
	idx += 2;
	int2byte.asInt=myCycleCount;
	for(int k=0;k<2;k++) data_out[k+idx]=int2byte.asBytes[k];
	idx += 2;
	int2byte.asInt=float2int(myAvgDod);
	for(int k=0;k<2;k++) data_out[k+idx]=int2byte.asBytes[k];
	idx += 2;
	int2byte.asInt=myFlag;
	for(int k=0;k<2;k++) data_out[k+idx]=int2byte.asBytes[k];
	idx += 2;
	int2byte.asInt=myFlagOverride;
	for(int k=0;k<2;k++) data_out[k+idx]=int2byte.asBytes[k];
	idx += 2;	
	int2byte.asInt=float2int(myBmeSum);//myVoltage
	for(int k=0;k<2;k++) data_out[k+idx]=int2byte.asBytes[k];
	idx += 2;
  // float curTempo=myCurrent;
  // if(abs(curTempo)<.15) curTempo=0;
	int2byte.asInt=float2int(myCurrent+150);
	for(int k=0;k<2;k++) data_out[k+idx]=int2byte.asBytes[k];
	idx += 2;
	int2byte.asInt=float2int(myPressure[myPressCount]);
	for(int k=0;k<2;k++) data_out[k+idx]=int2byte.asBytes[k];
	idx += 2;
	int2byte.asInt=float2int(myMaxPressRate);
	for(int k=0;k<2;k++) data_out[k+idx]=int2byte.asBytes[k];
  myMaxPressRate=0;
	idx += 2;
	int2byte.asInt=float2int(myPressureExt[myPressCount]);
	for(int k=0;k<2;k++) data_out[k+idx]=int2byte.asBytes[k];
	idx += 2;
	int2byte.asInt=float2int(myMaxPressExtRate);
	for(int k=0;k<2;k++) data_out[k+idx]=int2byte.asBytes[k];
  myMaxPressExtRate=0;

}

/*!******************************************************************************************************************
 \brief for sending relay data in byte form

@return uint8_t relay states,  the bmu relay states in byte form
 ******************************************************************************************************************/
uint8_t bmuShield::data_relay(){ 
	return ((myExtDO2<<3)+(myExtDO1<<2)+(myRelay2fb<<1)+myRelay1fb) & 0x0F;
}

/*!******************************************************************************************************************
 \brief turns a float to an uint16_t and keeps two decimal places
 @param[in] float val,  the value in float form
 @return uint16_t value,  the float value *100 in uint16_t form
 ******************************************************************************************************************/

uint16_t bmuShield::float2int(float val){
  uint16_t int_val=0;
  if(val>655) int_val=655;
  else if(val<0) int_val=0;
  else int_val=(uint16_t)(val*100); 
  return int_val;
}

/*!******************************************************************************************************************
 \brief gets the bmu flag that are active and have not been overridden

 @return uint16_t voltage,  the string voltage measurement 
 ******************************************************************************************************************/
uint16_t bmuShield::get_flag(){ 
  return (myFlag & ~myFlagOverride);
}

/*!******************************************************************************************************************
 \brief gets the bmu mode

 @return mode myMode,  the string voltage measurement 
 ******************************************************************************************************************/
Mode bmuShield::get_mode(){ 
  return myMode;
}

/*!******************************************************************************************************************
 \brief gets the bmu state conditions

 @out float battery states,  the string voltage measurement 
 ******************************************************************************************************************/
void bmuShield::get_icInfo(uint16_t countInfo[2], float ampHinfo[4]){ 
  countInfo[0] = myCycleCount;
  countInfo[1] = myChargeCounter;
  ampHinfo[0] = myCap;
  ampHinfo[1] = myTotalDiscahrge;
  ampHinfo[2] = myAvgDod;
  ampHinfo[3] = myDod;
}

/*!******************************************************************************************************************
 \brief set the bmu state conditions

 @out float battery states,  the string voltage measurement 
 ******************************************************************************************************************/
void bmuShield::set_icInfo(uint16_t countInfo[2], float ampHinfo[4]){ 
  myCycleCount = countInfo[0] ;
  myChargeCounter = countInfo[1];
  myCap = ampHinfo[0];
  myTotalDiscahrge = ampHinfo[1];
  myAvgDod = ampHinfo[2];
  myDod = ampHinfo[3];
}


/*!******************************************************************************************************************
 \brief gets the string voltage on bmu

 @return float voltage,  the string voltage measurement 
 ******************************************************************************************************************/
float bmuShield::get_voltage(){ 
  return myVoltage;
}

/*!******************************************************************************************************************
 \brief gets the on board pressure

 @return float pressure,  the pressure on the bmu
 ******************************************************************************************************************/
float bmuShield::get_pressure(){ 
  return myPressure[myPressCount];
}

/*!******************************************************************************************************************
 \brief gets the on board pressure rate

 @return float presRate,  the pressure rate on the bmu
 ******************************************************************************************************************/
float bmuShield::get_presRate(){ 
  return myPresRate;
}

/*!******************************************************************************************************************
 \brief gets the external pressure 

 @return float pressure,  the external pressure attached to the bmu
 ******************************************************************************************************************/
float bmuShield::get_ext_pressure(){ 
  return myPressureExt[myPressCount];
}

/*!******************************************************************************************************************
 \brief gets the external pressure rate

 @return float presRate,  the external pressure rate attached to the bmu
 ******************************************************************************************************************/
float bmuShield::get_ext_presRate(){ 
  return myPresExtRate;
}

/*!******************************************************************************************************************
 \brief gets the current reading

 @return float current,  the current on the bmu
 ******************************************************************************************************************/
float bmuShield::get_current(){ 
  return myCurrent;
}

/*!******************************************************************************************************************
 \brief gets the status of the front leak sensor

 @return bool status, the front leak sensor on the bmu
 ******************************************************************************************************************/
bool bmuShield::get_front_leak(){ 
  return myFwLeak;
}

/*!******************************************************************************************************************
 \brief gets the status of the back leak sensor

 @return bool status, the back leak sensor on the bmu
 ******************************************************************************************************************/
bool bmuShield::get_back_leak(){ 
  return myBwLeak;
}

/*!******************************************************************************************************************
 \brief gets the status of the contactor number 1

 @return bool status, the contactor on the bmu
 ******************************************************************************************************************/
bool bmuShield::get_relay1_state(){ 
  return myRelay1fb;
}

/*!******************************************************************************************************************
 \brief gets the status of the contactor number 2

 @return bool status, the contactor on the bmu
 ******************************************************************************************************************/
bool bmuShield::get_relay2_state(){ 
  return myRelay2fb;
}

/*------------------------------------------------------------------------------
  \brief initialize digital inputs, digital output and analog inputs on the BMU shield
 *----------------------------------------------------------------------------*/
void bmuShield::bmuShield_initialize(){

	//************************ BMU setup *************************// 

	//initializes the digitalpins as inputs and outputs
    pinMode(relay1Pin, OUTPUT); //pin selected to control
    digitalWrite(relay1Pin, LOW);
    pinMode(relay2Pin, OUTPUT); //pin selected to control
    digitalWrite(relay2Pin, LOW); 
    pinMode(extDO1Pin, OUTPUT); //pin selected to control
    digitalWrite(extDO1Pin, LOW);
    pinMode(extDO2Pin, OUTPUT); //pin selected to control
    digitalWrite(extDO2Pin, LOW);    
    
    pinMode(frontWPin, INPUT);      // set pin to input
    digitalWrite(frontWPin, HIGH);  // turn on pullup resistors
    pinMode(backWPin, INPUT);       // set pin to input
    digitalWrite(backWPin, HIGH);  // turn on pullup resistors
    pinMode(relay1fbPin, INPUT);      // set pin to input
    digitalWrite(relay1fbPin, HIGH);  // turn on pullup resistors
    pinMode(relay2fbPin, INPUT);       // set pin to input
    digitalWrite(relay2fbPin, HIGH);  // turn on pullup resistors

    //initializes the analog sensors
    analogReadResolution(12); // set ADC to 12 bit resalution
   
    // set the old pressure values for the rate calculation
    myPressure[0]=avgADC(presInPin,3)*PRESSURE_CONST-PRESSURE_OFFSET;  // read pressure from on board sensor
    myPressureExt[0]=avgADC(presInExtPin,3)*EXT_PRESSURE_CONST-EXT_PRESSURE_OFFSET;   //get external pressure
    for(int i=1;i<5;i++){
      myPressure[i]=myPressure[0];
      myPressureExt[i]=myPressureExt[0];
    }
  	myDT=0.2;
    myDtInv = 1.0/myDT;
    myDtHour = myDT/3600;
  	
    // [b, a] = butter(2,.16*2/5) in Matlab
	// second order butterworthcutoff freq at 0.16 normalized nyquist freq, sampling 5hz
	biPresrate.gain=1;//filter gain
	biPresrate.b0 = 0.0088;         //input k coefficient
	biPresrate.b1 = 0.0177;        //input k-1 coefficient
	biPresrate.b2 = 0.0088;         //input k-2 coefficient
	biPresrate.a1 = -1.7172;        //output k-1 coefficient
	biPresrate.a2 = 0.7525;         //output k-2 coefficient
	biPresrate.x1 = 0;              //filter state
	biPresrate.x2 = biPresrate.x1;    //filter state
	   
	biPresrateExt.gain=1;//filter gain
	biPresrateExt.b0 = 0.0088;              //input k coefficient
	biPresrateExt.b1 = 0.0177;              //input k-1 coefficient
	biPresrateExt.b2 = 0.0088;                //input k-2 coefficient
	biPresrateExt.a1 = -1.7172;                //output k-1 coefficient
	biPresrateExt.a2 = 0.7525;                //output k-2 coefficient
	biPresrateExt.x1 = 0;            //filter state
	biPresrateExt.x2 = biPresrateExt.x1;    //filter state
}

/*------------------------------------------------------------------------------
  \brief initialize digital inputs, digital output and analog inputs on the BMU shield
 *----------------------------------------------------------------------------*/
uint8_t bmuShield::set_dt(float dt){
	if (dt>0){
		myDT=dt;
    myDtInv = 1.0/dt;
    myDtHour = dt/3600;
		return 0;
	}
	else return -1;
	
}


/*!******************************************************************************************************************
 \brief reset time spent in a mode

 ******************************************************************************************************************/
 void bmuShield::time_reset(){ 
   myModeTime.timeKeepingStamp=millis();
   myModeTime.milliseconds = 0;   
   myModeTime.minutes = 0;
   myModeTime.hours = 0;
}

/*!******************************************************************************************************************
 \brief update time spent in a mode

 ******************************************************************************************************************/
 void bmuShield::time_update(){
   myModeTime.milliseconds+= myModeTimer.elapsed(); 
   myModeTimer.reset();   
   if (myModeTime.milliseconds>60000){
       myModeTime.milliseconds-=60000;
       myModeTime.minutes++;
   }
   if (myModeTime.minutes>60){
       myModeTime.minutes-=60;
       myModeTime.hours++;
   }  
}

/*------------------------------------------------------------------------------
 * void relayOn(void))
 * turns both relays on with 1 sec delay between
 *----------------------------------------------------------------------------*/
void bmuShield::set_relay_on(void){

	if(myRelayDelay){
		if(myRrelayTimer.check()) myRelayDelay = false;
	}
	digitalWrite(relay1Pin, HIGH);
	if(!myRelayDelay) digitalWrite(relay2Pin, HIGH);
}

/*------------------------------------------------------------------------------
 * void relayOff(void))
 * turns both relays off and sets currentOffset
 *----------------------------------------------------------------------------*/
void bmuShield::set_relay_off(void){
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, LOW);
  myRrelayTimer.reset();
  myRelayDelay = true;
}

 /*------------------------------------------------------------------------------
 * float biquadFilter(BiquadType biPram,float input)
 * applies Biquad filter to the input
 * returns the filtered value
 *-----------------------------------------------------------------------------*/
 float bmuShield::biquad_filter(BiquadType biPram, float input){
    // Apply Filter
    float x = biPram.gain*input - biPram.a1*biPram.x1 - biPram.a2*biPram.x2;
    float out = biPram.b0*x + biPram.b1*biPram.x1 + biPram.b2*biPram.x2;
    // Update states
    biPram.x2 = biPram.x1;
    biPram.x1 = x;
    return out;
 }
 
  /*------------------------------------------------------------------------------
 * uint16_t avgADC(uint16_t adcPin,uint8_t n)
 * average the ADC measurement 2^n times
 * returns the average
 *-----------------------------------------------------------------------------*/
 uint16_t bmuShield::avgADC(uint16_t adcPin, uint8_t n){
  long tempo=0;
  for(int i=0;i<(1<<n);i++){
    tempo+=analogRead(adcPin);
    delay(1);
  }
  tempo=tempo>>n;
  return (int)tempo;
 }

/*------------------------------------------------------------------------------
 * void socCal(void)
 * calculates state of charge of the battery
 *----------------------------------------------------------------------------*/
void bmuShield::batStateCal(){
  

  float tempoCap = 0;
  if(abs(myCurrent)>.3) tempoCap = myCurrent * myDtHour;

  //SOC calculations
  if(myBmeMax>=4.2 && myCurrent<=4.5 && myMode==CHARGE) myCap=MAX_CAP;
  else myCap+=tempoCap;
  if(myCap>MAX_CAP) myCap=MAX_CAP;
  if(myCap<0) myCap=0; 
  mySoc=myCap/MAX_CAP*100;

  // cycle count calculation
  if(tempoCap < 0) myTotalDiscahrge -= tempoCap;
  if(myTotalDiscahrge > MAX_CAP){
  	myCycleCount++;
  	myTotalDiscahrge -= MAX_CAP;
  }

  // avg DOD calculation
  if(myMode == SYS_ON){
  	myDod -= tempoCap;
  }
  else if(myMode == CHARGE && myDod!=0){
  	myChargeCounter++;
    if(myChargeCounter!=0) myAvgDod = (myAvgDod*(myChargeCounter-1)/(float)myChargeCounter)+ myDod/(float)myChargeCounter;
    else myAvgDod = 0;
  	myDod=0;
  }
  
}

/*------------------------------------------------------------------------------
 * void initalizeSoc(void)
 * calculates state of charge of the battery
 *----------------------------------------------------------------------------*/
void bmuShield::initalizeSoc(){

  mySoc=findIntSoc((uint16_t)(myBmeMin*1000))/10.0;
  if(mySoc>100) mySoc=100;
  if(mySoc<0) mySoc=0; 
  myCap=mySoc*MAX_CAP*0.01;

}
/*------------------------------------------------------------------------------
 * int findIntSoc(int lVol)
 * Finds the SOC given inital voltage
 *----------------------------------------------------------------------------*/
uint16_t bmuShield::findIntSoc(uint16_t lVol){
 uint16_t lo=0,hi=1046, mid=0; 
 while(hi-lo>1){
   mid=(lo+hi)/2;
   if(lookUpVoltage[mid] >= lVol) lo=mid;
   else hi=mid;
 }
 if(lookUpVoltage[lo] == lVol) hi=lo;
 return lookUpSOC[hi];
}
