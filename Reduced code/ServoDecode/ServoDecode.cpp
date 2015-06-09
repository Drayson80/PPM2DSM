#include "ServoDecode.h"
#include <avr/interrupt.h>
#include <Arduino.h>

//#define ISR_' // uncomment this to send low level ISR debug to the serial port

#define PULSE_START_ON_RISING_EDGE  0
#define PULSE_START_ON_FALLING_EDGE (1<<ICES1)
#define ACQUISITION_COUNT  8  // must have this many consecutive valid frames to transition to the ready state.
volatile uint8_t pulseEnd = PULSE_START_ON_RISING_EDGE ; // default value
static volatile unsigned int Pulses[ MAX_CHANNELS + 1]; // array holding channel pulses width value in microseconds
static volatile unsigned int Failsafe[MAX_CHANNELS + 1]; // array holding channel fail safe values
static volatile uint8_t Channel;	// number of channels detected so far in the frame (first channel is 1)
static volatile uint8_t NbrChannels; // the total number of channels detected in a complete frame
static volatile decodeState_t State;	   // this will be one of the following states:
static volatile uint8_t stateCount;	   // counts the number of times this state has been repeated

static void processSync(){
// Sync was detected so reset the channel to 1 and update the system state
   Pulses[0] = ICR1 / TICKS_PER_uS;  // save the sync pulse duration for debugging
   if(State == READY_state) {
//	   DEBUG_OUT('R');
	   if( Channel != NbrChannels){  // if the number of channels is unstable, go into failsafe
		   State = FAILSAFE_state;
	   }
   }
   else{
     if(State == NOT_SYNCHED_state){
		 State = ACQUIRING_state;	  // this is the first sync pulse, we need one more to fill the channel data array
		 stateCount = 0;
	 }
	 else if( State == ACQUIRING_state)	{
	    if(++stateCount > ACQUISITION_COUNT) {
		State = READY_state;	     // this is the second sync and all channel data is ok so flag that channel data is valid
		    NbrChannels = Channel; // save the number of channels detected
		}
     }
	 else if( State == FAILSAFE_state)	{
		 if(Channel == NbrChannels){  // did we get good pulses on all channels
			State = READY_state;
		}
	 }
   }
   Channel = 0;	 // reset the channel counter
}

ISR(TIMER1_OVF_vect){
  if(State == READY_state){
    State = FAILSAFE_state;  // use fail safe values if signal lost
    Channel = 0; // reset the channel count
  }
}

ISR(TIMER1_CAPT_vect)
{
  // we want to measure the time to the end of the pulse
  if( (_SFR_BYTE(TCCR1B) & (1<<ICES1)) == pulseEnd ){
    TCNT1 = 0;	 // reset the counter
    if(ICR1 >= SYNC_GAP_LEN){   // is the space between pulses big enough to be the SYNC
	processSync();
    }
    else if(Channel < MAX_CHANNELS) {  // check if its a valid channel pulse and save it
	if( (ICR1 >= MIN_IN_PULSE_WIDTH)  && (ICR1 <= MAX_IN_PULSE_WIDTH) ){ // check for valid channel data
	  Pulses[++Channel] = ICR1 / TICKS_PER_uS;  // store pulse length as microsoeconds
	}
	else if(State == READY_state){
	  State = FAILSAFE_state;  // use fail safe values if input data invalid
	  Channel = 0; // reset the channel count
	}
    }
  }
}

ServoDecodeClass::ServoDecodeClass(){
}

void ServoDecodeClass::begin(){
  pinMode(icpPin, INPUT);
  digitalWrite(icpPin, HIGH);
  Channel = 0;
  State = NOT_SYNCHED_state;
    TCCR1A  = 0x00;                         // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1
    //  PWM11=0,  PWM10=0 => PWM Operation disabled
    //  TCCR1B = (1<<ICES1) | (1<<CS11);        // capture using rising edge,  prescaler = 8
    TCCR1B = (1<<CS11);                     // capture using falling edge, prescaler = 8
    // 8MHz clock with prescaler 8 means TCNT1 increments every 1 uS
    TIMSK1 = _BV(ICIE1)|_BV (TOIE1);        // enable input capture and overflow interrupts for timer 1

 // TCCR1A = 0x00;	   // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1 -- PWM11=0,PWM10=0 => PWM Operation disabled
  //TCCR1B = 0x02;	   // 16MHz clock with prescaler means TCNT1 increments every .5 uS (cs11 bit set
  //TIMSK1 = _BV(ICIE1)|_BV (TOIE1);   // enable input capture and overflow interrupts for timer 1
}

decodeState_t ServoDecodeClass::getState(){
  return State;
}

uint8_t ServoDecodeClass::getChanCount(){
  return NbrChannels;
}

void  ServoDecodeClass::setFailsafe(uint8_t chan, int value){
// pulse width to use if invalid data, value of 0 uses last valid data
  if( (chan > 0) && (chan <=  MAX_CHANNELS)  ) {
	 Failsafe[chan] = value;
  }
}
void  ServoDecodeClass::setFailsafe(){
// setFailsafe with no arguments sets failsafe for all channels to their current values
// usefull to capture current tx settings as failsafe values
  if(State == READY_state)
    for(uint8_t chan = 1; chan <=  MAX_CHANNELS; chan++) {
	  Failsafe[chan] = Pulses[chan];
    }
}

int ServoDecodeClass::GetChannelPulseWidth( uint8_t channel)
{
  // this is the access function for channel data
  int result = 0; // default value
  if( channel <=  MAX_CHANNELS)  {
     if( (State == FAILSAFE_state)&& (Failsafe[channel] > 0 ) )
	   result = Failsafe[channel]; // return the channels failsafe value if set and State is Failsafe
	 else if( (State == READY_state) || (State == FAILSAFE_state) ){
	   cli();	 //disable interrupts
	   result =  Pulses[channel] ;  // return the last valid pulse width for this channel
	   sei(); // enable interrupts
	 }
  }
  return result;
}
// make one instance for the user
ServoDecodeClass ServoDecode = ServoDecodeClass() ;