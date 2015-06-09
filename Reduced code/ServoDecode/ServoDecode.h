// ServoDecodeClass.h
//This library decodes typical RC composite servo inputs into individual channel pulse widths

#ifndef ServoDecode_H
#define ServoDecode_H

#include <inttypes.h>

#define icpPin		8	   // this interrupt handler must use pin 8
#define TICKS_PER_uS	1	    // number of timer ticks per microsecond


typedef enum {
    NULL_state=-1, NOT_SYNCHED_state, ACQUIRING_state, READY_state, FAILSAFE_state
 } decodeState_t;

#define MAX_CHANNELS    8	   // maximum number of channels we can store, don't increase this above 8
#define MIN_IN_PULSE_WIDTH (750 * TICKS_PER_uS) //a valid pulse must be at least 750us (note clock counts in 0.5 us ticks)
#define MAX_IN_PULSE_WIDTH (2250 * TICKS_PER_uS) //a valid pulse must be less than  2250us
#define SYNC_GAP_LEN	(3000 * TICKS_PER_uS) // we assume a space at least 3000us is sync (note clock counts in 0.5 us ticks)
#define FAILSAFE_PIN   13  // if defined, this will set the given pin high when invalid data is received


class ServoDecodeClass
{
  public:
	ServoDecodeClass(); //Constructor
	void begin();
	decodeState_t getState(); //State Function
	int GetChannelPulseWidth(uint8_t chan);  // this is the access function for channel data
	void setFailsafe(uint8_t chan, int value); // pulse width to use if invalid data, value of 0 uses last valid data
	void setFailsafe();// setFailsafe with no arguments sets failsafe for all channels to their current values
			     // useful to capture current tx settings as failsafe values
	uint8_t getChanCount();
 private:

};

extern ServoDecodeClass ServoDecode;  // make an instance for the user

#endif