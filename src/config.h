#ifndef _BLUEJAY_ACTIVE_SAFETY_CONFIG
#define _BLUEJAY_ACTIVE_SAFETY_CONFIG

//min range before near space / active safety acts (zero to disable)
//FIXME: actually more a maximum range when started from zero...
//FIXME: NS_MIN_RANGE is a duplicate?
#define NS_MIN_RANGE 0
#define AS_MIN_RANGE 0

//default repulsion strength for objects
#define AS_REP_STRENGTH 1

//default attraction strength for targets
#define AS_ATTR_STRENGTH 1

//maximum velocity bound
#define AS_MAX_VELOCITY 10

//minimum velocity before cropping it to zero
#define AS_MIN_VELOCITY_XY 0
#define AS_MIN_VELOCITY_Z 0

//default control flags
#define AS_CTRL_FLAGS 15815 //SET_TARGET_VELOCITY (WARNING: update with bjos FlightController)!

//when quadratic lineair potential is used (default) the gap between 
#define AS_POT_TRANS_RANGE 3

#endif
