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

//minimum altitude for safe in-air state, below this we should enforce a (goto) land (and above disallow takeoff)
#define AS_MIN_SAFE_ALT 0.2 

////default BJOS control flags 
#define AS_CTRL_FLAG_IGN_LAND (1 << 13)
#define AS_CTRL_FLAG_IGN_TAKEOFF (1 << 12)
#define AS_CTRL_LAND_FLAGS 11719//SET_TARGET_LAND     (WARNING: update with bjos FlightController)!
#define AS_CTRL_VEL_FLAGS 15815 //SET_TARGET_VELOCITY (WARNING: update with bjos FlightController)!
#define AS_CTRL_POS_FLAGS 15864 //SET_TARGET_POSITION (WARNING: update with bjos FlightController)!
#define AS_CTRL_FLAGS AS_CTRL_VEL_FLAGS 

//the gap between when quadratic lineair potential is used (default)
#define AS_POT_TRANS_RANGE 3

//determine when to use a absolute position setpoint instead of velocity
#define AS_POS_RADIUS 0

#endif
