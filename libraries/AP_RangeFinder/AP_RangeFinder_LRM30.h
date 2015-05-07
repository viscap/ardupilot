// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_RANGEFINDER_LRM30_H__
#define __AP_RANGEFINDER_LRM30_H__

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define DEBUG 1


#define CRC8_INITIAL_VALUE  0xAA
#define CRC8_POLYNOMIAL     0xA6

class AP_RangeFinder_LRM30 : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_LRM30(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);

private:
    // start a reading
    static bool start_reading(void);
    static bool get_reading(uint16_t &reading_cm);
	
	int laserOFF();
	int laserON();

};
#endif  // __AP_RANGEFINDER_LRM30_H__
