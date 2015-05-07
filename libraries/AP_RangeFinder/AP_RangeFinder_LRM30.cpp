// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_RangeFinder_LRM30.cpp - 
 *       Code by Alejandro Hernández. Erlecobotics.com
 *
 *       datasheet: 
 *
 *       Sensor should be connected to the USB port
 */

#include "AP_RangeFinder_LRM30.h"
#include <AP_HAL.h>

#include <iostream>
#include <vector>

#include <serial/serial.h>
serialRS232::Serial* serial;

uint8_t CalcCrc8(uint8_t Data, unsigned char InitialValue);
uint8_t CalcCrc8FromArray(std::vector<uint8_t> pData, uint8_t InitialValue);


extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_LRM30::AP_RangeFinder_LRM30(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
}
#include <bitset>
int AP_RangeFinder_LRM30::laserOFF()
{
	std::vector<uint8_t> v;
	v.push_back(0xC0);
	v.push_back(0x42);
	v.push_back(0x0);
	v.push_back( CalcCrc8FromArray ( v, CRC8_INITIAL_VALUE ));

	size_t bytes_wrote = serial->write(v);

	std::vector<uint8_t> v_recv;
	int n  = serial->read(v_recv, 5);
	for(int i = 0; i < v_recv.size();i ++){
		printf(" - %.2X - ", v_recv[i]);
		std::bitset<8> x(v_recv[i]);
		std::cout << x ;
	}
	std::cout << std::endl;

#if DEBUG
		std::cout << "laserON Bytes written: " << bytes_wrote << " bytes recieved: " << n << std::endl;
#endif
}

int AP_RangeFinder_LRM30::laserON()
{
	std::vector<uint8_t> v;
	v.push_back(0xC0);
	v.push_back(0x41);
	v.push_back(0x0);
	v.push_back( CalcCrc8FromArray ( v, CRC8_INITIAL_VALUE ));

	size_t bytes_wrote = serial->write(v);
	std::vector<uint8_t> v_recv;

	int n  = serial->read(v_recv, 5);
	for(int i = 0; i < v_recv.size();i ++){
		printf(" - %.2X - ", v_recv[i]);
		std::bitset<8> x(v_recv[i]);
		std::cout << x ;
	}
	std::cout << std::endl;
//#if DEBUG
		std::cout << "laserON Bytes written: " << bytes_wrote << " bytes recieved: " << n << std::endl;
//#endif
}

/* 

*/
bool AP_RangeFinder_LRM30::detect(RangeFinder &_ranger, uint8_t instance)
{
	serial= new serialRS232::Serial("/dev/ttyACM0", 
									9600,
									serialRS232::Timeout::simpleTimeout(50),
									serialRS232::eightbits,
									serialRS232::parity_none,
									serialRS232::stopbits_one, 
									serialRS232::flowcontrol_none);
	std::cout << "Is the serial port open?";
	if(serial->isOpen())
		std::cout << " Yes." << std::endl;
	else
		std::cout << " No." << std::endl;

	std::cout << "AP_RangeFinder_LRM30::detect" << std::endl;
	std::vector<uint8_t> v;
	v.push_back(0xC0);
	v.push_back(0x41);
	v.push_back(0x0);
	v.push_back( CalcCrc8FromArray ( v, CRC8_INITIAL_VALUE ));

	size_t bytes_wrote = serial->write(v);
	std::vector<uint8_t> v_recv;

	int n  = serial->read(v_recv, 5);
	std::cout << "n::detect: "<<n << std::endl;

	for(int i = 0; i < v_recv.size();i ++){
		printf(" - %.2X - ", v_recv[i]);
		std::bitset<8> x(v_recv[i]);
		std::cout << x ;
	}
	std::cout << std::endl;
//#if DEBUG
		std::cout << "laserON Bytes written: " << bytes_wrote << " bytes recieved: " << n << std::endl;
//#endif

	return true;
	/*
    if (!start_reading()) {
        return false;
    }
    // give time for the sensor to process the request
    hal.scheduler->delay(50);
    uint16_t reading_cm;
    return get_reading(reading_cm);
    */
}

// start_reading() - ask sensor to make a range reading
bool AP_RangeFinder_LRM30::start_reading()
{
	std::cout << "AP_RangeFinder_LRM30::start_reading" << std::endl;
	/*
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        return false;
    }

    uint8_t tosend[1] = 
        { AP_RANGE_FINDER_MAXSONARI2CXL_COMMAND_TAKE_RANGE_READING };

    // send command to take reading
    if (hal.i2c->write(AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR,
                       1, tosend) != 0) {
        i2c_sem->give();
        return false;
    }

    // return semaphore
    i2c_sem->give();

    return true;
    */
}

// read - return last value measured by sensor
bool AP_RangeFinder_LRM30::get_reading(uint16_t &reading_cm)
{
	std::cout << "AP_RangeFinder_LRM30::get_reading" << std::endl;
	/*
    uint8_t buff[2];

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        return false;
    }

    // take range reading and read back results
    if (hal.i2c->read(AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR, 2, buff) != 0) {
        i2c_sem->give();
        return false;
    }
    i2c_sem->give();

    // combine results into distance
    reading_cm = ((uint16_t)buff[0]) << 8 | buff[1];

    // trigger a new reading
    start_reading();
*/
    return true;
    
}


/* 
   update the state of the sensor
*/
#include <stdio.h>
void AP_RangeFinder_LRM30::update(void)
{

	std::vector<uint8_t> v;
	v.push_back(0xC0);
	v.push_back(0x40);
	v.push_back(0x01);
	v.push_back(0x0D);
	v.push_back(CalcCrc8FromArray ( v, CRC8_INITIAL_VALUE ));

	std::vector<uint8_t> v_recv;
	size_t bytes_wrote = serial->write(v);
	int n  = serial->read(v_recv, 10);

	if(n==0) return ;
	if(v_recv[0]!=0) return;
	uint32_t _Recv2 = v_recv[5]<<24 | v_recv[4]<<16 | v_recv[3]<<8 | v_recv[2];

	//std::cout << "update value: " << _Recv2*50e-6 << std::endl;

	state.distance_cm = _Recv2*50e-4;
	update_status();

	//hal.console->printf_P(PSTR("%hhX\n", c));
	/*
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
    */
}

//***************************************************************************************************************************
// Calculate the CRC8 checksum for the given value. No reflection, inversion, reversion or final XOR used. Just plain CRC8.
//   input:
//     -- Data:         Data to calculate the CRC for.
//     -- InitialValue: The initial value for the CRC (e.g. previous calculated value).
//   output:
//      -- Calculated CRC8 checksum
//
//***************************************************************************************************************************
uint8_t CalcCrc8(uint8_t Data, unsigned char InitialValue)
{
  uint8_t i;
  
  for (i=0; i<8; i++){
    if (((InitialValue & 0x80) != 0) != ((Data >> (7-i)) & 1))
      InitialValue = (InitialValue << 1) ^ CRC8_POLYNOMIAL;
    else
      InitialValue <<= 1;
  }
  return InitialValue;
}

//***************************************************************************************************************************
// Calculate the CRC8 checksum for the given array. No reflection, inversion, reversion or final XOR used. Just plain CRC8.
//   input:
//     -- pData:        Pointer to the array to calculate the CRC for.
//     -- NumBytes:     Size of array in bytes.
//     -- InitialValue: The initial value for the CRC (e.g. previous calculated value).
//   output:
//      -- Calculated CRC8 checksum
//
//***************************************************************************************************************************
uint8_t CalcCrc8FromArray(std::vector<uint8_t> pData, uint8_t InitialValue)
{
  for(int i = 0; i < pData.size(); i++){
    InitialValue = CalcCrc8(pData[i], InitialValue);
  }
  return InitialValue;
}
