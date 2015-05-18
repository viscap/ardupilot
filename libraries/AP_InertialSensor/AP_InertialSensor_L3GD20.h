/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_L3GD20_H__
#define __AP_INERTIAL_SENSOR_L3GD20_H__

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

// enable debug to see a register dump on startup
#define L3GD20_DEBUG 0

class AP_InertialSensor_L3GD20 : public AP_InertialSensor_Backend
{
public:

    AP_InertialSensor_L3GD20(AP_InertialSensor &imu);

    /* Concrete implementation of AP_InertialSensor functions: */
    bool update();    
    bool accel_sample_available(void) { return true; }
    bool gyro_sample_available(void) { return _have_sample_available; }

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool                 _init_sensor();
    void                 _read_data_transaction();
    bool                 _data_ready();
    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    void                 _register_write_check(uint8_t reg, uint8_t val);
    bool                 _hardware_init(void);
    void                 disable_i2c(void);

    bool                 _sample_available();
    uint8_t              set_samplerate(uint16_t frequency);
    uint8_t              set_range(uint8_t max_dps);

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    // support for updating filter at runtime
    int16_t _last_gyro_filter_hz;

    // change the filter frequency
    void _set_gyro_filter(uint8_t filter_hz);

    // This structure is used to pass data from the timer which reads
    // the sensor to the main thread. The _shared_data_idx is used to
    // prevent race conditions by ensuring the data is fully updated
    // before being used by the consumer
    struct {
        Vector3f _gyro_filtered;        
    } _shared_data[1];
    volatile uint8_t _shared_data_idx;

    // Low Pass filters for gyro 
    LowPassFilter2pVector3f _gyro_filter;

    // do we currently have a sample pending?
    bool _have_sample_available;

    // gyro instances
    uint8_t _gyro_instance;    
    AP_HAL::DigitalSource *_drdy_pin_g;
    float           _gyro_range_scale;
    bool                        _initialised;
    float          _gyro_scale;


/*

    uint16_t       _num_samples;

    uint32_t _last_sample_time_micros;

    // ensure we can't initialise twice
    // how many hardware samples before we report a sample to the caller
    uint8_t _sample_shift;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;

    void _set_filter_register(uint8_t filter_hz, uint8_t default_filter);

    uint16_t _error_count;

    // accumulation in timer - must be read with timer disabled
    // the sum of the values since last read
    Vector3l _gyro_sum;
    volatile int16_t _sum_count;

*/

public:

#if L3GD20_DEBUG
    void                        _dump_registers(void);
#endif
};

#endif // __AP_INERTIAL_SENSOR_L3GD20_H__
