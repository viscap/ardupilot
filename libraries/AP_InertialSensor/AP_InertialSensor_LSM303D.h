/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_LSM303D_H__
#define __AP_INERTIAL_SENSOR_LSM303D_H__

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

// enable debug to see a register dump on startup
#define LSM303D_DEBUG 0

class AP_InertialSensor_LSM303D: public AP_InertialSensor_Backend
{
public:

    AP_InertialSensor_LSM303D(AP_InertialSensor &imu);

    /* Concrete implementation of AP_InertialSensor functions: */
    bool                update();    
    bool accel_sample_available(void) { return _have_sample_available; }
    bool gyro_sample_available(void) { return true; }

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:    
    bool                 _init_sensor(void);
    void                 _read_data_transaction();
    bool                 _data_ready();
    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    void                 _register_write_check(uint8_t reg, uint8_t val);    
    void                 _register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits);    
    bool                 _hardware_init(void);
    bool                 _sample_available();
    void                 disable_i2c(void);
    uint8_t              accel_set_range(uint8_t max_g);
    uint8_t              accel_set_samplerate(uint16_t frequency);
    uint8_t              accel_set_onchip_lowpass_filter_bandwidth(uint8_t bandwidth);
    uint8_t              mag_set_range(uint8_t max_ga);
    uint8_t              mag_set_samplerate(uint16_t frequency);

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    // support for updating filter at runtime
    int16_t _last_accel_filter_hz;

    // change the filter frequency
    void _set_accel_filter(uint8_t filter_hz);

    // This structure is used to pass data from the timer which reads
    // the sensor to the main thread. The _shared_data_idx is used to
    // prevent race conditions by ensuring the data is fully updated
    // before being used by the consumer
    struct {
        Vector3f _accel_filtered;
        Vector3f _gyro_filtered;
    } _shared_data[2];
    volatile uint8_t _shared_data_idx;

    // Low Pass filters for accel 
    LowPassFilter2pVector3f _accel_filter;

    // do we currently have a sample pending?
    bool _have_sample_available;

    // accel instances
    uint8_t _accel_instance;

    AP_HAL::DigitalSource *_drdy_pin_x;
    AP_HAL::DigitalSource *_drdy_pin_m;
    uint8_t         _accel_range_m_s2;
    float           _accel_range_scale;
    void            _read_data_transaction_accel();
    //void                 _read_data_transaction_mag();         
    uint8_t         _accel_samplerate;    
    uint8_t         _accel_onchip_filter_bandwith;    
    uint8_t         _mag_range_ga;
    float           _mag_range_scale;
    uint8_t         _mag_samplerate;    

    // expceted values of reg1 and reg7 to catch in-flight
    // brownouts of the sensor
    uint8_t         _reg1_expected;
    uint8_t         _reg7_expected;
    // ensure we can't initialise twice
    bool                        _initialised;



#if LSM303D_DEBUG
    void                        _dump_registers(void);
#endif
};


/*


    bool                 _sample_available();

    void                 _poll_data(void);

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    uint16_t                    _num_samples;
    float                       _accel_scale;
    float                       _mag_scale;

    uint32_t _last_sample_time_micros;

    int16_t              _LSM303D_product_id;

    // how many hardware samples before we report a sample to the caller
    uint8_t _sample_shift;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;

    void _set_filter_register(uint8_t filter_hz, uint8_t default_filter);

    uint16_t _error_count;

    // accumulation in timer - must be read with timer disabled
    // the sum of the values since last read
    Vector3l _accel_sum;
    Vector3l _mag_sum;
    volatile int16_t _sum_count;

public:

#if LSM303D_DEBUG
    void                        _dump_registers(void);
#endif
};
*/

#endif // __AP_INERTIAL_SENSOR_LSM303D_H__
