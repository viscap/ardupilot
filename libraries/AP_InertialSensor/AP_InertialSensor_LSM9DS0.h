/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_LSM9DS0_H__
#define __AP_INERTIAL_SENSOR_LSM9DS0_H__

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

// enable debug to see a register dump on startup
#define LSM9DS0_DEBUG 1

// gyro_scale_lsm9ds0 defines the possible full-scale ranges of the gyroscope:
enum gyro_scale_lsm9ds0
{
    G_SCALE_245DPS,     // 00:  245 degrees per second
    G_SCALE_500DPS,     // 01:  500 dps
    G_SCALE_2000DPS,    // 10:  2000 dps
};
// accel_scale defines all possible FSR's of the accelerometer:
enum accel_scale
{
    A_SCALE_2G, // 000:  2g
    A_SCALE_4G, // 001:  4g
    A_SCALE_6G, // 010:  6g
    A_SCALE_8G, // 011:  8g
    A_SCALE_16G // 100:  16g
};
// mag_scale defines all possible FSR's of the magnetometer:
enum mag_scale
{
    M_SCALE_2GS,    // 00:  2Gs
    M_SCALE_4GS,    // 01:  4Gs
    M_SCALE_8GS,    // 10:  8Gs
    M_SCALE_12GS,   // 11:  12Gs
};
// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
enum gyro_odr
{                           // ODR (Hz) --- Cutoff
    G_ODR_95_BW_125  = 0x0, //   95         12.5
    G_ODR_95_BW_25   = 0x1, //   95          25
    // 0x2 and 0x3 define the same data rate and bandwidth
    G_ODR_190_BW_125 = 0x4, //   190        12.5
    G_ODR_190_BW_25  = 0x5, //   190         25
    G_ODR_190_BW_50  = 0x6, //   190         50
    G_ODR_190_BW_70  = 0x7, //   190         70
    G_ODR_380_BW_20  = 0x8, //   380         20
    G_ODR_380_BW_25  = 0x9, //   380         25
    G_ODR_380_BW_50  = 0xA, //   380         50
    G_ODR_380_BW_100 = 0xB, //   380         100
    G_ODR_760_BW_30  = 0xC, //   760         30
    G_ODR_760_BW_35  = 0xD, //   760         35
    G_ODR_760_BW_50  = 0xE, //   760         50
    G_ODR_760_BW_100 = 0xF, //   760         100
};
// accel_oder defines all possible output data rates of the accelerometer:
enum accel_odr
{
    A_POWER_DOWN,   // Power-down mode (0x0)
    A_ODR_3125,     // 3.125 Hz (0x1)
    A_ODR_625,      // 6.25 Hz (0x2)
    A_ODR_125,      // 12.5 Hz (0x3)
    A_ODR_25,       // 25 Hz (0x4)
    A_ODR_50,       // 50 Hz (0x5)
    A_ODR_100,      // 100 Hz (0x6)
    A_ODR_200,      // 200 Hz (0x7)
    A_ODR_400,      // 400 Hz (0x8)
    A_ODR_800,      // 800 Hz (9)
    A_ODR_1600      // 1600 Hz (0xA)
};

  // accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
enum accel_abw
{
    A_ABW_773,      // 773 Hz (0x0)
    A_ABW_194,      // 194 Hz (0x1)
    A_ABW_362,      // 362 Hz (0x2)
    A_ABW_50,       //  50 Hz (0x3)
};


// mag_oder defines all possible output data rates of the magnetometer:
enum mag_odr
{
    M_ODR_3125, // 3.125 Hz (0x00)
    M_ODR_625,  // 6.25 Hz (0x01)
    M_ODR_125,  // 12.5 Hz (0x02)
    M_ODR_25,   // 25 Hz (0x03)
    M_ODR_50,   // 50 (0x04)
    M_ODR_100,  // 100 Hz (0x05)
};

// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BITS_DLPF_CFG_256HZ_NOLPF2              0x00
#define BITS_DLPF_CFG_188HZ                             0x01
#define BITS_DLPF_CFG_98HZ                              0x02
#define BITS_DLPF_CFG_42HZ                              0x03
#define BITS_DLPF_CFG_20HZ                              0x04
#define BITS_DLPF_CFG_10HZ                              0x05
#define BITS_DLPF_CFG_5HZ                               0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF              0x07
#define BITS_DLPF_CFG_MASK                              0x07


class AP_InertialSensor_LSM9DS0: public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_LSM9DS0(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool                update();

    bool gyro_sample_available(void) { return _sum_count_g >= _sample_count_g; }
    bool accel_sample_available(void) { return _sum_count_xm >= _sample_count_xm; }

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:

    // instance numbers of accel and gyro data
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    AP_HAL::DigitalSource *_drdy_pin;

    bool                 _init_sensor(void);
    bool                 _sample_available();
    void                 _read_data_transaction_g();
    void                 _read_data_transaction_xm();
    bool                 _data_ready();
    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    bool                 _hardware_init(void);

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    static const float          _gyro_scale;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;

    void _set_filter_register(uint8_t filter_hz);

    // count of bus errors
    uint16_t _error_count;

    // how many hardware samples before we report a sample to the caller
    uint8_t _sample_count_g;
    uint8_t _sample_count_xm;

#if LSM9DS0_FAST_SAMPLING
    Vector3f _accel_filtered;
    Vector3f _gyro_filtered;

    // Low Pass filters for gyro and accel 
    LowPassFilter2p _accel_filter_x;
    LowPassFilter2p _accel_filter_y;
    LowPassFilter2p _accel_filter_z;
    LowPassFilter2p _gyro_filter_x;
    LowPassFilter2p _gyro_filter_y;
    LowPassFilter2p _gyro_filter_z;
#else
    // accumulation in timer - must be read with timer disabled
    // the sum of the values since last read
    Vector3l _accel_sum;
    Vector3l _gyro_sum;
    Vector3l _mag_sum;
#endif
    volatile uint16_t _sum_count_g;
    volatile uint16_t _sum_count_xm;

    AP_HAL::DigitalSource *_drdy_pin_a;
    AP_HAL::DigitalSource *_drdy_pin_m;
    AP_HAL::DigitalSource *_drdy_pin_g;
    float _gRes, _aRes, _mRes;

    uint32_t _last_sample_time_micros;

    // initGyro() -- Sets up the gyroscope to begin reading.
    // This function steps through all five gyroscope control registers.
    // Upon exit, the following parameters will be set:
    //  - CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled. 
    //      95 Hz ODR, 12.5 Hz cutoff frequency.
    //  - CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
    //      set to 7.2 Hz (depends on ODR).
    //  - CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
    //      active high). Data-ready output enabled on DRDY_G.
    //  - CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
    //      address. Scale set to 245 DPS. SPI mode set to 4-wire.
    //  - CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
    void _initGyro();

    // initAccel() -- Sets up the accelerometer to begin reading.
    // This function steps through all accelerometer related control registers.
    // Upon exit these registers will be set as:
    //  - CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
    //  - CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
    //      all axes enabled.
    //  - CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
    //  - CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
    void _initAccel();

    // initMag() -- Sets up the magnetometer to begin reading.
    // This function steps through all magnetometer-related control registers.
    // Upon exit these registers will be set as:
    //  - CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
    //  - CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
    //      requests don't latch. Temperature sensor disabled.
    //  - CTRL_REG6_XM = 0x00:  2 Gs scale.
    //  - CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
    //  - INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
    void _initMag();

    void _calcgRes(gyro_scale_lsm9ds0 gScl);
    void _calcaRes(accel_scale aScl);
    void _calcmRes(mag_scale mScl);

    uint8_t _sample_shift;

#if LSM9DS0_DEBUG
    void                        _dump_registers(void);
#endif    
};


#endif // __AP_INERTIAL_SENSOR_LSM9DS0_H__
