/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 
    -- Coded by Victor Mayoral Vilches -- 
            the code reuses partially the Sparkfun library 
            from https://github.com/sparkfun/LSM9DS0_Breakout/tree/master/Libraries/Arduino/SFE_LSM9DS0

*/

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AP_InertialSensor_LSM9DS0.h"
#include "../AP_HAL_Linux/GPIO.h"

extern const AP_HAL::HAL& hal;

////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G          0x0F
#define CTRL_REG1_G         0x20
#define CTRL_REG2_G         0x21
#define CTRL_REG3_G         0x22
#define CTRL_REG4_G         0x23
#define CTRL_REG5_G         0x24
#define REFERENCE_G         0x25
#define STATUS_REG_G        0x27
#define OUT_X_L_G           0x28
#define OUT_X_H_G           0x29
#define OUT_Y_L_G           0x2A
#define OUT_Y_H_G           0x2B
#define OUT_Z_L_G           0x2C
#define OUT_Z_H_G           0x2D
#define FIFO_CTRL_REG_G     0x2E
#define FIFO_SRC_REG_G      0x2F
#define INT1_CFG_G          0x30
#define INT1_SRC_G          0x31
#define INT1_THS_XH_G       0x32
#define INT1_THS_XL_G       0x33
#define INT1_THS_YH_G       0x34
#define INT1_THS_YL_G       0x35
#define INT1_THS_ZH_G       0x36
#define INT1_THS_ZL_G       0x37
#define INT1_DURATION_G     0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM       0x05
#define OUT_TEMP_H_XM       0x06
#define STATUS_REG_M        0x07
#define OUT_X_L_M           0x08
#define OUT_X_H_M           0x09
#define OUT_Y_L_M           0x0A
#define OUT_Y_H_M           0x0B
#define OUT_Z_L_M           0x0C
#define OUT_Z_H_M           0x0D
#define WHO_AM_I_XM         0x0F
#define INT_CTRL_REG_M      0x12
#define INT_SRC_REG_M       0x13
#define INT_THS_L_M         0x14
#define INT_THS_H_M         0x15
#define OFFSET_X_L_M        0x16
#define OFFSET_X_H_M        0x17
#define OFFSET_Y_L_M        0x18
#define OFFSET_Y_H_M        0x19
#define OFFSET_Z_L_M        0x1A
#define OFFSET_Z_H_M        0x1B
#define REFERENCE_X         0x1C
#define REFERENCE_Y         0x1D
#define REFERENCE_Z         0x1E
#define CTRL_REG0_XM        0x1F
#define CTRL_REG1_XM        0x20
#define CTRL_REG2_XM        0x21
#define CTRL_REG3_XM        0x22
#define CTRL_REG4_XM        0x23
#define CTRL_REG5_XM        0x24
#define CTRL_REG6_XM        0x25
#define CTRL_REG7_XM        0x26
#define STATUS_REG_A        0x27
#define OUT_X_L_A           0x28
#define OUT_X_H_A           0x29
#define OUT_Y_L_A           0x2A
#define OUT_Y_H_A           0x2B
#define OUT_Z_L_A           0x2C
#define OUT_Z_H_A           0x2D
#define FIFO_CTRL_REG       0x2E
#define FIFO_SRC_REG        0x2F
#define INT_GEN_1_REG       0x30
#define INT_GEN_1_SRC       0x31
#define INT_GEN_1_THS       0x32
#define INT_GEN_1_DURATION  0x33
#define INT_GEN_2_REG       0x34
#define INT_GEN_2_SRC       0x35
#define INT_GEN_2_THS       0x36
#define INT_GEN_2_DURATION  0x37
#define CLICK_CFG           0x38
#define CLICK_SRC           0x39
#define CLICK_THS           0x3A
#define TIME_LIMIT          0x3B
#define TIME_LATENCY        0x3C
#define TIME_WINDOW         0x3D
#define ACT_THS             0x3E
#define ACT_DUR             0x3F


AP_InertialSensor_LSM9DS0::AP_InertialSensor_LSM9DS0(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu),
    _drdy_pin(NULL),
    _spi(NULL),
    _spi_sem(NULL),
    _last_filter_hz(0),
    _error_count(0),
    _sample_count_g(0),
    _sample_count_xm(0),
    _accel_sum(),
    _gyro_sum(),
    _sum_count_g(0),
    _sum_count_xm(0)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_LSM9DS0::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_LSM9DS0 *sensor = new AP_InertialSensor_LSM9DS0(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }

    return sensor;
}

/*
  initialise the sensor
 */
bool AP_InertialSensor_LSM9DS0::_init_sensor(void)
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_LSM9DS0_G);
    _spi_sem = _spi->get_semaphore();

    _drdy_pin_a = hal.gpio->channel(BBB_P8_8);
    _drdy_pin_m = hal.gpio->channel(BBB_P8_10);
    _drdy_pin_g = hal.gpio->channel(BBB_P8_34);

    hal.scheduler->suspend_timer_procs();

    uint8_t whoami = _register_read(WHO_AM_I_XM);
    hal.console->printf("LSM9DS0: WHOAMI 0x%x\n", (unsigned)whoami);
//    if (whoami != 0xD4) {
    if (whoami != 0x49) {
        // TODO: we should probably accept multiple chip
        // revisions. This is the one on the PXF
        hal.console->printf("LSM9DS0: unexpected WHOAMI 0x%x\n", (unsigned)whoami);
        return false;
    }

    uint8_t tries = 0;
    do {
        bool success = _hardware_init();
        if (success) {
            hal.scheduler->delay(5+2);
            if (!_spi_sem->take(100)) {
                hal.scheduler->panic(PSTR("LSM9DS0: Unable to get semaphore"));
                return false;
            }
            if (_data_ready()) {
                _spi_sem->give();
                break;
            } else {
                hal.console->println_P(
                        PSTR("LSM9DS0 startup failed: no data ready"));
                return false;
            }
            _spi_sem->give();
        }
        if (tries++ > 5) {
            hal.console->print_P(PSTR("failed to boot LSM9DS0 5 times")); 
            return false;
        }
    } while (1);

    hal.scheduler->resume_timer_procs();

    _last_sample_time_micros = hal.scheduler->micros();

    // grab the used instances
    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();

    /* read the first lot of data.
     * _read_data_transaction requires the spi semaphore to be taken by
     * its caller. */
    hal.scheduler->delay(10);
    if (_spi_sem->take(100)) {
        _read_data_transaction_g();
        _read_data_transaction_xm();
        _spi_sem->give();
    }

#if LSM9DS0_DEBUG
    _dump_registers();
#endif

    // start the timer process to read samples
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_InertialSensor_LSM9DS0::_poll_data));

    _sample_shift = 1;
    return true;
}

// return true if a sample is available
bool AP_InertialSensor_LSM9DS0::_sample_available()
{
    _poll_data();
    return (_sum_count_g >> _sample_shift || _sum_count_xm >> _sample_shift) > 0;
}

/*
  process any 
 */
bool AP_InertialSensor_LSM9DS0::update( void )
{    
    //hal.console->printf("UPDATE\n");
   // wait for at least 1 sample
    uint32_t start = hal.scheduler->millis();
    if(!_sample_available()){
        while ((hal.scheduler->millis() - start) < 1000) {
            hal.scheduler->delay_microseconds(100);
            if (_sample_available()) {
                break;
            }
        }
    }

    // we have a full set of samples
    Vector3f accel, gyro;

    // disable timer procs for mininum time
    hal.scheduler->suspend_timer_procs();

    gyro  = Vector3f(_gyro_sum.x, _gyro_sum.y, _gyro_sum.z);
    accel = Vector3f(_accel_sum.x, _accel_sum.y, _accel_sum.z);

    float _num_samples_g, _num_samples_xm;

    _num_samples_g = _sum_count_g;
    _num_samples_xm = _sum_count_xm;

    _accel_sum.zero();
    _gyro_sum.zero();
    _mag_sum.zero();
    _sum_count_g = 0;
    _sum_count_xm = 0;

    hal.scheduler->resume_timer_procs();

    //hal.console->printf(("-> %d\t%d\n"), gyro, accel);
    //hal.console->printf(("----------------------\n"));
    //hal.console->printf(("%.8f\t%.8f\t%.8f\n"), accel.x, accel.y, accel.z);
    //hal.console->printf(("%.8f\t%.8f\t%.8f\n"), accel.x * _aRes / _num_samples_xm,
    //                                            accel.y * _aRes / _num_samples_xm,
    //                                            accel.z * _aRes / _num_samples_xm);
    //hal.console->printf(("%.8f\t%.8f\t%.8f\n"), gyro.x, gyro.y, gyro.z);

    gyro *= _gRes / _num_samples_g;
    accel *= _aRes / _num_samples_xm;

    //hal.console->printf(("-* %d\t%d\n"), gyro, accel);
    //hal.console->printf(("%.8f\t%.8f\n"), _gRes, _aRes);
    //hal.console->printf(("%d\t\n"), num_samples);

    gyro.rotate(ROTATION_PITCH_180_YAW_90);
    accel.rotate(ROTATION_PITCH_180_YAW_90);

    _rotate_and_offset_gyro(_gyro_instance, gyro);
    _rotate_and_offset_accel(_accel_instance, accel);

    //hal.console->printf(("%.8f\t%.8f\t%.8f\n"), gyro.x, gyro.y, gyro.z);
    //hal.console->printf(("%.8f\t%.8f\t%.8f\n"), accel.x, accel.y, accel.z);

    if (_last_filter_hz != _imu.get_filter()) {
       if (_spi_sem->take(10)) {
            _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
            _set_filter_register(_imu.get_filter());
            _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
            _spi_sem->give();
        }
    }
    return true;
}

/*================ HARDWARE FUNCTIONS ==================== */

/**
 * Return true if the LSM9DS0 has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
bool AP_InertialSensor_LSM9DS0::_data_ready()
{
    uint8_t rvalue = 0;
    if (_drdy_pin_g) {
        if (_drdy_pin_g->read() != 0){
            rvalue=1;
        }
        if (_drdy_pin_a) {
            if (_drdy_pin_a->read() != 0){
                rvalue = 3;
            }
        }
    } else if (_drdy_pin_a) {
        if (_drdy_pin_a->read() != 0){
            rvalue = 2;
        }
    }
    return rvalue;
}

/**
 * Timer process to poll for new data from the LSM9DS0.
 */
void AP_InertialSensor_LSM9DS0::_poll_data(void)
{

    if (hal.scheduler->in_timerprocess()) {
        if (!_spi_sem->take_nonblocking()) {
            //
            //  the semaphore being busy is an expected condition when the
            //  mainline code is calling wait_for_sample() which will
            //  grab the semaphore. We return now and rely on the mainline
            //  code grabbing the latest sample.
            //
            return;
        } 
        if (_data_ready() == 1) {
            _last_sample_time_micros = hal.scheduler->micros();
            _read_data_transaction_g(); 
            _read_data_transaction_xm();             
        } else if (_data_ready() == 2){
            _last_sample_time_micros = hal.scheduler->micros();
            _read_data_transaction_xm();             
        } else if (_data_ready() == 3){
            _last_sample_time_micros = hal.scheduler->micros();
            _read_data_transaction_g(); 
            _read_data_transaction_xm();   
        }
        _spi_sem->give();
    } else {
        // Synchronous read - take semaphore //
        if (_spi_sem->take(10)) {
            if (_data_ready() == 1) {
                _last_sample_time_micros = hal.scheduler->micros();
                _read_data_transaction_g(); 
            _read_data_transaction_xm();             
            } else if (_data_ready() == 2){
                _last_sample_time_micros = hal.scheduler->micros();
                _read_data_transaction_xm();             
            } else if (_data_ready() == 3){
                _last_sample_time_micros = hal.scheduler->micros();
                _read_data_transaction_g(); 
                _read_data_transaction_xm();                         
            }
            _spi_sem->give();
        } else {
            hal.scheduler->panic(
                PSTR("PANIC: AP_InertialSensor_LSM9DS0::_poll_data "
                     "failed to take SPI semaphore synchronously"));
        }
    }
}

void AP_InertialSensor_LSM9DS0::_read_data_transaction_g()
{

    // read gyro values
    uint8_t temp[6]; // We'll read six bytes from the gyro into temp           
    for (uint8_t i=0;i<6;i++){
        temp[i] = _register_read(OUT_X_L_G + i);
    }

    uint16_t gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
    uint16_t gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
    uint16_t gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz    

    _gyro_sum.x  += gx;
    _gyro_sum.y  += gy;
    _gyro_sum.z  -= gz;    
    _sum_count_g++;

    if (_sum_count_g == 0) {
        _gyro_sum.zero();
    }
}
void AP_InertialSensor_LSM9DS0::_read_data_transaction_xm()
{
    //xm
    uint8_t temp[6]; // We'll read six bytes from the gyro into temp           
    // read accel values
    for (uint8_t i=0;i<6;i++){
        temp[i] = _register_read(OUT_X_L_A + i);
    }    
    uint16_t ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
    uint16_t ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
    uint16_t az = (temp[5] << 8) | temp[4]; // Store z-axis values into az    

    _accel_sum.x  += ax;
    _accel_sum.y  += ay;
    _accel_sum.z  -= az;
    _sum_count_xm++;

    // read mag values
    for (uint8_t i=0;i<6;i++){
        temp[i] = _register_read(OUT_X_L_M + i);
    }    
    uint16_t mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
    uint16_t my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
    uint16_t mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz    

    _mag_sum.x  += mx;
    _mag_sum.y  += my;
    _mag_sum.z  -= mz;    

    if (_sum_count_xm == 0) {
        _gyro_sum.zero();
        _mag_sum.zero();
    }
}

uint8_t AP_InertialSensor_LSM9DS0::_register_read( uint8_t reg )
{
    uint8_t addr = reg | 0x80; // Set most significant bit

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _spi->transaction(tx, rx, 2);

    return rx[1];
}

void AP_InertialSensor_LSM9DS0::_register_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg;
    tx[1] = val;
    _spi->transaction(tx, rx, 2);
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_LSM9DS0::_set_filter_register(uint8_t filter_hz)
{
/*
     uint8_t filter = default_filter;
     // choose filtering frequency
     switch (filter_hz) {
     case 5:
         filter = BITS_DLPF_CFG_5HZ;
         break;
     case 10:
         filter = BITS_DLPF_CFG_10HZ;
         break;
     case 20:
         filter = BITS_DLPF_CFG_20HZ;
         break;
     case 42:
         filter = BITS_DLPF_CFG_42HZ;
         break;
     case 98:
         filter = BITS_DLPF_CFG_98HZ;
         break;
     }

     if (filter != 0) {
         _last_filter_hz = filter_hz;
         _register_write(MPUREG_CONFIG, filter);
     }
*/
}

void AP_InertialSensor_LSM9DS0::_calcgRes(gyro_scale_lsm9ds0 gScl)
{
    // Possible gyro scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
    // to calculate DPS/(ADC tick) based on that 2-bit value:
    switch (gScl)
    {
    case G_SCALE_245DPS:
        _gRes = 245.0 / 32768.0;
        break;
    case G_SCALE_500DPS:
        _gRes = 500.0 / 32768.0;
        break;
    case G_SCALE_2000DPS:
        _gRes = 2000.0 / 32768.0;
        break;
    }
}

void AP_InertialSensor_LSM9DS0::_calcaRes(accel_scale aScl)
{
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an 
    // algorithm to calculate g/(ADC tick) based on that 3-bit value:
    _aRes = aScl == A_SCALE_16G ? 16.0 / 32768.0 : 
           (((float) aScl + 1.0) * 2.0) / 32768.0;
}

void AP_InertialSensor_LSM9DS0::_calcmRes(mag_scale mScl)
{
    // Possible magnetometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
    // to calculate Gs/(ADC tick) based on that 2-bit value:
    _mRes = mScl == M_SCALE_2GS ? 2.0 / 32768.0 : 
           (float) (mScl << 2) / 32768.0;
}

void AP_InertialSensor_LSM9DS0::_initGyro()
{

    hal.console->printf(("initGyro\n"));
    /* CTRL_REG1_G sets output data rate, bandwidth, power-down and enables
    Bits[7:0]: DR1 DR0 BW1 BW0 PD Zen Xen Yen
    DR[1:0] - Output data rate selection
        00=95Hz, 01=190Hz, 10=380Hz, 11=760Hz
    BW[1:0] - Bandwidth selection (sets cutoff frequency)
         Value depends on ODR. See datasheet table 21.
    PD - Power down enable (0=power down mode, 1=normal or sleep mode)
    Zen, Xen, Yen - Axis enable (o=disabled, 1=enabled) */
    _register_write(CTRL_REG1_G, 0x0F); // Normal mode, enable all axes
    hal.scheduler->delay(1);

    /* CTRL_REG2_G sets up the HPF
    Bits[7:0]: 0 0 HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
    HPM[1:0] - High pass filter mode selection
        00=normal (reset reading HP_RESET_FILTER, 01=ref signal for filtering,
        10=normal, 11=autoreset on interrupt
    HPCF[3:0] - High pass filter cutoff frequency
        Value depends on data rate. See datasheet table 26.
    */
    _register_write(CTRL_REG2_G, 0x00); // Normal mode, high cutoff frequency
    hal.scheduler->delay(1);

    /* CTRL_REG3_G sets up interrupt and DRDY_G pins
    Bits[7:0]: I1_IINT1 I1_BOOT H_LACTIVE PP_OD I2_DRDY I2_WTM I2_ORUN I2_EMPTY
    I1_INT1 - Interrupt enable on INT_G pin (0=disable, 1=enable)
    I1_BOOT - Boot status available on INT_G (0=disable, 1=enable)
    H_LACTIVE - Interrupt active configuration on INT_G (0:high, 1:low)
    PP_OD - Push-pull/open-drain (0=push-pull, 1=open-drain)
    I2_DRDY - Data ready on DRDY_G (0=disable, 1=enable)
    I2_WTM - FIFO watermark interrupt on DRDY_G (0=disable 1=enable)
    I2_ORUN - FIFO overrun interrupt on DRDY_G (0=disable 1=enable)
    I2_EMPTY - FIFO empty interrupt on DRDY_G (0=disable 1=enable) */
    // Int1 enabled (pp, active low), data read on DRDY_G:
    _register_write(CTRL_REG3_G, 0x88);
    hal.scheduler->delay(1); 

    /* CTRL_REG4_G sets the scale, update mode
    Bits[7:0] - BDU BLE FS1 FS0 - ST1 ST0 SIM
    BDU - Block data update (0=continuous, 1=output not updated until read
    BLE - Big/little endian (0=data LSB @ lower address, 1=LSB @ higher add)
    FS[1:0] - Full-scale selection
        00=245dps, 01=500dps, 10=2000dps, 11=2000dps
    ST[1:0] - Self-test enable
        00=disabled, 01=st 0 (x+, y-, z-), 10=undefined, 11=st 1 (x-, y+, z+)
    SIM - SPI serial interface mode select
        0=4 wire, 1=3 wire */
    _register_write(CTRL_REG4_G, 0x00); // Set scale to 245 dps
    hal.scheduler->delay(1);

    /* CTRL_REG5_G sets up the FIFO, HPF, and INT1
    Bits[7:0] - BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
    BOOT - Reboot memory content (0=normal, 1=reboot)
    FIFO_EN - FIFO enable (0=disable, 1=enable)
    HPen - HPF enable (0=disable, 1=enable)
    INT1_Sel[1:0] - Int 1 selection configuration
    Out_Sel[1:0] - Out selection configuration */
    _register_write(CTRL_REG5_G, 0x00);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM9DS0::_initAccel()
{
    hal.console->printf(("initAccel\n"));
    /* CTRL_REG0_XM (0x1F) (Default value: 0x00)
    Bits (7-0): BOOT FIFO_EN WTM_EN 0 0 HP_CLICK HPIS1 HPIS2
    BOOT - Reboot memory content (0: normal, 1: reboot)
    FIFO_EN - Fifo enable (0: disable, 1: enable)
    WTM_EN - FIFO watermark enable (0: disable, 1: enable)
    HP_CLICK - HPF enabled for click (0: filter bypassed, 1: enabled)
    HPIS1 - HPF enabled for interrupt generator 1 (0: bypassed, 1: enabled)
    HPIS2 - HPF enabled for interrupt generator 2 (0: bypassed, 1 enabled)   */
    _register_write(CTRL_REG0_XM, 0x00);
    hal.scheduler->delay(1);

    /* CTRL_REG1_XM (0x20) (Default value: 0x07)
    Bits (7-0): AODR3 AODR2 AODR1 AODR0 BDU AZEN AYEN AXEN
    AODR[3:0] - select the acceleration data rate:
        0000=power down, 0001=3.125Hz, 0010=6.25Hz, 0011=12.5Hz, 
        0100=25Hz, 0101=50Hz, 0110=100Hz, 0111=200Hz, 1000=400Hz,
        1001=800Hz, 1010=1600Hz, (remaining combinations undefined).
    BDU - block data update for accel AND mag
        0: Continuous update
        1: Output registers aren't updated until MSB and LSB have been read.
    AZEN, AYEN, and AXEN - Acceleration x/y/z-axis enabled.
        0: Axis disabled, 1: Axis enabled                                    */ 
    _register_write(CTRL_REG1_XM, 0x57); // 100Hz data rate, x/y/z all enabled
    hal.scheduler->delay(1);

    //Serial.println(xmReadByte(CTRL_REG1_XM));
    /* CTRL_REG2_XM (0x21) (Default value: 0x00)
    Bits (7-0): ABW1 ABW0 AFS2 AFS1 AFS0 AST1 AST0 SIM
    ABW[1:0] - Accelerometer anti-alias filter bandwidth
        00=773Hz, 01=194Hz, 10=362Hz, 11=50Hz
    AFS[2:0] - Accel full-scale selection
        000=+/-2g, 001=+/-4g, 010=+/-6g, 011=+/-8g, 100=+/-16g
    AST[1:0] - Accel self-test enable
        00=normal (no self-test), 01=positive st, 10=negative st, 11=not allowed
    SIM - SPI mode selection
        0=4-wire, 1=3-wire                                                   */
    _register_write(CTRL_REG2_XM, 0x00); // Set scale to 2g
    hal.scheduler->delay(1);

    /* CTRL_REG3_XM is used to set interrupt generators on INT1_XM
    Bits (7-0): P1_BOOT P1_TAP P1_INT1 P1_INT2 P1_INTM P1_DRDYA P1_DRDYM P1_EMPTY
    */
    // Accelerometer data ready on INT1_XM (0x04)
    _register_write(CTRL_REG3_XM, 0x04); 
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM9DS0::_initMag()
{   

    hal.console->printf(("initMag\n"));
    // CTRL_REG5_XM enables temp sensor, sets mag resolution and data rate
    //Bits (7-0): TEMP_EN M_RES1 M_RES0 M_ODR2 M_ODR1 M_ODR0 LIR2 LIR1
    //TEMP_EN - Enable temperature sensor (0=disabled, 1=enabled)
    //M_RES[1:0] - Magnetometer resolution select (0=low, 3=high)
    //M_ODR[2:0] - Magnetometer data rate select
    //    000=3.125Hz, 001=6.25Hz, 010=12.5Hz, 011=25Hz, 100=50Hz, 101=100Hz
    //LIR2 - Latch interrupt request on INT2_SRC (cleared by reading INT2_SRC)
    //    0=interrupt request not latched, 1=interrupt request latched
    //LIR1 - Latch interrupt request on INT1_SRC (cleared by readging INT1_SRC)
    //    0=irq not latched, 1=irq latched                                     
    _register_write(CTRL_REG5_XM, 0x14); // Mag data rate - 100 Hz
    hal.scheduler->delay(1);

    // CTRL_REG6_XM sets the magnetometer full-scale
    //Bits (7-0): 0 MFS1 MFS0 0 0 0 0 0
    //MFS[1:0] - Magnetic full-scale selection
    //00:+/-2Gauss, 01:+/-4Gs, 10:+/-8Gs, 11:+/-12Gs                           
    _register_write(CTRL_REG6_XM, 0x00); // Mag scale to +/- 2GS
    hal.scheduler->delay(1);

    // CTRL_REG7_XM sets magnetic sensor mode, low power mode, and filters
    // AHPM1 AHPM0 AFDS 0 0 MLP MD1 MD0
    //AHPM[1:0] - HPF mode selection
    //    00=normal (resets reference registers), 01=reference signal for filtering, 
    //    10=normal, 11=autoreset on interrupt event
    //AFDS - Filtered acceleration data selection
    //    0=internal filter bypassed, 1=data from internal filter sent to FIFO
    //MLP - Magnetic data low-power mode
    //    0=data rate is set by M_ODR bits in CTRL_REG5
    //    1=data rate is set to 3.125Hz
    //MD[1:0] - Magnetic sensor mode selection (default 10)
    //    00=continuous-conversion, 01=single-conversion, 10 and 11=power-down 
    _register_write(CTRL_REG7_XM, 0x00); // Continuous conversion mode
    hal.scheduler->delay(1);

    // CTRL_REG4_XM is used to set interrupt generators on INT2_XM
    //Bits (7-0): P2_TAP P2_INT1 P2_INT2 P2_INTM P2_DRDYA P2_DRDYM P2_Overrun P2_WTM
    //
    _register_write(CTRL_REG4_XM, 0x04); // Magnetometer data ready on INT2_XM (0x08)
    hal.scheduler->delay(1);

    // INT_CTRL_REG_M to set push-pull/open drain, and active-low/high
    //Bits[7:0] - XMIEN YMIEN ZMIEN PP_OD IEA IEL 4D MIEN
    //XMIEN, YMIEN, ZMIEN - Enable interrupt recognition on axis for mag data
    //PP_OD - Push-pull/open-drain interrupt configuration (0=push-pull, 1=od)
    //IEA - Interrupt polarity for accel and magneto
    //    0=active-low, 1=active-high
    //IEL - Latch interrupt request for accel and magneto
    //    0=irq not latched, 1=irq latched
    //4D - 4D enable. 4D detection is enabled when 6D bit in INT_GEN1_REG is set
    //MIEN - Enable interrupt generation for magnetic data
    //    0=disable, 1=enable) 
    _register_write(INT_CTRL_REG_M, 0x09); // Enable interrupts for mag, active-low, push-pull
    hal.scheduler->delay(1);
}


bool AP_InertialSensor_LSM9DS0::_hardware_init(void)
{
    hal.console->println_P(PSTR("_hardware_init:"));
    // Store the resolutions in private variables
    _calcgRes(G_SCALE_245DPS); // Calculate DPS / ADC tick, stored in gRes variable
    _calcmRes(M_SCALE_2GS); // Calculate Gs / ADC tick, stored in mRes variable
    _calcaRes(A_SCALE_2G); // Calculate g / ADC tick, stored in aRes variable

    if (!_spi_sem->take(100)) {
        hal.scheduler->panic(PSTR("LSM9DS0: Unable to get semaphore"));
    }

    // initially run the bus at low speed (500kHz on APM2)
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);

    // Init the sensors
    _initGyro();
    _initAccel();
    _initMag();

   // To verify communication, we can read from the WHO_AM_I register of
    // each device.    
    uint8_t gTest = _register_read(WHO_AM_I_G);      // Read the gyro WHO_AM_I
    uint8_t xmTest = _register_read(WHO_AM_I_XM);   // Read the accel/mag WHO_AM_I

    // now that we have initialised, we set the SPI bus speed to high
    // (8MHz on APM2)
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);

    _spi_sem->give();

    return true;

}
// TODO check the registers, dump first the Gyro registers and then the Mag registers

// dump all config registers - used for debug
void AP_InertialSensor_LSM9DS0::_dump_registers(void)
{
    hal.console->println_P(PSTR("LSM9DS0 registers:"));
    hal.console->println_P(PSTR("Gyroscope registers:"));
    const uint8_t first = OUT_TEMP_L_XM;
    const uint8_t last = ACT_DUR;
    for (uint8_t reg=first; reg<=last; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
        if ((reg - (first-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();

    hal.console->println_P(PSTR("Accelerometer and Magnetometers registers:"));
    for (uint8_t reg=first; reg<=last; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
        if ((reg - (first-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();
}

#endif // CONFIG_HAL_BOARD
