#ifndef BERRYIMU_H
#define BERRYIMU_H

#include <stdint.h>
#include <fcntl.h>
#include <math.h>
#include <string>

#include <chrono>       // Needed for waiting time required for ...
#include <thread>       // ... temperature & pressure sensors

#ifdef __linux__
	#include <linux/i2c-dev.h>
#else
	#include "i2cdummy.h"   // REPLACE
#endif

/*  Usage:
 * *                    REPLACE include i2cdummy.h by the installed i2c protocol. Needs to implement
 *                          ioctl(int, int, int);
                            i2c_smbus_read_i2c_block_data(int,int,int,uint8_t*);
                            i2c_smbus_read_byte_data(int,int);
                            i2c_smbus_write_byte_data(int , int , int );

    This class provides the direct interaction with the IMU sensor.

 *  class BerryIMU:
 *  enableIMU() : activates i2c protocol and enables all sensors with the parameters set in
 *                  _accState, _magState, _gyrState, _configState
 *  configMag(..),configAcc(..),configGyr(..)
 *              : modify content of accState,magState,gyrState,configState and send it to the sensor
 *
 *  bool read(sensor_type type, int * output, bool reuse_device=false):
 *              : reads data to output. acc&mag can reuse_device
 *              : syntactic sugar: readACC,readMAG,readGYR
 *
 *  void readTandP(float &T, float &P): Reads temp & pressure, this function sleeps at least 10ms!!
 *  char * getLastMessage(): reads last generated message (warning error etc)
 *  Helper functions:
 *  getAltitude : implements barimetric formula
 *
 *
 *  This namespage defines a number of enums and constants which correspond to the allowed values of the hardware parameters
 *      and addresses
 *
 * *  Future changes:
 *                  - add modifier functions for all parameters which in turn call the adequate configX function
 *                  - CTRL_REG0_XM : set fifo mode
 *                  - CTRL_REG4_XM is used to set interrupt generators on INT2_XM
                        Bits (7-0): P2_TAP P2_INT1 P2_INT2 P2_INTM P2_DRDYA P2_DRDYM P2_Overrun P2_WTM
                        xmWriteByte(CTRL_REG4_XM, 0x04); // Magnetometer data ready on INT2_XM (0x08)
                    - use https://github.com/sparkfun/SparkFun_LSM9DS0_Arduino_Library/blob/master/src/SFE_LSM9DS0.cpp

                      =>FIFO usage:
                      ==>https://www.raspberrypi.org/forums/viewtopic.php?t=111710&p=766487
 * */

namespace BerryIMU {

static double getAltitude(float pressure) {// uses barometric formula to calculate altitude in [m] from [mBar]
        double  p0 = 1013.25; // pressure at sea level
        return 44330.*(1-pow(pressure/p0,1./5.255)) ;// international barometric formula
    }


// ----------------------------------------------------
// The following constants are understood by the sensor
// ----------------------------------------------------
enum acc_selftest_mode { A_TEST_OFF=0, A_TEST_POSITIVE_SIGN= 1, A_TEST_NEGATIVE_SIGN = 2, A_TEST_THIS_IS_NOT_ALLOWED=3}; // for acc
enum gyr_selftest_mode { G_TEST_OFF=0, G_TEST_XPositiveYZNegative=1, G_TEST_XNegativeYZPositive=3}; // For gyr
enum acc_aa_bandwidth { A_BANDWIDTH_773Hz=0, A_BANDWIDTH_194Hz=1, A_BANDWIDTH_362Hz=2, A_BANDWIDTH_50Hz=3 };
enum acc_scale { A_SCALE_2g = 0, // maximal range of accelerometer ( in multiples of g=9.81m/s^2 )
                 A_SCALE_4g = 1,
                 A_SCALE_6g = 2,
                 A_SCALE_8g = 3,
                 A_SCALE_16g = 4 };
// accel_oder defines all possible output data rates of the accelerometer:
enum acc_odr
{
    A_POWER_DOWN=0, 	// Power-down mode (0x0)
    A_ODR_3p125Hz,    // 3.125 Hz	(0x1)
    A_ODR_6p25Hz,		// 6.25 Hz (0x2)
    A_ODR_12p5Hz,		// 12.5 Hz (0x3)
    A_ODR_25Hz,		// 25 Hz (0x4)
    A_ODR_50Hz,		// 50 Hz (0x5)
    A_ODR_100Hz,		// 100 Hz (0x6)
    A_ODR_200Hz,		// 200 Hz (0x7)
    A_ODR_400Hz,		// 400 Hz (0x8)
    A_ODR_800Hz,		// 800 Hz (9)
    A_ODR_1600Hz		// 1600 Hz (0xA)
};

enum mag_odr { M_ODR_3p125Hz=0, M_ODR_6p25Hz, M_ODR_12p5Hz=2, // table 84
                   M_ODR_25Hz,M_ODR_50Hz=4,M_ODR_100Hz};
enum mag_scale { M_SCALE_2Gs=0,M_SCALE_4Gs, M_SCALE_8Gs, M_SCALE_12Gs};
enum mag_resolution {M_LOW_RES = 0, M_HIGH_RES=0b11};
enum mag_filter_acceleration {M_FILTER_INTERNAL_BYPASSED = 0, M_FILTER_FROM_INTERNAL_TO_OUTPUT_AND_FIFO = 1};
enum mag_sensor_mode {M_SENSOR_CONTINUOUS_CONVERSION = 0, M_SENSOR_SINGLE_CONVERSION= 1, M_SENSOR_POWER_DOWN=2, M_SENSOR_POWER_DOWN_B=3}; // Tab 91. diff between A&B unclear
enum mag_power_mode { M_POWER_HIGH=0,M_POWER_LOW = 1}; // Tab. 89

enum gyr_scale { G_SCALE_245dps =0, G_SCALE_500dps, G_SCALE_2000dps=2, G_SCALE_2000dpsB=3  };

enum gyro_odr               // 4 bit
{							// ODR (Hz) --- Cutoff
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

// Max 7%. or 2^(-k) thereof. see tab??
enum gyr_high_pass { G_HIGH_MAX, G_HIGH_1_2, G_HIGH_1_4, G_HIGH_1_8, G_HIGH_1_16, G_HIGH_1_32, G_HIGH_1_64,
                     G_HIGH_1_128, G_HIGH_1_256, G_HIGH_1_512, G_HIGH_1_1024};

//bdu : Block data update for acceleration and magnetic data. Default value: 0
//       (0: continuous update; 1: output registers not updated until MSB and LSB have been read)
enum acc_bdu { A_CONTINUOUS_UPDATE=0, A_UPDATE_AFTER_MSB_LSB_READ=1 };
enum gyr_bdu { G_CONTINUOUS_UPDATE=0, G_UPDATE_AFTER_MSB_LSB_READ=1 };

enum gyr_power_mode { G_POWER_DOWN = 0, G_POWER_NORMAL= 1, G_POWER_SLEEP = 0b11}; // Sleep disables all axes

enum pressure_oversampling {T_ULTRA_LOW_POWER=0, T_STANDARD, T_HIGH_RESOLUTION,T_ULTRA_HIGH_RESOLUTION};

// DONT UNDERSTAND YET:
enum HighPassMode {NormalResetting=0, ReferenceSignalForFiltering, Normal=2, AutoResetOnInterrupt=3};
// ----------------------------------------------------

// Types that define the state of each sensor
struct AccState { // Table 71 and 75 for registers CTRL_REG1_XM and CTRL_REG2_XM
    acc_scale        scale    = A_SCALE_2g;
    acc_odr          odr      = A_ODR_1600Hz;
    acc_aa_bandwidth aa_bw    = A_BANDWIDTH_773Hz;
    acc_bdu          bdu      = A_CONTINUOUS_UPDATE;
    acc_selftest_mode selftest= A_TEST_OFF;
    bool spi_interface_mode = false;
    bool enableX =1,enableY=1,enableZ=1;
};

struct MagState {
    mag_scale scale                 = M_SCALE_4Gs;
    mag_odr odr                     = M_ODR_100Hz;
    mag_resolution resolution       = M_HIGH_RES;
    mag_power_mode power            = M_POWER_HIGH;
    mag_sensor_mode sensormode      = M_SENSOR_CONTINUOUS_CONVERSION;
    mag_filter_acceleration filter  = M_FILTER_INTERNAL_BYPASSED;
    HighPassMode highpass           = NormalResetting;
};

struct GyrState {
    gyr_scale scale              = G_SCALE_2000dps;
    gyro_odr  odr                = G_ODR_760_BW_30;
    gyr_power_mode power         = G_POWER_NORMAL;

    gyr_high_pass highpasscutoff = G_HIGH_MAX;
    HighPassMode highpassmode    = NormalResetting;

    gyr_selftest_mode selftest   = G_TEST_OFF;
    gyr_bdu bdu                  = G_CONTINUOUS_UPDATE;
    bool spi_interface_mode = 0;
    bool enableX=1,enableY=1,enableZ=1;
};


struct configuration {
    bool bigendian = false;
    bool temperature_sensor_activated = true;
    pressure_oversampling oversampling=T_ULTRA_LOW_POWER; // notice that low power is fastest.
};

class BerryIMU
{
    enum sensor_type {ACC, GYR, MAG, TP};
    // The following enums contain the bit flags which are used to set
    // the mode in the device

public:
    BerryIMU();
    bool enableIMU();
    bool disableIMU();

    bool isEnabled();


    // if reuse_device, do not call select device. useful since MAG_ADDRESS==ACC_ADDRESS
    bool readRaw(sensor_type type, int16_t * output, bool reuse_device=false);
    bool read(sensor_type type, double * output, bool reuse_device=false);
    inline bool readACC(double *output,bool reuse_device=false)  { return read(ACC, output,reuse_device); }
    inline bool readMAG(double *output,bool reuse_device=false)  { return read(MAG, output,reuse_device); }
    inline bool readGYR(double *output,bool reuse_device=false)  { return read(GYR, output,reuse_device); }


    bool enableTemperatureSensor();
    bool disableTemperatureSensor();

    void readTandP(float & T, float & P,bool reuse_device=false, bool only_T=false, bool reuse_T=false);
    void readP(float & P,bool reuse_device = false);

    void readT(float & T,bool reuse_device = false);
    void readTlsm(float &T);                 // reads T from LSM9DS0

    bool setDatarate(mag_odr datarate);
    bool setDatarate(gyro_odr datarate);
    bool setDatarate(acc_odr datarate);

    bool configureMag(mag_odr datarate, mag_resolution mag_resolution = M_HIGH_RES,
                        bool temperature=true, bool latch_interrupt_on_int1_src=0,bool latch_interrupt_on_int2_src=0) ;
    bool configureMag(mag_scale scale);
    bool configureMag(HighPassMode highpass, mag_filter_acceleration filter=M_FILTER_INTERNAL_BYPASSED,
                        mag_sensor_mode sensormode=M_SENSOR_CONTINUOUS_CONVERSION,mag_power_mode powermode=M_POWER_HIGH);

    bool configureGyr(gyro_odr odr, gyr_power_mode pd=G_POWER_NORMAL, bool enableX=true, bool enableY=true, bool enableZ=true);
    bool configureGyr(HighPassMode hpm, gyr_high_pass hpcf);
    bool configureGyr(gyr_scale scale, gyr_bdu bdu=G_CONTINUOUS_UPDATE, bool bigEndian = false,
                        gyr_selftest_mode selftest=G_TEST_OFF, bool spi_interface_mode = 0) ;

    bool configureAcc(acc_odr datarate, acc_bdu bdu = A_CONTINUOUS_UPDATE, bool enableX = true,bool enableY = true,bool enableZ = true);
    bool configureAcc(acc_scale scale, acc_aa_bandwidth anti_alias_bandwidth=A_BANDWIDTH_773Hz,
                        acc_selftest_mode selftest=A_TEST_OFF, bool spi_interface_mode = 0);

    void enableFIFO(sensor_type type);
    void disableFIFO(sensor_type type);
    int pollFIFO(sensor_type type);
    const char * getLastMessage();
    AccState _accState;
    MagState _magState;
    GyrState _gyrState;
    configuration _configState;

    bool getSensorID(uint8_t &chip_id, uint8_t &version);


private:
    float gain(sensor_type type);
    int m_i2c_file = -1;
    float m_last_temperature_reading = 0.;
    std::string m_message;
    bool m_fifo_g=false,m_fifo_a=false;

    // Helper functions
    void decoupleDataBlock(int16_t * a, uint8_t * block) {
            *a = (int16_t)(block[0] | block[1] << 8);
        *(a+1) = (int16_t)(block[2] | block[3] << 8);
        *(a+2) = (int16_t)(block[4] | block[5] << 8);
    }
    void usleep(int microseconds) {
        std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
    }
    void msleep(int milliseconds) {
        usleep(1000*milliseconds);
    }
    bool selectDevice(sensor_type type);
    bool selectDevice(int file, int addr);

    bool readBlock(uint8_t command, uint8_t size, uint8_t *data);
    void readReg(uint8_t command, uint8_t & data);
    bool writeReg(sensor_type type, uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t command);

    inline int getWaitingTimeTemperature() {
        //Waiting time in us for reading temperature values
        return 4500; // time.sleep(0.0045) was in python
    }

    int getWaitingTimePressure(pressure_oversampling oversampling) {
        /*
         * Waiting times in us for reading pressure values
         */
        const int extra_time = 0; // To check wether I dnt wait enough
        switch (oversampling) {
        case 0: return extra_time+4500;
        case 1: return extra_time+7500;
        case 2: return extra_time+13500;
        case 3: return extra_time+25500;
        }
        return -1;
    }

    void setMessage(std::string msg) {
        m_message = msg;
    }
};

//------------------------------------------------
// Pressure & Temp sensor BMP180
//------------------------------------------------
const int BMP180_CTRL = 0xF4; // AC register
const int BMP180_REG_PRE = 0xF6; // * Pressure register
const int BMP180_COMMAND_TEMPERATURE = 0x2E;
const int BMP180_COMMAND_PRESSURE = 0x34;
//-----------------------------------------------------------------------
// Constants to interact with LSM9DS0
//-----------------------------------------------------------------------
//I2C addresses
const int MAG_ADDRESS = 0x1E; // use i2c detection tool to determine
const int ACC_ADDRESS = 0x1E;
const int GYR_ADDRESS = 0x6A;
const int TP_ADDRESS  = 0x77;

const int READ_MULTIPLE_BYTES_FLAG = 0x80; // See Sec.6.1.1 datasheet

//------------------------------------------------
/**LSM9DS0GyroRegisters**/

const int WHO_AM_I_G  = 0x0F;
const int CTRL_REG1_G = 0x20;
const int CTRL_REG2_G = 0x21;
const int CTRL_REG3_G = 0x22;
const int CTRL_REG4_G = 0x23;
const int CTRL_REG5_G = 0x24;
const int REFERENCE_G = 0x25;
const int STATUS_REG_G = 0x27;
const int OUT_X_L_G = 0x28;
const int OUT_X_H_G = 0x29;
const int OUT_Y_L_G = 0x2A;
const int OUT_Y_H_G = 0x2B;
const int OUT_Z_L_G = 0x2C;
const int OUT_Z_H_G = 0x2D;
const int FIFO_CTRL_REG_G = 0x2E;
const int FIFO_SRC_REG_G = 0x2F;
const int INT1_CFG_G = 0x30;
const int INT1_SRC_G = 0x31;
const int INT1_THS_XH_G = 0x32;
const int INT1_THS_XL_G = 0x33;
const int INT1_THS_YH_G = 0x34;
const int INT1_THS_YL_G = 0x35;
const int INT1_THS_ZH_G = 0x36;
const int INT1_THS_ZL_G = 0x37;
const int INT1_DURATION_G = 0x38;

//------------------------------------------------
//LSM9DS0Accel/Magneto(XM)Registers//

const int OUT_TEMP_L_XM = 0x05;
const int OUT_TEMP_H_XM = 0x06;
const int STATUS_REG_M = 0x07;
const int OUT_X_L_M = 0x08;
const int OUT_X_H_M = 0x09;
const int OUT_Y_L_M = 0x0A;
const int OUT_Y_H_M = 0x0B;
const int OUT_Z_L_M = 0x0C;
const int OUT_Z_H_M = 0x0D;
const int WHO_AM_I_XM = 0x0F;
const int INT_CTRL_REG_M = 0x12;
const int INT_SRC_REG_M = 0x13;
const int INT_THS_L_M = 0x14;
const int INT_THS_H_M = 0x15;
const int OFFSET_X_L_M = 0x16;
const int OFFSET_X_H_M = 0x17;
const int OFFSET_Y_L_M = 0x18;
const int OFFSET_Y_H_M = 0x19;
const int OFFSET_Z_L_M = 0x1A;
const int OFFSET_Z_H_M = 0x1B;
const int REFERENCE_X = 0x1C;
const int REFERENCE_Y = 0x1D;
const int REFERENCE_Z = 0x1E;
const int CTRL_REG0_XM = 0x1F;
const int CTRL_REG1_XM = 0x20;
const int CTRL_REG2_XM = 0x21;
const int CTRL_REG3_XM = 0x22;
const int CTRL_REG4_XM = 0x23;
const int CTRL_REG5_XM = 0x24;
const int CTRL_REG6_XM = 0x25;
const int CTRL_REG7_XM = 0x26;
const int STATUS_REG_A = 0x27;
const int OUT_X_L_A = 0x28;
const int OUT_X_H_A = 0x29;
const int OUT_Y_L_A = 0x2A;
const int OUT_Y_H_A = 0x2B;
const int OUT_Z_L_A = 0x2C;
const int OUT_Z_H_A = 0x2D;
const int FIFO_CTRL_REG = 0x2E;
const int FIFO_SRC_REG = 0x2F;
const int INT_GEN_1_REG = 0x30;
const int INT_GEN_1_SRC = 0x31;
const int INT_GEN_1_THS = 0x32;
const int INT_GEN_1_DURATION = 0x33;
const int INT_GEN_2_REG = 0x34;
const int INT_GEN_2_SRC = 0x35;
const int INT_GEN_2_THS = 0x36;
const int INT_GEN_2_DURATION = 0x37;
const int CLICK_CFG = 0x38;
const int CLICK_SRC = 0x39;
const int CLICK_THS = 0x3A;
const int TIME_LIMIT = 0x3B;
const int TIME_LATENCY = 0x3C;
const int TIME_WINDOW = 0x3D;

}
#endif // BERRYIMU_H
