#include "berryimu.h"
#include <unistd.h> // for close
#include <QDebug>

namespace BerryIMU {
BerryIMU::BerryIMU()    {
    // Default settings for sensor
    _accState.odr   = A_ODR_200Hz;
    _accState.scale = A_SCALE_2g;
    _accState.aa_bw = A_BANDWIDTH_50Hz;

    _magState.scale = M_SCALE_2Gs;
    _magState.odr   = M_ODR_100Hz;

    _gyrState.scale = G_SCALE_245dps;
    _gyrState.odr   = G_ODR_190_BW_125;
    _gyrState.highpasscutoff = G_HIGH_MAX;

    _configState.temperature_sensor_activated = false;
}
bool BerryIMU::readBlock(uint8_t command, uint8_t size, uint8_t *data)
{
    int result = i2c_smbus_read_i2c_block_data(m_i2c_file, command, size, data);
    if (result != size)
    {
        setMessage("Failed to read block from I2C.: %i");
        return false;
    }
    return true;
}

void BerryIMU::readReg(uint8_t command, uint8_t & data) {
    data = i2c_smbus_read_byte_data(m_i2c_file, command);
}

uint8_t BerryIMU::readReg(uint8_t command) {
    return i2c_smbus_read_byte_data(m_i2c_file, command);
}
bool BerryIMU::selectDevice(sensor_type type)    {
    int addr;
    switch (type) {
    case ACC:
        addr = ACC_ADDRESS;            break;
    case MAG:
        addr = MAG_ADDRESS;            break;
    case GYR:
        addr = GYR_ADDRESS;            break;
    case TP:
        addr = TP_ADDRESS;             break;
    }
    if (ioctl(m_i2c_file, I2C_SLAVE, addr) < 0) {
        setMessage("Failed to select I2C device.");
        return false;
    }
    return true;
}
bool BerryIMU::selectDevice(int file, int addr)    {
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        setMessage("Failed to select I2C device.");
        return false;
    }
    return true;
}

bool BerryIMU::readRaw(sensor_type type, int16_t * output, bool reuse_device) {
    //readBlock(0x80 | OUT_X_L_A, sizeof(block), block);
    /*
    An array of 6 bytes is first created to store the values.
    The I2C slave address is selected for the accelerometer, by passing the accelerometer address of ACC_ADDRESS or 0x1E  to the selectDevice() function.
    Using the readBlock() function from i2c-dev.h, we read 6 bytes starting at OUT_X_L_A (0x28). This is shown on page 61 of the datasheet.
    The values are expressed in 2’s complement (MSB for the sign and then 15 bits for the value) so we need to combine;
    block[0] & block[1] for X axis
    block[2] & block[3] for Y axis
    block[4] & block[5] for Z axis
    */
    int register_address = READ_MULTIPLE_BYTES_FLAG;
    uint8_t block[6];
    if(!reuse_device)
        selectDevice(type);
    switch (type) {
    case ACC:
        register_address |= OUT_X_L_A;            break;
    case MAG:
        register_address |= OUT_X_L_M;            break;
    case GYR:
        register_address |= OUT_X_L_G;            break;
    case TP:
        setMessage("Warning: Use dedicated function readTandP(..)");
        return false;
    default:
        throw("unknown sensor_type");
    }
    if( readBlock(register_address, sizeof(block), block) ) {
        decoupleDataBlock(output, block);
        return true;
    }
    return false;
}

bool BerryIMU::writeReg(sensor_type type, uint8_t reg, uint8_t value) {
    selectDevice(type);
    if (-1==i2c_smbus_write_byte_data(m_i2c_file, reg, value)) {
        setMessage ("Failed to write byte to I2C.");
        return false;
    }
    return true;
}



bool BerryIMU::configureMag(mag_odr datarate, mag_resolution mag_resolution, bool temperature,
                                bool latch_interrupt_on_int1_src,bool latch_interrupt_on_int2_src) {
    // Configuration according to Tab. 83
    // Bitmasking just in case
    _magState.odr   = datarate;
    _magState.resolution = mag_resolution;
    _configState.temperature_sensor_activated = temperature;

    uint8_t value;
    value =  0b1 & latch_interrupt_on_int1_src;
    value|= (0b1 & latch_interrupt_on_int2_src)<<1;
    value|= (0b111 & datarate)<<2;
    value|= (0b11  & mag_resolution)<<5;
    value|= (0b1   & temperature)<<7;
    return writeReg(MAG,CTRL_REG5_XM, value);
    //writeReg(MAG, CTRL_REG5_XM, 0b1 11 100 00);   // Temp enable, high res, Mag data rate = 50Hz
}
bool BerryIMU::configureMag(mag_scale scale) {
    // Configuration according to Tab. 85
    // Bitmasking just in case
    _magState.scale = scale;
    uint8_t value;
    value =  (0b11 & scale)<<5;
    return writeReg(MAG,CTRL_REG6_XM, value);
}

bool BerryIMU::configureMag(HighPassMode highpass, mag_filter_acceleration filter,
                                mag_sensor_mode sensormode, mag_power_mode powermode) {
    // Configuration according to Tab. 88
    // Bitmasking just in case
    _magState.sensormode = sensormode;
    _magState.power      = powermode;
    _magState.filter     = filter;
    _magState.highpass   = highpass;
    uint8_t value;
    if (powermode == M_POWER_LOW && _accState.odr != A_ODR_3p125Hz) // this is automatically set by sensor
        setMessage("Mag datarate has been temporarily changed due to power saving mode.");
    value =  (0b11 & sensormode);
    value|=  (0b1  & powermode)<<2;
    value|=  (0b1  & filter)<<5; // gap for 2 zero bits
    value|=  (0b11  & highpass)<<6;
    return writeReg(MAG,CTRL_REG7_XM, value);
}

bool  BerryIMU::configureGyr(gyro_odr odr, gyr_power_mode pd, bool enableX, bool enableY, bool enableZ) {
    // dr & bw set the data rate (ODR) and cutoff for low-pass filter (Cutoff)
    _gyrState.odr = odr;
    _gyrState.power    = pd;
    _gyrState.enableX  = enableX;
    _gyrState.enableY  = enableY;
    _gyrState.enableZ  = enableZ;
    uint8_t value;
    if (pd==G_POWER_SLEEP) { // will be overwritten acc. to Tab. 22
        enableX = 0; enableY = 0; enableZ = 0;
    }
    value = 0b1 & enableY;
    value|= (0b1 & enableX)<<1;
    value|= (0b1 & enableZ)<<2;
    value|= (0b1 & pd)<<3;
    value|= (0b1111 & odr)<<4;
    return writeReg(GYR,CTRL_REG1_G, value);
}

bool BerryIMU::configureGyr(HighPassMode hpm, gyr_high_pass hpcf) {
    // Tab. 23
    _gyrState.highpassmode   = hpm;
    _gyrState.highpasscutoff = hpcf;
    uint8_t value;
    value = 0b1111 & hpcf;
    value|= (0b11 & hpm)<<4;
    return writeReg(GYR,CTRL_REG2_G, value);
}

bool BerryIMU::configureAcc(acc_odr datarate, acc_bdu bdu, bool enableX,bool enableY ,bool enableZ ) {
    // Configuration according to Tab. 71
    // Bitmasking just in case
    // bdu : Block data update for acceleration and magnetic data. Default value: 0
    //       (0: continuous update; 1: output registers not updated until MSB and LSB have been read)
    _accState.odr     = datarate;
    _accState.bdu     = bdu;
    _accState.enableX = enableX;
    _accState.enableY = enableY;
    _accState.enableZ = enableZ;

    uint8_t value;
    value = 0b1 & enableX;
    value|= (0b1 & enableY)<<1;
    value|= (0b1 & enableZ)<<2;
    value|= (0b1 & bdu)<<3;
    value|= (0b1111 & datarate)<<4;
    return writeReg(ACC,CTRL_REG1_XM, value);
}

bool BerryIMU::configureGyr(gyr_scale scale, gyr_bdu bdu, bool bigEndian ,
                  gyr_selftest_mode selftest, bool spi_interface_mode ) {
    // bdu : Block data update def=0 (0: continuous update; 1: output registers not updated until MSB and LSB have been read)
    // spi: 0=4 wire interface, 1=3 wire interface
    _gyrState.scale              = scale;
    _gyrState.selftest           = selftest;
    _gyrState.bdu                = bdu;
    _gyrState.spi_interface_mode = spi_interface_mode;
    _configState.bigendian       = bigEndian;

    uint8_t value=0;
    value = 1 & spi_interface_mode;
    value|= (0b11  & selftest)<<1;
    value|= (0b11 & scale)<<(3+1); // 0bit at position 4
    value|= (0b1 & bigEndian)<<6;
    value|= (0b1   & bdu)<<7;
    return writeReg(GYR,CTRL_REG4_G, value);
    //writeReg(GYR,CTRL_REG4_G, 0b0 0 11 0 00 0); // Continuos update, 2000 dps full scale
}

bool BerryIMU::configureAcc(acc_scale scale, acc_aa_bandwidth anti_alias_bandwidth,
                  acc_selftest_mode selftest, bool spi_interface_mode ) {
    // Configuration according to Tab. 74
    // Bitmasking just in case
    _accState.scale              = scale;
    _accState.selftest           = selftest;
    _accState.aa_bw              = anti_alias_bandwidth;
    _accState.spi_interface_mode = spi_interface_mode;

    uint8_t value=0;
    value = 1 & spi_interface_mode;
    value|= (0b11  & selftest)<<1;
    value|= (0b111 & scale)<<3;
    value|= (0b11  & anti_alias_bandwidth)<<6;

    return writeReg(ACC,CTRL_REG2_XM, value);
}

void BerryIMU::enableFIFO(BerryIMU::sensor_type type)
{
    uint8_t c;
    switch (type) {
    case GYR:
        c = readReg(CTRL_REG5_G);
        writeReg(GYR, CTRL_REG5_G, c | 0x40);         // Enable gyro FIFO
        msleep(20);                                 // Wait for change to take effect
        writeReg(GYR, FIFO_CTRL_REG_G, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
        m_fifo_g = true;
        // delay 1000 milliseconds to collect FIFO samples
        break;
    case ACC:
        c = readReg(CTRL_REG0_XM);
        writeReg(ACC, CTRL_REG0_XM, c | 0x40);         // Enable gyro FIFO
        msleep(20);                                 // Wait for change to take effect
        writeReg(ACC, FIFO_CTRL_REG, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
        m_fifo_a = true;
        // delay 1000 milliseconds to collect FIFO samples
        break;
    default:
        setMessage("No FIFO for Magnetometer available.");
        break;
    }

}

void BerryIMU::disableFIFO(BerryIMU::sensor_type type)
{
    uint8_t c;
    switch(type) {
    case GYR:
        c = readReg(CTRL_REG5_G);
        writeReg(GYR,CTRL_REG5_G, c & ~0x40);  // Disable gyro FIFO
        msleep(20);
        writeReg(GYR,FIFO_CTRL_REG_G, 0x00);   // Enable gyro bypass mode
        m_fifo_g = false;
        break;
    case ACC:
        c = readReg(CTRL_REG0_XM);
        writeReg(GYR,CTRL_REG0_XM, c & ~0x40);  // Disable acc FIFO
        msleep(20);
        writeReg(GYR,FIFO_CTRL_REG, 0x00);   // Enable acc bypass mode
        m_fifo_a = false;
        break;
    }


}

int BerryIMU::pollFIFO(BerryIMU::sensor_type type) {
    uint8_t command;
    switch(type) {
    case GYR:
        if(!m_fifo_g)
            return -1;
        command = FIFO_SRC_REG_G;
        break;
    case ACC:
        if(!m_fifo_a)
            return -1;
        command = FIFO_SRC_REG;
        break;
    default:
        setMessage("Error: pollFIFO only defined for GYR and ACC");
        return -1;
    }
    // Read number of stored samples. They can be accessed using a for loop over read(...)
    return (readReg(command) & 0x1F);
}



const char *BerryIMU::getLastMessage()
{
    return m_message.c_str();
}


float BerryIMU::gain(sensor_type type) {
   switch(type) {
    case ACC:
       // Possible gains
       switch(_accState.scale) {
        case A_SCALE_2g: return 0.061;
        case A_SCALE_4g: return 0.122;
        case A_SCALE_6g: return 0.183;
        case A_SCALE_8g: return 0.244;
        case A_SCALE_16g:return 0.732;
       default: throw("accscale not defined");
       }

   case GYR:
       switch(_gyrState.scale) {
        case G_SCALE_245dps: return 8.75;
        case G_SCALE_500dps: return 17.50;
        case G_SCALE_2000dps:return 70.;
       default: throw("gyrscale not defined");
       }
   case MAG:
       switch(_magState.scale) {
        case M_SCALE_2Gs: return 0.08;
        case M_SCALE_4Gs: return 0.16;
        case M_SCALE_8Gs: return 0.32;
        case M_SCALE_12Gs:return 0.48;
       default: throw("magscale not defined");
       }
   default:
       throw("sensor_type not recognised");
   }
}


bool BerryIMU::disableIMU() {
    bool ret = close(m_i2c_file);
    if(ret)
        m_i2c_file = -1;
    return ret;
}

bool BerryIMU::read(BerryIMU::sensor_type type, double *output, bool reuse_device) {
    int16_t raw_signal[3];
    bool ret = readRaw(type, raw_signal,reuse_device);
    double _gain = gain(type);
    output[0] = raw_signal[0]*_gain;
    output[1] = raw_signal[1]*_gain;
    output[2] = raw_signal[2]*_gain;
    return ret;
}

bool BerryIMU::isEnabled() {
    return (m_i2c_file>=0);
}

bool BerryIMU::enableIMU() {
        if(m_i2c_file>=0)    // If enabled, disable first
            disableIMU();

        char filename[20];
        sprintf(filename, "/dev/i2c-%d", 1);
        m_i2c_file = open(filename, O_RDWR);
        if (m_i2c_file<0) {
            setMessage("Unable to open I2C bus!");
            return false;
        }
        // Enable accelerometer.
        configureAcc(_accState.scale,_accState.aa_bw,_accState.selftest,_accState.spi_interface_mode);
        configureAcc(_accState.odr,_accState.bdu,_accState.enableX,_accState.enableY,_accState.enableZ);
        //Enable the magnetometer
        configureMag(_magState.highpass,_magState.filter,_magState.sensormode,_magState.power);
        configureMag(_magState.scale);
        configureMag(_magState.odr,_magState.resolution,_configState.temperature_sensor_activated);
        // Enable Gyro
        configureGyr(_gyrState.scale,_gyrState.bdu,_configState.bigendian,_gyrState.selftest,_gyrState.spi_interface_mode);
        configureGyr(_gyrState.highpassmode,_gyrState.highpasscutoff);
        configureGyr(_gyrState.odr,_gyrState.power,_gyrState.enableX,_gyrState.enableY,_gyrState.enableZ);
        return true;
    }



// return two bytes from data as a signed 16-bit value
int16_t get_short(uint8_t * data, int index) {
    return ((data[index] << 8) + data[index + 1]);
}
uint16_t get_ushort(uint8_t * data, int index) {
    return ((data[index] << 8) + data[index + 1]);
}


bool BerryIMU::getSensorID(uint8_t &chip_id, uint8_t & version) {
    uint8_t buffer[2];
    selectDevice(TP);
    if(!i2c_smbus_read_i2c_block_data(TP_ADDRESS, 0xD0, 2, buffer)) {
        setMessage("Error:  Could not read sensor ID and Version.");
        return false;
    }
    chip_id = buffer[1];
    version = buffer[0];
    return true;
}

bool BerryIMU::enableTemperatureSensor() {
    return configureMag(_magState.odr, _magState.resolution, true );
}
bool BerryIMU::disableTemperatureSensor() {
    return configureMag(_magState.odr, _magState.resolution, false );
}


void BerryIMU::readTandP(float & T, float & P,bool reuse_device, bool only_T, bool reuse_T){
#ifdef _WIN32
    return;
#endif
        //# Print temperature & pressure & chip data
        int oversampling = _configState.oversampling;
        if(!reuse_device)
            selectDevice(TP);

        //# Read whole calibration EEPROM data [only 1x]
        const int BUFFER_SIZE = 22;
        static uint8_t cal[BUFFER_SIZE];
        static bool calibrated = false;
        //BMP180_COMMAND_TEMPERATURE (0x2E) to the register BMP180_REG_CONTROL (0xF4)
        //BMP180_COMMAND_PRESSURE (0xF4) to the register BMP180_REG_CONTROL (0xF4)
        if(!calibrated) {
            calibrated = 0<i2c_smbus_read_i2c_block_data(m_i2c_file, 0xAA, BUFFER_SIZE, cal);
            qDebug() << "BMP190 sensor calibration data read.";
        }

        //cal = read_i2c_block_data(addr, 0xAA, BUFFER_SIZE);
        //# Convert byte data to word values
        int16_t ac1 = get_short(cal, 0);
        int16_t ac2 = get_short(cal, 2);
        int16_t ac3 = get_short(cal, 4);
        uint16_t ac4 = get_ushort(cal, 6);
        uint16_t ac5 = get_ushort(cal, 8);
        uint16_t ac6 = get_ushort(cal, 10);
        int16_t b1 = get_short(cal, 12);
        int16_t b2 = get_short(cal, 14);
        //int16_t mb = get_short(cal, 16); // not used. is =-32768
        int16_t mc = get_short(cal, 18);
        int16_t md = get_short(cal, 20);

        uint8_t msb,lsb,xsb;
        int32_t  b5, x1, x2, t, ut;
        if(!reuse_T) {
            // "Starting temperature conversion..."
            // request data
            i2c_smbus_write_byte_data(m_i2c_file, BMP180_CTRL, BMP180_COMMAND_TEMPERATURE);
            // wait while data is getting ready
            usleep(getWaitingTimeTemperature());

            // read temperature results
            //        (msb, lsb) = bus.read_i2c_block_data(addr, 0xF6, 2)
            msb = i2c_smbus_read_byte_data(m_i2c_file, BMP180_REG_PRE) & 0xFF;
            lsb = i2c_smbus_read_byte_data(m_i2c_file, BMP180_REG_PRE+1) & 0xFF;
            if(msb==0 && lsb==0) { // to avoid exception div/0 if read goes wrong
                setMessage("Error: BMP180 temperature read unsuccessful");
                return;
            }
            ut = (msb << 8) + lsb;
            //"Calculating temperature..."
            x1 = ((ut - ac6) * ac5) >> 15;
            x2 = round((mc << 11) / (x1 + md));
            b5 = (x1 + x2);
            t = (b5 + 8) >> 4;
            T = t/10.0; //# [C]
            m_last_temperature_reading = T;
            if(only_T)
                return;
        }

        //"Starting pressure conversion..."
        // request data
        i2c_smbus_write_byte_data(m_i2c_file, BMP180_CTRL, BMP180_COMMAND_PRESSURE + (oversampling << 6));
        // wait while data is getting ready
        usleep(getWaitingTimePressure(this->_configState.oversampling));
        //(msb, lsb, xsb) = bus.read_i2c_block_data(addr, BMP180_REG_PRE, 3)// from python, works there!
        //i2c_smbus_read_i2c_block_data(m_i2c_file,BMP180_REG_PRE,3,data);
        msb = i2c_smbus_read_byte_data(m_i2c_file, BMP180_REG_PRE) & 0xFF;
        lsb = i2c_smbus_read_byte_data(m_i2c_file, BMP180_REG_PRE+1) & 0xFF;
        xsb = i2c_smbus_read_byte_data(m_i2c_file, BMP180_REG_PRE+2) & 0xFF;

        int32_t up = ((msb << 16) + (lsb << 8) + xsb) >> (8 - oversampling);

        //print "Calculating pressure..."
        if(reuse_T)
            b5 = ( (static_cast<int>(T*10))<<4)-8;
        int32_t b6 = b5 - 4000;
        int32_t b62 = (b6 * b6) >> 12;
        x1 = (b2 * b62) >> 11;
        x2 = (ac2 * b6) >> 11;
        int32_t x3 = x1 + x2;
        int32_t b3 = (((ac1 * 4 + x3) << oversampling) + 2) >> 2;

        x1 = (ac3 * b6) >> 13;
        x2 = (b1 * b62) >> 16;
        x3 = ((x1 + x2) + 2) >> 2;
        int32_t b4 = (ac4 * (x3 + 32768)) >> 15;
        uint32_t b7 = ((unsigned long)up - b3) * (50000 >> oversampling);

        int32_t p;
        if(b7<0x80000000)
            p = (b7 * 2) / b4;
        else
            p = (b7/b4) *2;
        x1 = (p >> 8) * (p >> 8);
        x1 = (x1 * 3038) >> 16;
        x2 = (-7357 * p) >> 16;
        p = p + ((x1 + x2 + 3791) >> 4);

        P = p/ 100.0;// [hPa]
//        qDebug()<<" " <<b5;
//        throw("END");
}

void BerryIMU::readP(float &P, bool reuse_device) {
    //void readTandP(float & T, float & P,bool reuse_device, bool only_T=false, bool reuse_T=false);
    readTandP(m_last_temperature_reading, P , reuse_device, false, true);
}

void BerryIMU::readT(float &T, bool reuse_device) {
     float discard;
     readTandP(T, discard, reuse_device, false );

}

void BerryIMU::readTlsm(float &T)  {
    if(_configState.temperature_sensor_activated) {
        uint8_t buffer[2]; // We'll read two bytes from the temperature sensor into temp
        selectDevice(TP); // Register lies with magnetometer
        if(!readBlock(READ_MULTIPLE_BYTES_FLAG | OUT_TEMP_L_XM, 2, buffer)) {
            setMessage("Error:readTlsm::readBlock returned 0 bytes.");
            return;
        }
        int8_t lo = buffer[0];
        int16_t hi = buffer[1];
        T = static_cast<float>( ((hi<<8) | lo )>> 3 ) + 25.;
        setMessage(std::string("OUT_TEMP_L_XM:")+std::to_string((hi<<8) | lo )+" "+std::to_string((hi<<8) | lo ));
        // T = (float)temp / 8.0 + 25.; // celsius // 25 is undocumented
        //float temperature_f = temperature_c * 1.8 + 32.;
    }
}

bool BerryIMU::setDatarate(mag_odr datarate) {
    _magState.odr   = datarate;
    uint8_t value = readReg(CTRL_REG5_XM);
    // Then mask out the mag ODR bits:
    value &= 0xFF^(0b111 << 2); // &= 0b11100011
    // Then shift in our new ODR bits:
    value |= (datarate << 2);
    // And write the new register value back into CTRL_REG5_XM:
    writeReg(MAG,CTRL_REG5_XM, value);
}

bool BerryIMU::setDatarate(gyro_odr datarate) {
    _gyrState.odr   = datarate;
    uint8_t value = readReg(CTRL_REG1_G);
    value &= 0xFF^(0xF << 4);
    value |= (datarate << 4);
    writeReg(GYR,CTRL_REG1_G, value);
}

bool BerryIMU::setDatarate(acc_odr datarate) { // see mag
    _accState.odr   = datarate;
    uint8_t value = readReg(CTRL_REG1_XM);
    value &= 0xFF^(0xF << 4);
    value |= (datarate << 4);
    writeReg(ACC,CTRL_REG1_XM, value);
}



}
