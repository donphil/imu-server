#ifndef I2CDUMMY_H
#define I2CDUMMY_H

#define I2C_SMBUS_BLOCK_MAX 0 //
#define I2C_SLAVE 2

#include <QDebug>
//inline int read_i2c_block_data(int, int ,int *) {return 0;}
inline int ioctl(int, int, int) {return 0;}
inline int i2c_smbus_read_i2c_block_data(int,int,int size,uint8_t* data) {
    int rnd = 0;
    if(size==6) {
        rnd = (500*(rand()-RAND_MAX/2))/RAND_MAX ;

        data[0]= rnd&0xFF;
        data[1]= (rnd&0xFF00)>>8;
        rnd = (500*(rand()-RAND_MAX/2))/RAND_MAX ;
        data[2]= rnd&0xFF;
        data[3]= (rnd&0xFF00)>>8;
        rnd = (500*(rand()-RAND_MAX/2))/RAND_MAX ;
        data[4]= rnd&0xFF;
        data[5]= (rnd&0xFF00)>>8;
    }
    return size;
}
inline int i2c_smbus_read_byte_data(int,int) {return rand();}
inline int i2c_smbus_write_byte_data(int , int , int ) { return 0;}

#endif // I2CDUMMY_H
