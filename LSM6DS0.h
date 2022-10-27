/*
 * LSM6DS0.h
 *
 *  Created on: May 17, 2016
 *      Author: justin
 */

#ifndef LSM6DS0_H_
#define LSM6DS0_H_

#include <stdint.h>

extern int8_t LSM6DS0_Init(void);
extern void read(uint8_t* readData, uint8_t addr, uint8_t numBytes);
typedef struct{

    int16_t raw_gx;
    int16_t raw_gy;
    int16_t raw_gz;

    int16_t raw_ax;
    int16_t raw_ay;
    int16_t raw_az;

    int16_t tmp;

}accel_gryo_str;

extern void accelLoad(accel_gryo_str* sample);
extern void gyroLoad(accel_gryo_str* sample);
extern void tmpLoad(accel_gryo_str* sample);
extern void findAccelScale(void);
extern void findGyroScale(void);
extern void configAccelScale(int g);
extern void configGyroScale(int dps);

void displayBitsByte(uint8_t value);


#endif /* LSM6DS0_H_ */
