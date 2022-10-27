/*
 * LSM6DS0.c
 *
 *  Created on: May 17, 2016
 *      Author: justin
 *
 * Inspiration:
 * src: https://developer.mbed.org/teams/ST-Americas-mbed-Team/code/Nucleo_Sensor_Shield/docs/tip/
 * src: https://github.com/ChristopherJD/STM32L053R8/blob/master/Intern_Project/LSM6DS0.c
 * src: https://eewiki.net/pages/viewpage.action?pageId=47644832
 */



#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <limits.h>
#include "LSM6DS0.h"
#include "LSM6DS0_Platform.h"

//global vars

float accel_scale;
float gyro_scale;

#define SELECT()  do { P5OUT &= ~(BIT3); __delay_cycles(1000); } while ( 0 )
#define DESELECT()  do { P5OUT |= (BIT3); __delay_cycles(1000); } while ( 0 )


static uint8_t readByte();
static void setReadMode(uint8_t addr);
static void setWriteMode(uint8_t addr);
static void writeByte(uint8_t data);
static void writeRegister(uint8_t addr, uint8_t data);



/**
 * Initializes the transducer to output gyro and accelerometer data at 238Hz
 */
extern int8_t LSM6DS0_Init(){
    printf("\n\nLSM6DS0 Init beginning...\n");
    uint8_t deviceId;

    read(&deviceId, LSM6DS0_XG_WHO_AM_I_ADDR, 1); // First detect if module is attached - WHO_AM_I SHOULD=0x0F


    printf("READING done.\n");

    if(deviceId != I_AM_LSM6DS0_XG){
        //return -1; //error
        printf("ERROR: LSM6DS0 not found!\n");
        printf("device ID is: %d", deviceId);
    }





    else{
        printf("SUCCESS!! LSM6DS0 found!\n");
    }

    writeRegister(CTRL2_G, LSM6DS0_G_ODR_238HZ); // enable gyro  at 238 Sampling rate (~4-8mA runtime)
    writeRegister(CTRL1_XL, LSM6DS0_G_ODR_238HZ); // enable accel at 238 Sampling rate (~4-8mA runtime)
    writeRegister(LSM6DS0_CTRL_REG3_G, HP_EN | 0x00); // enable high-pass filter for gyro
    writeRegister(LSM6DS0_FIFO_CTR, 0x00); // Bypass mode, turn of FIFO

    return 1; // success
}

////////////////////////////////////////////////////////////////
//The LSM6DS0 has all of its registers in 2's complement
////////////////////////////////////////////////////////////////

/**
 * Get all 3-axis accelerometer readings. Convert from mg to g.
 */
extern void accelLoad(accel_gryo_str* sample){


#define GRAVITY_STANDARD 9.80665

    printf("\n\n");
   //printf("note accel_scale is: %.3f\n", accel_scale);

    //accel X plane----------------
    int16_t ax;
    uint8_t aLx = 0x00;
    uint8_t aHx = 0x00;
    float ax_f = 0.0;

  //  printf("Reading X plane..\n");
    //read LOW gyro reg
    read((uint8_t*)&aLx,LSM6DS0_XG_OUT_X_L_XL, 1);
//    printf("aLx: \n");
//    displayBits(aLx);
    //read HIGH gyro reg
    read((uint8_t*)&aHx,LSM6DS0_XG_OUT_X_H_XL, 1);
//    printf("aHx: \n");
//    displayBits(aHx);
    //LOW and HIGH together
    ax = aHx;
    ax <<= 8;
    ax |= aLx;
//    printf("raw ax:\n");
//    displayBits(ax);
//    printf("ax int: %d\n",ax);
    //to construct
    sample->raw_ax = ax;

//    printf("gx int: %d\n",ax);

    ax_f = (ax  * accel_scale * GRAVITY_STANDARD) / 1000.0; //conversion to FP 16b --> 4B
    printf("Accel X: %.4f m/s^2\n",ax_f);
 //   printf("\n");

    //x_gyro->gyro_z = ax_f;


    //accel Y plane----------------
    int16_t ay;
    uint8_t aLy = 0x00;
    uint8_t aHy = 0x00;
    float ay_f = 0.0;

  //  printf("Reading Y plane..\n");
    //read LOW gyro reg
    read((uint8_t*)&aLy,LSM6DS0_XG_OUT_Y_L_XL, 1);
//    printf("aLy: \n");
//    displayBits(aLy);
    //read HIGH gyro reg
    read((uint8_t*)&aHy,LSM6DS0_XG_OUT_Y_H_XL, 1);
//    printf("aHy: \n");
//    displayBits(aHy);
    //LOW and HIGH together
    ay = aHy;
    ay <<= 8;
    ay |= aLy;
//    printf("raw ay:\n");
//    displayBits(ay);
//    printf("ay int: %d\n",ay);
    //to construct
    sample->raw_ay = ay;

//    printf("gy int: %d\n",ay);
    ay_f = (ay  * accel_scale * GRAVITY_STANDARD) / 1000.0; //conversion to FP 16b --> 4B
    printf("Accel Y: %.4f m/s^2\n",ay_f);
 //   printf("\n");

    //accel Z plane----------------
    int16_t az;
    uint8_t aLz = 0x00;
    uint8_t aHz = 0x00;
    float az_f = 0.0;

   // printf("Reading Z plane..\n");
    //read LOW gyro reg
    read((uint8_t*)&aLz,LSM6DS0_XG_OUT_Y_L_XL, 1);
//    printf("aLz: \n");
//    displayBits(aLz);
    //read HIGH gyro reg
    read((uint8_t*)&aHz,LSM6DS0_XG_OUT_Y_H_XL, 1);
//    printf("aHz: \n");
//    displayBits(aHz);
    //LOW and HIGH together
    az = aHz;
    az <<= 8;
    az |= aLz;
//    printf("raw az:\n");
//    displayBits(az);

    //to construct
    sample->raw_az = az;

//    printf("az int: %d\n",az);
    az_f = (az  * accel_scale* GRAVITY_STANDARD) / 1000.0; //conversion to FP 16b --> 4B
    printf("Accel Z: %.4f m/s^2\n",az_f);
 //   printf("\n");


}

/**
 * Get all 3-axis gyro readings. Convert from mdps to dps.
 */

// display bits of an unsigned int value
void displayBits(unsigned short value)
{
    unsigned int c;
   unsigned int displayMask = 1 << ((CHAR_BIT * sizeof(unsigned short)) - 1);

   printf("%7u = ", value);

   // loop through bits
   for (c = 1; c <= (CHAR_BIT * sizeof(unsigned short)); ++c) {
      putchar(value & displayMask ? '1' : '0');
      value <<= 1; // shift value left by 1

      if (c % CHAR_BIT == 0) { // output a space after 8 bits
         putchar(' ');
      }
   }

   putchar('\n');
}

void displayBitsByte(uint8_t value)
{
    unsigned int c;
   unsigned int displayMask = 1 << ((CHAR_BIT * sizeof(uint8_t)) - 1);

   printf("%7u = ", value);

   // loop through bits
   for (c = 1; c <= (CHAR_BIT * sizeof(uint8_t)); ++c) {
      putchar(value & displayMask ? '1' : '0');
      value <<= 1; // shift value left by 1

      if (c % CHAR_BIT == 0) { // output a space after 8 bits
         putchar(' ');
      }
   }

   putchar('\n');
}

extern void gyroLoad(accel_gryo_str* sample){
#define SENSORS_DPS_TO_RADS 0.017453293

//printf("note gyro_scale is: %.3f\n", gyro_scale);
    int16_t gx;
    uint8_t gLx = 0x00;
    uint8_t gHx = 0x00;
    float gx_f = 0.0;

    int16_t gy;
    uint8_t gLy = 0x00;
    uint8_t gHy = 0x00;
    float gy_f = 0.0;

    int16_t gz;
    uint8_t gLz = 0x00;
    uint8_t gHz = 0x00;
    float gz_f = 0.0;

 //   printf("Reading X plane..\n");
    //read LOW gyro reg
    read((uint8_t*)&gLx,LSM6DS0_XG_OUT_X_L_G, 1);
    //read HIGH gyro reg
    read((uint8_t*)&gHx,LSM6DS0_XG_OUT_X_H_G, 1);
//    printf("gLx: \n");
//    displayBits(gLx);
//    printf("gHx: \n");
//    displayBits(gHx);
    //LOW and HIGH together
    gx = gHx;
    gx <<= 8;
    gx |= gLx;
//    printf("gx raw:\n");
//    displayBits(gx);
//    printf("gx int: %d\n",gx);

    //to construct
    sample->raw_gx = gx;

    //conversion
    //printf("gyro_scale is %.2f\n", gyro_scale);


    gx_f = (gx * gyro_scale * SENSORS_DPS_TO_RADS ) / 1000.0; //conversion to FP 16b --> 4B
    printf("Gyro X: %.2f rad/s\n",gx_f);




  //  printf("Reading Y plane..\n");
    read((uint8_t*)&gLy,LSM6DS0_XG_OUT_Y_L_G, 1);
    read((uint8_t*)&gHy,LSM6DS0_XG_OUT_Y_H_G, 1);
//    printf("gLy: \n");
//    displayBits(gLy);
//    printf("gHy: \n");
//    displayBits(gHy);
    //LOW and HIGH together
    gy = gHy;
    gy <<= 8;
    gy |= gLy;

//    printf("gy raw:\n");
//    displayBits(gy);
//    printf("gy int: %d\n",gy);
    //to construct
    sample->raw_gy = gy;
    //conversion
    gy_f = (gy * gyro_scale * SENSORS_DPS_TO_RADS) / 1000.0; //conversion to FP 16b --> 4B
    printf("Gyro Y: %.2f rad/s\n",gy_f);



  //  printf("Reading Z plane..\n");
    read((uint8_t*)&gLz,LSM6DS0_XG_OUT_Z_L_G, 1);
    read((uint8_t*)&gHz,LSM6DS0_XG_OUT_Z_H_G, 1);
//    printf("gLz: \n");
//    displayBits(gLz);
//    printf("gHz: \n");
//    displayBits(gHz);
    //LOW and HIGH together
    gz = gHz;
    gz <<= 8;
    gz |= gLz;
//    printf("gz raw:\n");
//    displayBits(gz);
//    printf("gz int: %d\n",gz);
    //to construct
    sample->raw_gz = gz;
    //conversion
    gz_f = (gz * gyro_scale * SENSORS_DPS_TO_RADS) / 1000.0; //conversion to FP 16b --> 4B
    printf("Gyro Z: %.2f rad/s\n",gz_f);


}

extern void tmpLoad(accel_gryo_str* sample){
    int16_t temp = 0x0000;
    uint8_t tmpL = 0x00;
    uint8_t tmpH = 0x00;
    float tempf = 0.0;

    read((uint8_t*)&tmpL,OUT_TEMP_L , 1);

//    printf("tmpL is: \n");
//    displayBits(tmpL);

    read((uint8_t*)&tmpH,OUT_TEMP_H , 1);

//    printf("tmpH is: \n");
//    displayBits(tmpH);

    //RAW BEFORE CONVERSION
    temp = (tmpH<<8)|tmpL;
//    printf("tmp raw:\n");

//    displayBits(temp);

//    printf("temp 2's: %d\n",temp);
    printf("Temperature C: %.2f\n",temp/256.0 + 25);

    sample->tmp = temp;

}


/**
 * Reads any number of bytes from a specific address. Make sure that
 * the sensor will self-increment its register pointer after subsequent reads
 */
extern void read(uint8_t* readData, uint8_t addr, uint8_t numBytes){
    uint8_t i;
    //SELECT();
    P5OUT &= ~(BIT3);
    __delay_cycles(1000);
    setReadMode(addr);
    for(i = 0; i != numBytes; i++){
        *readData = readByte();         //**address of deviceID gets RxData
        readData++; // increase to next address
    }

    P5OUT |= (BIT3); __delay_cycles(1000);
}//end read

/**
 * Writes a single byte to a single register
 */

static void writeRegister(uint8_t addr, uint8_t data){
    //SELECT();
    P5OUT &= ~(BIT3);

    __delay_cycles(1000);
    setWriteMode(addr);
    writeByte(data);
    __delay_cycles(1000);
    //DESELECT();
    P5OUT |= (BIT3);
}

/**
 * Write an initial byte
 */
static void setWriteMode(uint8_t addr){
    __delay_cycles(1000);
    while (!(UCB1IFG&UCTXIFG)); // TX buffer ready?
    UCB1TXBUF = (addr | 0x00); // 1 = read / 0 = write
    while (!(UCB1IFG&UCTXIFG)); //  TX buffer ready?
    __delay_cycles(1000);
}

/**
 * Write a single byte
 */
static void writeByte(uint8_t data){
    __delay_cycles(1000);
    while (!(UCB1IFG&UCTXIFG)); // TX buffer ready?
    UCB1TXBUF = data; // send data
    while (!(UCB1IFG&UCTXIFG)); //  TX buffer ready?
    __delay_cycles(1000);
}

/**
 * Read a single byte
 */
static void setReadMode(uint8_t addr){
    while (!(UCB1IFG&UCTXIFG)); // TX buffer ready? - USCI_B1 TX buffer ready?

    UCB1TXBUF = (addr | 0x80); // 1 = read / 0 = write - **send LSM6DS0_XG_WHO_AM_I_ADDR | 0x80
    /*
     *msb first - 1 = read, followed by 7bit data = 0x0F.
     */

    while (!(UCB1IFG&UCTXIFG)); //  TX buffer ready?
    //printf("Tx buffer is ready!x2\n");
}


static uint8_t readByte(){
    __delay_cycles(1000);
    while (!(UCB1IFG&UCTXIFG)); // TX buffer ready?
    UCB1TXBUF = 0xAA; // send garbage
    while (!(UCB1IFG&UCTXIFG)); //  TX buffer ready?
    while (!(UCB1IFG&UCRXIFG)); //  RX Received?
    __delay_cycles(1000);

    return UCB1RXBUF; // Store received data
}

extern void configAccelScale(int g){

    //note: CTRL1_XL is accel ctrl register. read & write
    uint8_t a_scale = 0;
    //get original value
    read((uint8_t*)&a_scale, CTRL1_XL, 1);
    //configure

    printf("configuring to NEW scale...\n");

    if(g == 4){
        printf("configure to 4g...\n");
        //clear out LSB 4 bits to modify. do not modify MSB.
        a_scale= a_scale & 0b11110000;
        //modify for 4g
        a_scale|=0x08;
        //now write to register to actually modify scale to CTRL1_XL
        writeRegister(CTRL1_XL, a_scale);
        read((uint8_t*)&a_scale, CTRL1_XL, 1);
        displayBitsByte(a_scale);
        printf("\n");
    }
    else if(g==8){
        printf("configure to 8g...\n");
        a_scale&=0b11110000;
        a_scale|=0x0C;
        writeRegister(CTRL1_XL, a_scale);
        read((uint8_t*)&a_scale, CTRL1_XL, 1);
        displayBitsByte(a_scale);
        printf("\n");
    }
    else if(g==16){
        printf("configure to 16g...\n");
        a_scale&=0b11110000;
        a_scale|=0x04;
        writeRegister(CTRL1_XL, a_scale);
        read((uint8_t*)&a_scale, CTRL1_XL, 1);
        displayBitsByte(a_scale);
        printf("\n");
    }
    else{
        //assume case 2g
        printf("configure to 2g...\n");
        a_scale &= 0b11110011;
        writeRegister(CTRL1_XL, a_scale);
        read((uint8_t*)&a_scale, CTRL1_XL, 1);
        displayBitsByte(a_scale);
        printf("\n");
    }

}


extern void findAccelScale(void){

    //find scale and orientation range FIRST**
    //note: CTRL1_XL is accel ctrl register. read & write
    //THIS FUNCTION DETERMINES THE DIVISOR within accelLoad. divisor passed t global var -> float 'accel_scale'

//#define XL_2g = 0x00
//#define XL_4g = 0x08
//#define XL_8g = 0x0C //0000-1100
//#define XL_16g = 0x04


    uint8_t mask_a_scale = 0;
    uint8_t a_scale = 0;

    read((uint8_t*)&a_scale, CTRL1_XL, 1);
    printf("Accel Reg - CTRL1_XL is: \n");
    displayBitsByte(a_scale);

    //construct bit mask to find scale
    mask_a_scale = a_scale & 0b00001100;

    //find case
    if(mask_a_scale == 0x0C){
        printf("ACCEL SCALE: 8g \n");
        accel_scale = 0.244;
    }
    else if(mask_a_scale == 0b00000100){
        printf("ACCEL SCALE: 16g \n");
        accel_scale = 0.488;
    }
    else if(mask_a_scale == 0b00001000){
        printf("ACCEL SCALE: 4g \n");
        accel_scale = 0.122;
    }
    else{
        //case '00'
        printf("ACCEL SCALE: 2g \n");
        accel_scale = 0.061;
    }

}


extern void findGyroScale(void){
        //gyro scale
        uint8_t g_scale = 0;
        uint8_t mask_g_scale = 0;
        read((uint8_t*)&g_scale, CTRL2_G, 1);
        printf("Gyro Ctrl Reg - ORIGINAL VALUE CTRL2_G is: \n");
        displayBits(g_scale);

        //construct bit mask
        mask_g_scale = g_scale & 0b00001100;
        if(mask_g_scale == 0b00000000){
            printf("gyro UI chain full scale-sel: 250dps \n");
            gyro_scale = 8.75;

        }
        else if(mask_g_scale == 0b00000100){
            printf("gyro UI chain full scale-sel: 500dps \n");
            gyro_scale = 17.50;

        }
        else if(mask_g_scale == 0b00001000){
            printf("gyro UI chain full scale-sel: 1000dps \n");
            gyro_scale = 35.0;

        }
        else{
            //case '11'
            printf("gyro UI chain full scale-sel: 2000dps \n");
            gyro_scale = 70.0;

        }
}

extern void configGyroScale(int dps){

    //note: CTRL2_G is gyro ctrl register. read & write
     uint8_t g_scale = 0;
     //get original value
     read((uint8_t*)&g_scale, CTRL2_G, 1);
     printf("\nCONFIGGYROSCALE(), original GYRO reg value is:\n");
     displayBitsByte(g_scale);

     if(dps == 500){
         printf("configure to 500dps...\n");
         //clear out LSB 4 bits to modify. do not modify MSB.
         g_scale= g_scale & 0b11110000;
         g_scale|=0x04; //modify for 500dps '01'
         //now write to register to actually modify scale to CTRL2_G
         writeRegister(CTRL2_G, g_scale);
         read((uint8_t*)&g_scale, CTRL2_G, 1);
         displayBitsByte(g_scale);
         printf("\n");
     }
     else if(dps==1000){
         printf("configure to 1000dps...\n");
         //clear out LSB 4 bits to modify. do not modify MSB.
         g_scale= g_scale & 0b11110000;
         g_scale|=0x08; //modify for 1000dps '10'
         //now write to register to actually modify scale to CTRL2_G
         writeRegister(CTRL2_G, g_scale);
         read((uint8_t*)&g_scale, CTRL2_G, 1);
         displayBitsByte(g_scale);
         printf("\n");
     }
     else if(dps==2000){
         //modify for 2000dps '11'
         printf("configure to 2000dps...\n");
         //clear out LSB 4 bits to modify. do not modify MSB.
         g_scale= g_scale & 0b11110000;
         g_scale|=0x0C; //modify for 2000dps '11'
         //now write to register to actually modify scale to CTRL2_G
         writeRegister(CTRL2_G, g_scale);
         read((uint8_t*)&g_scale, CTRL2_G, 1);
         displayBitsByte(g_scale);
         printf("\n");
     }
     else{
         //assume 250 dps
         printf("configure to 250dps...\n");
         //clear out LSB 4 bits to modify. do not modify MSB.
         g_scale= g_scale & 0b11110000;//modify for 250dps '00'
         //now write to register to actually modify scale to CTRL2_G
         writeRegister(CTRL2_G, g_scale);
         read((uint8_t*)&g_scale, CTRL2_G, 1);
         displayBitsByte(g_scale);
         printf("\n");
     }

}

