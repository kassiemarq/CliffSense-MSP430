
#include <msp430.h>
#include "LSM6DS0_Platform.h"
#include "LSM6DS0.h"
#include <stdio.h>
#include <string.h>
#include "cliffsense.h"

//global vars
float avg_ax;
float avg_ay;
int avg_count;
uint32_t ADC_SoilVal;
uint32_t ADC_BattSense;
uint32_t ADC_5V;
uint32_t ADC_Coil;
accel_gryo_str sample;
char rxbuf[256];
unsigned int counter = 0;

void SPI_init(void);
void RS232_UARTInit(void);
void sendCMD(char *cmd);
void initESPUART();
void initADC1();

extern float accel_scale;


//------------------------------------------------
//------------------------------------------------
union Sample {
   uint16_t b16;
   unsigned char b8[2];
};



int main(void)
{

//    unsigned char NwkSkey[16] = { 0xDE, 0xB1, 0xDA, 0xD2, 0xFA, 0xD3, 0xBA, 0xD4, 0xFA, 0xB5, 0xFE, 0xD6, 0xAB, 0xBA, 0xDB, 0xBA };
//    unsigned char AppSkey[16] = { 0xDE, 0xB1, 0xDA, 0xD2, 0xFA, 0xD3, 0xBA, 0xD4, 0xFA, 0xB5, 0xFE, 0xD6, 0xAB, 0xBA, 0xDB, 0xBA };

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog etimer

    //initialize ports
     SPI_init();
     LSM6DS0_Init(); // initialize sensor
     RS232_UARTInit();
     initESPUART();
     initADC1();
     UCA2IE |= UCRXIE;                         // Enable USCI_A2 RX interrupt
     UCA3IE |= UCRXIE;                         // Enable USCI_A3 RX interrupt

     __enable_interrupt();                   //global enable for maskables



     //P7.3 ESP32 POWER ON
     P7SEL0 &= (~BIT3); // Set P1.0 SEL for GPIO
     P7DIR |= BIT3; // Set P1.0 as Output
     P7OUT &= (~BIT3); // Set LOW

     //P7.4 lora POWER ON
     P7SEL0 &= (~BIT4); // Set P1.0 SEL for GPIO
     P7DIR |= BIT4; // Set P1.0 as Output
     P7OUT &= (~BIT4); // Set LOW

//
  //   configuring Acceleration
     printf("\n");
     configAccelScale(2);
     printf("\n");
     //CHANGE DIVISOR
     printf("DOUBLE CHECK CONFIG:\n");
     findAccelScale();
     printf("\n\n");
     //configuring Gyroscope
     configGyroScale(250);
     printf("\n");
     printf("DOUBLE CHECK CONFIG:\n");
     findGyroScale();


     //LORA ID
     char *cmd =   "AT+DI\n";
     sendCMD(cmd);
//     sendCMD(cmd);

     //LORA INIT
     char *cmd1 = "at+nsk=de.b1.da.d2.fa.d3.ba.d4.fa.b5.fe.d6.ab.ba.db.ba\n";
     char *cmd2 = "at+dsk=de.b1.da.d2.fa.d3.ba.d4.fa.b5.fe.d6.ab.ba.db.ba\n";
     sendCMD(cmd1);

     sendCMD(cmd2);
     char *cmd3 =   "AT+NSK\n";
     sendCMD(cmd3);

     // Network Join Mode: Manual configuration
     char *cmd4 =   "AT+NJM=0\n";
     sendCMD(cmd4);

     char *txn =   "AT+TXN\n";

     float avg_ax[7] = {0};
     float avg_ay[7] = {0};
     avg_count = 0;
     float avg_ax = 0.0;
     float avg_ay = 0.0;

     while(1){

         tmpLoad(&sample);
         accelLoad(&sample);         // get the gyro readings
         gyroLoad(&sample);         // get the gyro readings
         ADC12CTL0 |= ADC12ENC | ADC12SC;
         printf("ADC_Coil:%u\n", ADC_Coil);


         //--------------RAW DATA-----------------

         char buf[64];

         sprintf(buf,"AT+Send=%.2f:%.2f\n",
                 (sample.raw_ax  * accel_scale * GRAVITY_STANDARD) / 1000.0,
                 (sample.raw_ay  * accel_scale * GRAVITY_STANDARD) / 1000.0);
         printf("sending string: %s\n",buf);

         avg_ax[avg_count] = sample.raw_ax  * accel_scale * GRAVITY_STANDARD) / 1000.0;
         avg_ay[avg_count] = sample.raw_ay  * accel_scale * GRAVITY_STANDARD) / 1000.0;

         if(avg_count = 6){
             for(w=0;w<7;w++){
                 avg_ax += avg_ax[w];
                 avg_ay += avg_ay[w];
             }
             avg_ax = avg_ax/7.0;
             avg_ay = avg_ax/7.0;
             printf("%.2f", avg_ax);
             printf("%.2f", avg_ay);


         }//end if


//         AT+TXN Transmit Next
//         Returns the time, in milliseconds, until the next free channel is available to transmit data. The time can range from
//         0-2793000 milliseconds.
         sendCMD(txn);
         sendCMD(buf);
                          if(!(UCA3IFG & UCRXIFG)){
                                   printf("[%s]\n",rxbuf);
                                   if(counter == 127) counter = 0;
                                   counter = 0;
                               }
           //--------------RAW DATA-----------------
      avg_count++;

     }//END WHILE(1)
}//END MAIN

void SPI_init(void){
    //configure USCB1 ports for SPI
    UCB1CTLW0 = UCSWRST; //put USCI-B into software RESET to configure bits for SPI

    UCB1CTLW0 |= UCSSEL_2; //SM CLK in master mode
    UCB1BR0 = 32; //bit rate divide by 32
    //UCB1BR0 = 6;

    UCB1CTLW0 |= UCSYNC; //SPI mode set (synchronous clk)
    UCB1CTLW0 |= UCMST; //set MASTER
    UCB1CTLW0 |= UCMSB; //set MSB 1st rx/tx acc. to BAUER
    UCB1CTLW0 |= UCCKPL;

    //lets get to 4 WIRE MODE w active low --> '10'
    UCB1CTLW0 |= (UCMODE1); //1st bit of UCMODE gets 1
    UCB1CTLW0 &= ~(UCMODE0); //0th bit of UCMODE gets 0
    UCB1CTLW0 |= UCSTEM; //UCSTEM makes STE as output enable for slave


    //CONFIGURING SPI PORTS - then.. set SEL1&SEL0 bits = 01 for ea. mosi, miso, &
    //realize: UCB1 = @P5
    P5SEL1 &= ~(BIT0); //set simo p5.0 - master data out!
    P5SEL0 |= (BIT0);

    P5SEL1 &= ~(BIT1); //set somi p5.1 - master data in!
    P5SEL0 |= (BIT1);

    P5SEL1 &= ~(BIT2); //UCB1 clk p5.2
    P5SEL0 |= (BIT2);


    //SET AS GENERAL PURPOSE OUTPUT instead to manually control
    P5SEL0 &= (~BIT3); // Set P1.0 SEL for GPIO
    P5DIR |= BIT3; // Set P1.0 as Output


    PM5CTL0 &= ~LOCKLPM5; //take out of LOW POWER mode. (clear LPM5 bit)

    //FINALLY, take out of software reset.
     UCB1CTLW0 &= ~(UCSWRST); //clear

}


void RS232_UARTInit(void){

    UCA3CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK, 1Mhz
    UCA3BR0 = 6;                              //integer division only
    UCA3MCTLW |= UCOS16 | UCBRF_8 | 0x2000;   //0xF700 is UCBRSx = 0x20

  //LORA
  // Configure GPIO
  P6SEL1 &= ~(BIT0 | BIT1);                 //bit0-TX & bit1-RX
  P6SEL0 |= (BIT0 | BIT1);                // USCI_A3 UART operation


  PM5CTL0 &= ~LOCKLPM5;                       // turn on I/O
  UCA3CTLW0 &= ~UCSWRST;                      // put UART A3 into SW Reset

}

//---------------

void sendCMD(char *cmd){
    int position;                                // index for the string
    int i, j;                                    // integers for delay loops

    //RS232 UART
    for(position = 0; position < strlen(cmd); position++)
            {
                UCA3TXBUF = cmd[position];
                for(i = 0; i < 1000; i++){}          // delay between characters
            }
            for(j = 0; j < 30000; j++){}            // delay between strings

            if(!(UCA3IFG & UCRXIFG)){
                     printf("[%s]\n",rxbuf);
                     if(counter == 256) counter = 0;
                     counter = 0;
            }

}


//------------
void initESPUART(){

    //CONFIGURE PORTS TO UCA2

    // Configure USCI_A2 for UART mode
       UCA2CTLW0 = UCSWRST;                      // Put eUSCI in reset

       // Baud Rate Setting
       // Use Table 30-5 in Family User Guide

       UCA2CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK, 1Mhz


       UCA2BR0 = 6;                              //integer division only
       UCA2MCTLW |= UCOS16 | UCBRF_8 | 0x2000;   //0xF700 is UCBRSx = 0x20

       UCA2CTLW0 &= ~UCSWRST;                    // Initialize eUSCI


       //set up UART A2
       // Configure GPIO
//       * ESP32 - UART UCA2
//       * P5.4 - UCA2TX to ESP32RX
//       * P5.5 - UCA2RX to ESP32TX
       P5SEL1 &= ~(BIT4 | BIT5);
       P5SEL0 |= (BIT4 | BIT5);                // USCI_A2 UART operation

     // Disable the GPIO power-on default high-impedance mode to activate
      // previously configured port settings
      PM5CTL0 &= ~LOCKLPM5;


}
//---------
void initADC1(){

//    P1OUT &= ~BIT0;                         // Clear LED to start
//    P1DIR |= BIT0;                          // Set P1.0/LED to output (bottom left)


    //GPIO INIT
    //set p1.2 LOW to ENABLE SOIL SENSE
    P1SEL0 &= (~BIT2); // Set P1.2 SEL for GPIO
    P1DIR |= BIT2; // Set P1.2 as Output
    P1OUT &= (~BIT2); // Set P1.2 LOW
    // Configure P3.0 for SOIL SENSE ADC
    P3SEL1 |= BIT0;
    P3SEL0 |= BIT0;

    //set p8.2 high to ENABLE 3.3VBatt ADC
    P8SEL0 &= (~BIT2); // Set P8.2 SEL for GPIO
    P8DIR |= BIT2; // Set P8.2 as Output
    P8OUT |= BIT2; // Set P1.0 HIGH
    // Configure P7.6 for 3.3VBatt ADC
    P7SEL1 |= BIT6;
    P7SEL0 |= BIT6;

    //set p8.3 high to ENABLE COIL_SENSE
    P8SEL0 &= (~BIT3); // Set P8.3 SEL for GPIO
    P8DIR |= BIT3; // Set P8.3 as Output
    P8OUT |= BIT3; // Set P8.3 HIGH
    // Configure P7.7 for analog COIL_SENSE
    P7SEL1 |= BIT7;
    P7SEL0 |= BIT7;

    //set p8.1 high to ENABLE 5VP_BUCK_VOLTAGE_SENSE
    P8SEL0 &= (~BIT1); // Set P8.1 SEL for GPIO
    P8DIR |= BIT1; // Set P8.1 as Output
    P8OUT |= BIT1; // Set P8.1 HIGH
    // Configure P3.3 for 5VP_BUCK_VOLTAGE_SENSE
    P3SEL1 |= BIT3;
    P3SEL0 |= BIT3;


    PM5CTL0 &= ~LOCKLPM5;// Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    // Configure ADC12
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON | ADC12MSC;      // Sampling time, S&H=16, ADC12 on, enable MultipleSampleConversion
    ADC12CTL1 |= ADC12SSEL_3 | ADC12CONSEQ_3;           //choose SM CLK & REPEAT SEQUENCE CHANNEL CONV mode
    ADC12CTL1 = ADC12SHP;                   // Use sampling timer
    ADC12CTL2 |= ADC12RES_2;                // 12-bit conversion results


    //INPUT CHANNEL SELECT
    //ADC12MCTL0 |= ADC12INCH_12;            // A12 ADC input select; Vref = SOIL CONDUCTIVITY PROBE
    //ADC12MCTL0 |= ADC12INCH_15;            // A15 ADC input select; Vref = 5VP BUCK VOLTAGE SENSE
    //ADC12MCTL0 |= ADC12INCH_18;            // A18 ADC input select; Vref = 3.3 BATT SENSE
    ADC12MCTL0 |= ADC12INCH_19;            // A19 ADC input select; Vref = VP COIL SENSE

    ADC12IER0 |= ADC12IE0;                  // Enable ADC conv complete interrupt

}

//-------------
#pragma vector = EUSCI_A3_VECTOR
__interrupt void EUSCI_A3_RX_ISR(void){
    //RECEIVE LORA MESSAGES
    rxbuf[counter++] = UCA3RXBUF;
    rxbuf[counter] = '\0';
}
//--------
#pragma vector = ADC12_B_VECTOR
__interrupt void ADC12_ISR(void){
    //ADC_SoilVal = ADC12MEM0;
    //ADC_BattSense = ADC12MEM0;
    ADC_Coil = ADC12MEM0;

}
//-------



