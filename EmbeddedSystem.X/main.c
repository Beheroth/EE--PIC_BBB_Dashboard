/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65
        Device            :  PIC18F46K22
        Driver Version    :  2.00
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "mcc_generated_files/mcc.h"
#include "main.h"


#define TEMP = 0x48     //[todo]what does it define exaclty?

/*
                         Main application
 */


void main(void)
{
    
    // Initialize the device
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    i2c1_init();
    
    temp_init();
    
    int counter = 0;
    
    OSCCONbits.IDLEN = 0;
    
     while(1)  
    {      
        SLEEP();
    }
}



void i2c1_init(void)
{
    TRISCbits.RC3 =1;            // set SCL pin as input
    TRISCbits.RC4 =1;            // set SDA pin as input

    SSP1CON1 = 0x38;      // set I2C master mode
    SSP1CON2 = 0x00;

    // 400kHz bus with 10MHz xtal - use 0x0C with 20MHz xtal
    SSP1ADD = 10;            // 100k at 4Mhz clock

    SSP1STATbits.CKE =0;     // use I2C levels      worked also with '0'
    SSP1STATbits.SMP =0;     // disable slew rate control  worked also with '0'

    PIR1bits.SSP1IF=0;      // clear SSPIF interrupt flag
    PIR2bits.BCL1IF=0;      // clear bus collision flag
    
}

/******************************************************************************************/

void i2c1_waitForIdle(void)
{
    while (( SSP1CON2 & 0x1F ) | SSP1STATbits.RW ) {}; // wait for idle and not writing
    //Concerned SSP1CON bits are, Acknowledge sequence, Receive, Stop condition, Repeated Start condition, Start condition
}

/******************************************************************************************/

void i2c1_start(void)
{
    i2c1_waitForIdle();
    SSP1CON2bits.SEN=1;
}

/******************************************************************************************/

void i2c1_repStart(void)
{
    i2c1_waitForIdle();
    SSP1CON2bits.RSEN=1;
}

/******************************************************************************************/

void i2c1_stop(void)
{
    i2c1_waitForIdle();
    SSP1CON2bits.PEN=1;
}

/******************************************************************************************/

int i2c1_read( unsigned char ack )
{
    unsigned char i2cReadData;

    i2c1_waitForIdle();

    SSP1CON2bits.RCEN=1;

    i2c1_waitForIdle();

    i2cReadData = SSP1BUF;

    i2c1_waitForIdle();

    if ( ack )
    {
        SSP1CON2bits.ACKDT=0;
    }
    else
    {
        SSP1CON2bits.ACKDT=1;
    }
        SSP1CON2bits.ACKEN=1;               // send acknowledge sequence

    return( i2cReadData );
}

/******************************************************************************************/

unsigned char i2c1_write( unsigned char i2cWriteData )
{
    i2c1_waitForIdle();
    SSP1BUF = i2cWriteData;

    return ( ! SSP1CON2bits.ACKSTAT  ); // function returns '1' if transmission is acknowledged
}

/******************************************************************************************/

void memory_write (char mem, char *temp)
{
    i2c1_start();
    
    i2c1_write( 0xA0 );
    
    i2c1_write( mem );
    i2c1_write( ++mem );
    
    i2c1_write( temp[0] );
    i2c1_write( temp[1] );
    
    i2c1_stop();
}

char memory_read(char *temp, char *mem)
{
    i2c1_start();
    
    i2c1_write( 0xA0 );
    
    i2c1_write( 0x00 );
    i2c1_write( mem );
    
    i2c1_repStart();
    
    i2c1_start( 0xA1 );
    
    unsigned char temp = i2c1_read(0);
    unsigned char temp1 = i2c1_read(1);
    
    i2c1_stop();
    
    int t = (int) (temp<<8)+temp1;
    
    return t;
}

void temp_init()
{
    i2c1_start();
    
    i2c1_write( 0x90 );
    i2c1_write( 0xEE );
    
    i2c1_stop();
}

char temp_read()
{

    i2c1_start();
    
	i2c1_write( 0x90 );     //probe addres + R
	i2c1_write( 0xAA );     //probe writes 2 bytes for one temperature measurment
    
	i2c1_repStart();
    
    i2c1_write( 0x91 );     //probe addres + W 
    
    char temp[2];
	char temp[0] = i2c1_read(0);
	char temp[1] = i2c1_read(1);
    
	i2c1_stop();
       
    return temp;
}

/******************************************************************************************/

/**
 End of File
*/
