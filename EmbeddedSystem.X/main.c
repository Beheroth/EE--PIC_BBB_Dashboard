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
    
    // === RUNNING ===
    /*
     * Configure Temp
     * Configure Memory
     * 
     * Counter by 2 (ask by BBB)
     *     - Ask temp
     *     - Write Temp in Mem (place via counter (2 bytes))
     * 
     * BBB ask Values Temp
     *     - Read all Memory up to counter (by 2)
     *     - give it to BBB
     *     - Reset Counter
     * 
     * NOTE: PIC is sleeping and BBB wakes up the PIC to work.
     */
    
    I2CS_Handler_t hI2CS;
    
    I2CS_Init(&hI2CS);
    temp_init(&hI2CS);
    
    //[todo] PIC enters idle mode and waits to be awaken by I2C2 (BBB) 
    //interruption caused by a write request
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

//void i2c2_Initialize(void)
//{
//    // initialize the hardware
//    // R_nW write_noTX; P stopbit_notdetected; S startbit_notdetected; BF RCinprocess_TXcomplete; SMP High Speed; UA dontupdate; CKE disabled; D_nA lastbyte_address; 
//    SSP2STAT = 0x00;
//    // SSPEN enabled; WCOL no_collision; CKP disabled; SSPM 7 Bit Polling; SSPOV no_overflow; 
//    SSP2CON1 = 0x26;
//    // ACKEN disabled; GCEN disabled; PEN disabled; ACKDT acknowledge; RSEN disabled; RCEN disabled; ACKSTAT received; SEN disabled; 
//    SSP2CON2 = 0x00;
//    // SBCDE disabled; BOEN disabled; SCIE disabled; PCIE disabled; DHEN disabled; SDAHT 100ns; AHEN disabled; 
//    SSP2CON3 = 0x00;
//    // MSK0 127; 
//    SSP2MSK = (0x00 << 1);  // adjust UI mask for R/nW bit            
//    // SSPADD 0; 
//    SSP2ADD = (0x7F << 1);  // adjust UI address for R/nW bit
//
//    // clear the slave interrupt flag
//    PIR3bits.SSP2IF = 0;
//    // enable the master interrupt
//    PIE3bits.SSP2IE = 1;
//    
//    TRISDbits.RD0=1;    //Setting as input as given in datasheet
//    TRISDbits.RD1=1;    //Setting as input as given in datasheet    
//
//}

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

//void memory_write (I2CS_Handler_t *hI2CS, char mem, char t1, char t2)
//{
//    hI2CS->slaveAddr = 0x50;     // memory address
//    hI2CS->mode = I2CS_READ;     // mode read
//    
//    hI2CS->tx_buffer[0] = mem+1;    // address high byte
//    hI2CS->tx_buffer[1] = mem;      // address low byte
//    
//    hI2CS->tx_buffer[2] = t1;       // mesure temp 1
//    hI2CS->tx_buffer[3] = t2;       // mesure temp 2
//
//    hI2CS->tx_to_process = 4;
//
//    i2c1_start();
//    
//    i2c1_write( 0xA0 );     //[todo] what address is this?
//    
//    i2c1_write( 0x00 );     //[todo] what instruction is this?
//    i2c1_write( mem );      
//    
//    i2c1_write( value>>8 ); 
//    i2c1_write( value );
//    
//    i2c1_stop();
//
//}
//
//char memory_read(I2CS_Handler_t *hI2CS, char mem)
//{
//    hI2CS->slaveAddr = 0x50;
//    hI2CS->mode = I2CS_READ;
//    
//    hI2CS->tx_buffer[0] = mem+1;
//    hI2CS->tx_buffer[1] = mem;
//    
//    hI2CS->tx_to_process = 2;
//    
//    /* restart */
//    
////    I2CS->mode I2CS_WRITE;
//    
//    char t[2];
//    t[0] = hI2CS->rx_buffer[0];
//    t[1] = hI2CS->rx_buffer[1];
//    
//    hI2CS->rx_to_process = 2;
//    
//    return t;
//    /*
//    i2c1_start();
//    
//    i2c1_write( 0xA0 );
//    
//    i2c1_write( 0x00 );
//    i2c1_write( mem );
//    
//    i2c1_repStart();
//    
//    i2c1_start( 0xA1 );
//    
//    unsigned char temp = i2c1_read(0);
//    unsigned char temp1 = i2c1_read(1);
//    
//    i2c1_stop();
//    
//    int t = (int) (temp<<8)+temp1;
//    
//    return t;
//    */
//}

void temp_init(I2CS_Handler_t *hI2CS)
{
    //[todo] initializing temp sensor chould not modify hI2CS handler
    hI2CS->slaveAddr = 0x48;     // probe address
    
    hI2CS->tx_buffer[0] = 0xEE;     // message to send, settings to start temperature sampling
    hI2CS->tx_to_process = 1;
}

void temp_read(I2CS_Handler_t *hI2CS)
{
    
//    I2CS->slaveAddr = 0x48;     //  probe address
//    I2CS->mode = I2CS_READ;     // mode read
//    I2CS->tx_buffer[0] = 0xAA;     // probe write 2 bytes for one temperature measurement
//    I2CS->tx_to_process = 1;
  

    i2c1_start();
    
	i2c1_write( 0x90 );     //probe addres + R
	i2c1_write( 0xAA );     //probe writes 2 bytes for one temperature measurment
    
	i2c1_repStart();
    
    i2c1_write( 0x91 );     //probe addres + W 
    
	hI2CS->tx_buffer[0] = i2c1_read(0);
	hI2CS->tx_buffer[1] = i2c1_read(1);
    
	i2c1_stop();
    
//    int t = (int) (temp<<8) | temp1;        //concatenates 2 bytes
//       
//    return t;

}

/*
void write_msg( unsigned char address, unsigned char msg )
{
    i2c_start();
    i2c_write( address <<1 );
    i2c_write( msg );
    i2c_stop();
}
*/

/******************************************************************************************/

/**
 End of File
*/
