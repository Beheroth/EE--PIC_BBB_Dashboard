/*
 * File:   I2CS.c
 * Author: Arnaud
 *
 * Created on 14 avril 2018, 15:22
 */

#include <pic18f46k22.h>

#include "I2CS.h"


I2CS_Handler_t *hI2CS_2 = NULL;



void I2CS_Init(I2CS_Handler_t *hI2CS)
{
    //[MCC]
    
    // initialize the hardware
    // R_nW write_noTX; P stopbit_notdetected; S startbit_notdetected; BF RCinprocess_TXcomplete; SMP High Speed; UA dontupdate; CKE disabled; D_nA lastbyte_address; 
    SSP2STAT = 0x00;
    // SSPEN enabled; WCOL no_collision; CKP disabled; SSPM 7 Bit Polling; SSPOV no_overflow; 
    SSP2CON1 = 0x26;
    // ACKEN disabled; GCEN disabled; PEN disabled; ACKDT acknowledge; RSEN disabled; RCEN disabled; ACKSTAT received; SEN disabled; 
    SSP2CON2 = 0x00;
    // SBCDE disabled; BOEN disabled; SCIE disabled; PCIE disabled; DHEN disabled; SDAHT 100ns; AHEN disabled; 
    SSP2CON3 = 0x00;
    // MSK0 127; 
    SSP2MSK = (I2C2_SLAVE_MASK << 1);  // adjust UI mask for R/nW bit            
    // SSPADD 0; 
    SSP2ADD = (I2C2_SLAVE_ADDRESS << 1);  // adjust UI address for R/nW bit

    // clear the slave interrupt flag
    PIR3bits.SSP2IF = 0;
    // enable the master interrupt
    PIE3bits.SSP2IE = 1;    
    
    TRISDbits.RD0=1;    //Setting as input as given in datasheet
    TRISDbits.RD1=1;    //Setting as input as given in datasheet
    
    hI2CS_2 = hI2CS;
    hI2CS->mode = 3;
}


void I2CS_ISR()
{
    PIR3bits.SSP2IF = 0;                     //clearing interruption flag    
    if(hI2CS_2->mode == 0 & hI2CS_2->rx_to_process)     //masking the 7 MSB of the address,asserting READ mode
    //READ
    {
        hI2CS_2->rx_buffer[hI2CS_2->rx_cursor++] = SSP2BUF;
        hI2CS_2->rx_to_process--;
        
        if(hI2CS_2->rx_to_process == 0)     //last byte has been received
        {
            unsigned long reg = hI2CS_2->rx_buffer[0];
            switch(reg)
            {
                case 0x0F:       
                    //temp_read(hI2CS_2);             //[TODO]fix include problems
                    //memory_write();               //[todo]
                    //tx_buffer is filled, waiting for master to pull it
                    break;
            }
            
            hI2CS_2->mode = 3; //reset handler's mode
        }
    }
    
    if(hI2CS_2->mode == 1 & hI2CS_2->tx_to_process)    //WRITE
    {
        SSP2BUF = hI2CS_2->tx_buffer[hI2CS_2->tx_cursor++];
        hI2CS_2->tx_to_process--;
    }
    
    else if(hI2CS_2->mode > 1)            //mode is not defined yet 
    {
        hI2CS_2->mode = SSP2BUF && 0x01;    //masking irrelevant bits 
        if(hI2CS_2->mode == 0)      //configuring for READ mode
        {
            hI2CS_2->rx_to_process = 1;
            hI2CS_2->rx_cursor = 0;
        }
        if(hI2CS_2->mode == 1)      //configuring for WRITE mode
        {
            hI2CS_2->tx_to_process = 2;
            hI2CS_2->tx_cursor = 0;
        }
    }
    
    else
    {
        hI2CS_2->mode = 3;      //reset handler's mode
    }
}