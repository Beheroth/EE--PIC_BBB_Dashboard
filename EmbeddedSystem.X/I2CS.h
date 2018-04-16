

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */
 
#ifndef I2CS_H
#define	I2CS_H

#include "stdio.h"

#define I2C2_SLAVE_ADDRESS 0x00 
#define I2C2_SLAVE_MASK    0x7F

#define I2C_BUFFER_LENGHT       10
#define I2CS_WRITE               0x01
#define I2CS_READ                0x00

typedef struct{
    unsigned long tx_buffer[I2C_BUFFER_LENGHT];
    unsigned long rx_buffer[I2C_BUFFER_LENGHT];
    unsigned long mode;         //the mode should only be managed by I2CS.c
    unsigned char tx_to_process;  
    unsigned char rx_to_process;
    unsigned char tx_cursor;    
    unsigned char rx_cursor;
    unsigned char slaveAddr;
}I2CS_Handler_t;

void I2CS_Init(I2CS_Handler_t *hI2C);
void I2CS_WriteByte(I2CS_Handler_t *hI2C, unsigned char slaveAddr, unsigned char reg, unsigned char data) ;
void I2CS_ReadByte(I2CS_Handler_t *hI2C, unsigned char slaveAddr, unsigned char reg);
void I2CS_Start(I2CS_Handler_t *hI2C);

#endif	/* I2CS_H */

