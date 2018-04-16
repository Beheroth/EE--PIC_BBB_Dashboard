
#ifndef MAIN_H
#define	MAIN_H

#include "I2CS.h"

void memory_write (I2CS_Handler_t *hI2CS, char mem, char t1, char t2);
char memory_read(I2CS_Handler_t *hI2CS, char mem);
void temp_init(I2CS_Handler_t *hI2CS);
void temp_read(I2CS_Handler_t *hI2CS);
void i2c1_init(void);
void i2c2_Initialize(void);
void i2c1_waitForIdle(void);
void i2c1_start(void);
void i2c1_repStart(void);
void i2c1_stop(void);
int i2c1_read(unsigned char ack);
unsigned char i2c1_write(unsigned char i2cWriteData);



//int ReadTemp();
//void ConfTemp();
//void write_Arduino();
//void i2c_waitForIdle();
//void i2c_start();
//void i2c_repStart();
//void i2c_stop();
//int i2c_read( unsigned char ack );
//unsigned char i2c_write( unsigned char i2cWriteData );
//void memory_write (I2CS_Handler_t *I2CS, char mem, char t1, char t2);


#endif	/* MAIN_H */

