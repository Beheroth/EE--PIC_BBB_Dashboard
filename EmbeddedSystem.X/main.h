
#ifndef MAIN_H
#define	MAIN_H

#include "I2CS.h"

int ReadTemp();
void ConfTemp();
void write_Arduino();
void i2c_init();
void i2c_waitForIdle();
void i2c_start();
void i2c_repStart();
void i2c_stop();
int i2c_read( unsigned char ack );
unsigned char i2c_write( unsigned char i2cWriteData );


#endif	/* MAIN_H */

