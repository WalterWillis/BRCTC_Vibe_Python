/*
ADIS16460.h
Library for BRCTC Space Team's Gyroscope.
Walter Willis -- Lead Programmer

This code was created for an Arduino system taking part in the 2016 RockSat program.
It has been adapted for us in the 2017 RockSat Program for the Raspberry Pi

In this file are some command definitions and the class for the gyroscope.

RESOURCES:
Steven Hard shared an example file with me called IMU.ino. Most of the commands and logic of this class file were borrowed from this code.

Arduino version 1.6.9

This code is free to use. Distributed as-is; no warranty is given.

*/
#include "stdafx.h"

#ifndef ADIS16460_h
#define ADIS16460_h

//Scale for each data type -- each bit is equal to the following amount
#define GYROSCALE				0.005 // degrees per second
#define ACCSCALE				0.00025 // mg
#define TEMPSCALE				0.05 // may need to add 25 to result after multiplying
#define DELTA_ANGLE_SCALE		0.005 // degrees
#define DELTA_VELOCITY_SCALE	2.5 // mm/sec

#define FLASH_CNT   0x00  //Flash memory write count
#define DIAG_STAT   0x02  //Diagnostic and operational status
#define X_GYRO_LOW  0x04  //X-axis gyroscope output, lower word
#define X_GYRO_OUT  0x06  //X-axis gyroscope output, upper word
#define Y_GYRO_LOW  0x08  //Y-axis gyroscope output, lower word
#define Y_GYRO_OUT  0x0A  //Y-axis gyroscope output, upper word
#define Z_GYRO_LOW  0x0C  //Z-axis gyroscope output, lower word
#define Z_GYRO_OUT  0x0E  //Z-axis gyroscope output, upper word
#define X_ACCL_LOW  0x10  //X-axis accelerometer output, lower word
#define X_ACCL_OUT  0x12  //X-axis accelerometer output, upper word
#define Y_ACCL_LOW  0x14  //Y-axis accelerometer output, lower word
#define Y_ACCL_OUT  0x16  //Y-axis accelerometer output, upper word
#define Z_ACCL_LOW  0x18  //Z-axis accelerometer output, lower word
#define Z_ACCL_OUT  0x1A  //Z-axis accelerometer output, upper word
#define SMPL_CNTR   0x1C  //Sample Counter, MSC_CTRL[3:2]=11
#define TEMP_OUT    0x1E  //Temperature output (internal, not calibrated)
#define X_DELT_ANG  0x24  //X-axis delta angle output
#define Y_DELT_ANG  0x26  //Y-axis delta angle output
#define Z_DELT_ANG  0x28  //Z-axis delta angle output
#define X_DELT_VEL  0x2A  //X-axis delta velocity output
#define Y_DELT_VEL  0x2C  //Y-axis delta velocity output
#define Z_DELT_VEL  0x2E  //Z-axis delta velocity output
#define MSC_CTRL    0x32  //Miscellaneous control
#define SYNC_SCAL   0x34  //Sync input scale control
#define DEC_RATE    0x36  //Decimation rate control
#define FLTR_CTRL   0x38  //Filter control, auto-null record time
#define GLOB_CMD    0x3E  //Global commands
#define XGYRO_OFF   0x40  //X-axis gyroscope bias offset error
#define YGYRO_OFF   0x42  //Y-axis gyroscope bias offset error
#define ZGYRO_OFF   0x44  //Z-axis gyroscope bias offset factor
#define XACCL_OFF   0x46  //X-axis acceleration bias offset factor
#define YACCL_OFF   0x48  //Y-axis acceleration bias offset factor
#define ZACCL_OFF   0x4A  //Z-axis acceleration bias offset factor
#define LOT_ID1     0x52  //Lot identification number
#define LOT_ID2     0x54  //Lot identification number
#define PROD_ID     0x56  //Product identifier
#define SERIAL_NUM  0x58  //Lot-specific serial number
#define CAL_SGNTR   0x60  //Calibration memory signature value
#define CAL_CRC     0x62  //Calibration memory CRC values
#define CODE_SGNTR  0x64  //Code memory signature value
#define CODE_CRC 0x66 //Code memory CRC values
#define BURST_READ 0x3E // Used to read the most common registers with one command

using namespace std;

class ADIS16460
{
	private:
		//WiringPi Pin Scheme
		int _channel = 1; // CE0 on WiringPi
		int _mode = 3; // SPI Mode
		int _speed = 1000000; // frequency of signal
		uint8_t burstdataModel[22] = {BURST_READ, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT, FLASH_CNT};

		int16_t RegRead(uint8_t regAddr);
		int RegWrite(uint8_t regAddr, int16_t regData);
		
	public:
		//int Diagnostic, GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ, Temp, SampleCounter, Checksum; // Perform GetADISReadings function, and then read from the  data.
		//CHECKSUM is the sum off all the preceeding values in unsigned 8 bit number format.		
		const string fileName = "ADIS16460.txt";

		ADIS16460();	
		ADIS16460(int channel, int speed, int mode);	
		int16_t *burstRead(void);
		int16_t checksum(int16_t * burstArray);
		int16_t checksum(int16_t sensorData);
		float accelScale(int16_t sensorData);
		float gyroScale(int16_t sensorData);
		float tempScale(int16_t sensorData);
		float deltaAngleScale(int16_t sensorData);
		float deltaVelocityScale(int16_t sensorData);
		int resetDUT(uint8_t ms);

		//unsigned int readreg(unsigned char nbits, unsigned char reg);
		/*void ClearBuffer(void);
		double twoscomptransform(unsigned char nbits, unsigned int num);
		void GetADISReadings(void);
		unsigned int SetData(unsigned char nbits, unsigned char upper, unsigned char lower);*/		
};

#endif
