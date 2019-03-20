/*
ADIS16460.cpp
Library for BRCTC Space Team's Gyroscope.
Walter Willis -- Lead Programmer

Slightly modified from publicly available library

NOTES: using WiringPi delayMicroseconds() with less than 100 microseconds can cause hangs when using multiple threads

*/
//DISCLAIMER
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  September 2016
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16460.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This library provides all the functions necessary to interface the ADIS16460 IMU with a 
//  PJRC 32-Bit Teensy 3.2 Development Board. Functions for SPI configuration, reads and writes,
//  and scaling are included. This library may be used for the entire ADIS1646X family of devices 
//  with some modification.
//
//  Permission is hereby granted, free of charge, to any person obtaining
//  a copy of this software and associated documentation files (the
//  "Software"), to deal in the Software without restriction, including
//  without limitation the rights to use, copy, modify, merge, publish,
//  distribute, sublicense, and/or sell copies of the Software, and to
//  permit persons to whom the Software is furnished to do so, subject to
//  the following conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
//#include "wiringPi.h"
#include <wiringPiSPI.h>
#include "ADIS16460.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>


ADIS16460::ADIS16460()
{
	(wiringPiSPISetupMode (_channel, _speed, _mode) < 0);
}

ADIS16460::ADIS16460(int channel, int speed, int mode)
{
	_channel = channel;
	_speed = speed;
	_mode = mode;
	(wiringPiSPISetupMode(channel, speed, mode) < 0);
}

////////////////////////////////////////////////////////////////////////////
// Performs a hardware reset by setting _RST pin low for delay (in ms).
////////////////////////////////////////////////////////////////////////////
//int ADIS16460::resetDUT(uint8_t ms) {
//	digitalWrite(_RST, LOW);
//	delay(100);
//	digitalWrite(_RST, HIGH);
//	delay(ms);
//	return(1);
//}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////////////////
int16_t ADIS16460::RegRead(uint8_t regAddr)
{
	unsigned char Data[] = { regAddr, 0x00 };
	wiringPiSPIDataRW(_channel, Data, 2);

	delayMicroseconds(40);

	int16_t dataOut = (Data[0] << 8) | (Data[1] & 0xFF);

	return dataOut;
}

////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
int ADIS16460::RegWrite(uint8_t regAddr, int16_t regData) 
{
	// Write register address and data
	uint16_t addr = (((regAddr & 0x7F) | 0x80) << 8);
	uint16_t lowWord = (addr | (regData & 0xFF));
	uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF));

	//Split words into chars and place into char array
	unsigned char Data[4];
	Data[0] = (highWord >> 8);
	Data[1] = (highWord & 0xFF);
	Data[2] = (lowWord >> 8);
	Data[3] = (lowWord & 0xFF);

	//Write to SPI bus
	wiringPiSPIDataRW(_channel, Data, 4);

	return 1;
}

////////////////////////////////////////////////////////////////////////////
// Intiates a burst read from the sensor.
// Returns a pointer to an array of sensor data. 
////////////////////////////////////////////////////////////////////////////
int16_t *ADIS16460::burstRead(void) {

	const int16_t length = 22;
	uint8_t burstdata[length];
	memcpy(burstdata, burstdataModel, length*sizeof(uint8_t));
	static int16_t burstwords[length/2];

	// Trigger Burst Read
	wiringPiSPIDataRW(_channel, burstdata, length);

	// Join bytes into words
	burstwords[0] = ((burstdata[2] << 8) | (burstdata[3] & 0xFF)); 
	burstwords[1] = ((burstdata[4] << 8) | (burstdata[5] & 0xFF)); 
	burstwords[2] = ((burstdata[6] << 8) | (burstdata[7] & 0xFF)); 
	burstwords[3] = ((burstdata[8] << 8) | (burstdata[9] & 0xFF)); 
	burstwords[4] = ((burstdata[10] << 8) | (burstdata[11] & 0xFF)); 
	burstwords[5] = ((burstdata[12] << 8) | (burstdata[13] & 0xFF)); 
	burstwords[6] = ((burstdata[14] << 8) | (burstdata[15] & 0xFF)); 
	burstwords[7] = ((burstdata[16] << 8) | (burstdata[17] & 0xFF)); 
	burstwords[8] = ((burstdata[18] << 8) | (burstdata[19] & 0xFF)); 
	burstwords[9] = ((burstdata[20] << 8) | (burstdata[21] & 0xFF)); 
	
	//Data order
	//DIAG_STAT//XGYRO//YGYRO//ZGYRO//XACCEL//YACCEL//ZACCEL//TEMP_OUT//SMPL_CNTR//CHECKSUM
	return burstwords;
}

////////////////////////////////////////////////////////////////////////////
// Calculates checksum based on burst data.
// Returns the calculated checksum.
////////////////////////////////////////////////////////////////////////////
// *burstArray - array of burst data
// return - (int16_t) signed calculated checksum
////////////////////////////////////////////////////////////////////////////
int16_t ADIS16460::checksum(int16_t * burstArray) {
	int16_t s = 0;
	for (int i = 0; i < 9; i++) // Checksum value | burstArray[9] | is not part of the sum!!
	{
		s += (burstArray[i] & 0xFF); // Count lower byte
		s += ((burstArray[i] >> 8) & 0xFF); // Count upper byte
	}

	return s;
}

int16_t ADIS16460::checksum(int16_t sensorData) {
	int16_t s = 0;

	s += (sensorData & 0xFF); // Count lower byte
	s += ((sensorData >> 8) & 0xFF); // Count upper byte

	return s;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Converts accelerometer data output from the regRead() function and returns
// acceleration in mg's
/////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled accelerometer in g's
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::accelScale(int16_t sensorData)
{
	float finalData = sensorData * 0.00025; // Multiply by accel sensitivity (25 mg/LSB)
	return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts gyro data output from the regRead() function and returns gyro rate in deg/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled gyro in degrees/sec
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::gyroScale(int16_t sensorData)
{
	float finalData = sensorData * 0.005;
	return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts temperature data output from the regRead() function and returns temperature 
// in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled temperature in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::tempScale(int16_t sensorData)
{
	int signedData = 0;
	int isNeg = sensorData & 0x8000;
	if (isNeg == 0x8000) // If the number is negative, scale and sign the output
		signedData = sensorData - 0xFFFF;
	else
		signedData = sensorData;
	float finalData = (signedData * 0.05) + 25; // Multiply by temperature scale and add 25 to equal 0x0000
	return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts integrated angle data output from the regRead() function and returns delta angle in degrees
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled delta angle in degrees
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::deltaAngleScale(int16_t sensorData)
{
	float finalData = sensorData * 0.005; // Multiply by delta angle scale (0.005 degrees/LSB)
	return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts integrated velocity data output from the regRead() function and returns delta velocity in mm/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled delta velocity in mm/sec
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::deltaVelocityScale(int16_t sensorData)
{
	float finalData = sensorData * 2.5; // Multiply by velocity scale (2.5 mm/sec/LSB)
	return finalData;
}

//Old ADS16305 code
//void ADIS16460::ClearBuffer(void)
//{
//	for (int i = 0; i < 18; i += 2) //Don't count reserved registers 6 and 8
//	{
//		if(i == 6 || i == 8)
//		{//do nothing
//		}
//		else
//		{
//			unsigned char C = 0x00;
//			buffer[i+1] = C; // Fills the element above i with a zero byte
//			C += (2 * i);
//			buffer[i] = C;	 // Sets the LSB address of the	target register
//		}
//	}
//	
//	buffer[0] = (BURST_READ>>8) & 0xff;
//	buffer[1] = BURST_READ & 0xff;
//}
//
//double ADS16305::twoscomptransform(unsigned char nbits, unsigned int num)
//{	
//	unsigned int mask;//, padding;
//    // select correct mask
//    mask = 1 << (nbits -1);
//  
//    // if MSB is 1, then number is negative, so invert it and add one
//    // if MSB is 0, then just return the number 
//    return (num & mask)?( -1.0 * (~(num | 0xFF << nbits)  + 1) ):( 1.0 * num );
//}
//
//// Joins the bytes of data together. Pass array values.
//void JoinData(unsigned char *data, int16_t *words, int length) 
//{
//	length = length / 2;
//	for (int i = 0; i < length; i++)
//	{
//		int index = i * 2;
//		words[i] = ((data[index] << 8) | (data[index] & 0xFF));
//	}
//}
//
//void ADS16305::GetADISReadings(void) 
//{
//  
//  // I believe the format is: Set the buffer to contain the bits required, and then run the SPI RW function with
//  // the channel, the buffer array with the commands to write to the register, 
//  // and the amount of bits that will be first writen, and then replaced by the read bits.
//  // wiringPiSPIDataRW(channel, BufferContainingRegisterInfo, BytesToBeWritten)
//	int length = 20;
//	unsigned char burstData[length];
//	int16_t joinedData[length / 2];
//
//	for (int i = 0; i < length; i++)
//	{
//		  burstData[i] = 0x00;
//	}
//
//  //A check can be drawn here to verify that wiringPiSPIDataRW >= 0. Anything above 0 should be okay.
//  
//  wiringPiSPIDataRW(channel, burstData, length); // Passes the char array into the function, along with the element length of the buffer
//  delayMicroseconds(40);
//  
//  //Every two elements contain two bytes of info
//  //This info needs to be combined to create the single 16 bit (word) answer from the device
//  Diagnostic = SetData(SUPPLYBITS, buffer[0], buffer[1])* SUPPLYSCALE;
//  
//  GyroX = SetData(GYROBITS, buffer[2], buffer[3]);
//  GyroX = twoscomptransform(GYROBITS, GyroX);
//  GyroX *= GYROSCALE;
//  
//  GyroY = SetData(ACCBITS, buffer[4], buffer[5]);
//  GyroY = twoscomptransform(ACCBITS, GyroY);
//  GyroY *= ACCSCALE;
//  
//  GyroZ = SetData(ACCBITS, buffer[6], buffer[7]);
//  //GyroZ = twoscomptransform(ACCBITS, GyroZ);
//  GyroX *= ACCSCALE;
//  
//  AccelX = SetData(ACCBITS, buffer[8], buffer[9]);
//  AccelX = twoscomptransform(ACCBITS, AccelX);
//  AccelX *= ACCSCALE;
//  
//  AccelY = SetData(TEMPBITS, buffer[10], buffer[11]);
//  //AccelY = twoscomptransform(TEMPBITS, AccelY);
//  AccelY *= TEMPSCALE;
//  
//  AccelZ = SetData(PITCHBITS, buffer[12], buffer[13]);
//  AccelZ = twoscomptransform(PITCHBITS, AccelZ);
//  AccelZ *= PITCHSCALE;
//  
//  Temp = SetData(ROLLBITS, buffer[14], buffer[15]);
//  Temp = twoscomptransform(ROLLBITS, Temp);
//  Temp *= ROLLSCALE;
//  
//  SampleCounter = SetData(AUXBITS, buffer[16], buffer[17]); // should be between 0 and 3.3 V -- refer to the datasheet to determine the voltage using the decimal representation
//
//}
//
//// Takes the upper and lower bytes from the array and combines them into an int value
//unsigned int ADS16305::SetData(unsigned char nbits, unsigned char upper, unsigned char lower)
//{
//	unsigned char mask;	  
//	  
//	mask = 0xFF >> (16 - nbits); //for 14 bit transfers, shifts the 0xFF byte by two: one for the ND (New Data) bit, one for the EA (Eror/Alarm) bit.
//	  
//	return   ( ( upper & mask) << 8 | lower)  ; // ANDs the two bytes consisting of 7 bits of data together, and then ORs them to zero out the 1s from the mask.
//}
