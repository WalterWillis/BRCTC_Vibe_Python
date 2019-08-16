import gpiozero
import gpiozero.pins.local
import time
import spidev
import datetime
usleep = lambda x: time.sleep(x/1000000.0) #time.sleep works well-enough for microseconds above 20 uS https://stackoverflow.com/questions/1133857/how-accurate-is-pythons-time-sleep
from gpiozero.pins import SPI
from ctypes import c_short, c_byte, c_uint
from enum import Enum

class ADIS16460Enum(Enum):
     # User Register Memory Map from Table 6
    FLASH_CNT = 0x00 #Flash memory write count
    DIAG_STAT = 0x02 #Diagnostic and operational status
    X_GYRO_LOW= 0x04 #X-axis gyroscope output, lower word
    X_GYRO_OUT= 0x06 #X-axis gyroscope output, upper word
    Y_GYRO_LOW= 0x08 #Y-axis gyroscope output, lower word
    Y_GYRO_OUT= 0x0A #Y-axis gyroscope output, upper word
    Z_GYRO_LOW= 0x0C #Z-axis gyroscope output, lower word
    Z_GYRO_OUT= 0x0E #Z-axis gyroscope output, upper word
    X_ACCL_LOW= 0x10 #X-axis accelerometer output, lower word
    X_ACCL_OUT= 0x12 #X-axis accelerometer output, upper word
    Y_ACCL_LOW= 0x14 #Y-axis accelerometer output, lower word
    Y_ACCL_OUT= 0x16 #Y-axis accelerometer output, upper word
    Z_ACCL_LOW= 0x18 #Z-axis accelerometer output, lower word
    Z_ACCL_OUT= 0x1A #Z-axis accelerometer output, upper word
    SMPL_CNTR = 0x1C #Sample Counter, MSC_CTRL[3:2=11
    TEMP_OUT = 0x1E #Temperature output (internal, not calibrated)
    X_DELT_ANG= 0x24 #X-axis delta angle output
    Y_DELT_ANG= 0x26 #Y-axis delta angle output
    Z_DELT_ANG= 0x28 #Z-axis delta angle output
    X_DELT_VEL= 0x2A #X-axis delta velocity output
    Y_DELT_VEL= 0x2C #Y-axis delta velocity output
    Z_DELT_VEL= 0x2E #Z-axis delta velocity output
    MSC_CTRL = 0x32 #Miscellaneous control
    SYNC_SCAL = 0x34 #Sync input scale control
    DEC_RATE = 0x36 #Decimation rate control
    FLTR_CTRL = 0x38 #Filter control, auto-null record time
    GLOB_CMD = 0x3E #Global commands
    XGYRO_OFF = 0x40 #X-axis gyroscope bias offset error
    YGYRO_OFF = 0x42 #Y-axis gyroscope bias offset error
    ZGYRO_OFF = 0x44 #Z-axis gyroscope bias offset factor
    XACCL_OFF = 0x46 #X-axis acceleration bias offset factor
    YACCL_OFF = 0x48 #Y-axis acceleration bias offset factor
    ZACCL_OFF = 0x4A #Z-axis acceleration bias offset factor
    LOT_ID1= 0x52 #Lot identification number
    LOT_ID2= 0x54 #Lot identification number
    PROD_ID= 0x56 #Product identifier
    SERIAL_NUM= 0x58 #Lot-specific serial number
    CAL_SGNTR = 0x60 #Calibration memory signature value
    CAL_CRC= 0x62 #Calibration memory CRC values
    CODE_SGNTR= 0x64 #Code memory signature value
    CODE_CRC= 0x66 #Code memory CRC values


class ADCEnum(Enum):
    SINGLE_0 = 0b1000,  # single channel 0 */
    SINGLE_1 = 0b1001,  # single channel 1 */
    SINGLE_2 = 0b1010,  # single channel 2 */
    SINGLE_3 = 0b1011,  # single channel 3 */
    SINGLE_4 = 0b1100,  # single channel 4 */
    SINGLE_5 = 0b1101,  # single channel 5 */
    SINGLE_6 = 0b1110,  # single channel 6 */
    SINGLE_7 = 0b1111,  # single channel 7 */
    DIFF_0PN = 0b0000,  # differential channel 0 (input 0+,1-) */
    DIFF_0NP = 0b0001,  # differential channel 0 (input 0-,1+) */
    DIFF_1PN = 0b0010,  # differential channel 1 (input 2+,3-) */
    DIFF_1NP = 0b0011,  # differential channel 1 (input 2-,3+) */
    DIFF_2PN = 0b0100,  # differential channel 2 (input 4+,5-) */
    DIFF_2NP = 0b0101,  # differential channel 2 (input 5-,5+) */
    DIFF_3PN = 0b0110,  # differential channel 3 (input 6+,7-) */
    DIFF_3NP = 0b0111   # differential channel 3 (input 6-,7+) */

    #Currently broken. Gpiozero for multi-spi use is unclear and over-engineered. Recommend using spidev instead. Use working C++ code instead.
class SpiHub():
   
    GyroPort: int
    GyroDevice: int
    ADCPort: int
    ADCDevice: int

    def SetDevice(self, port, device):
        if self.SpiDevice._device != device:
            self.SpiDevice._interface.close()
            self.SpiDevice._device = device
            self.SpiDevice._interface.open(port, device)


    def __init__(self, gyroPort, gyroCS, adcPort, adcCS):
        self.SpiDevice : gpiozero.pins.local.LocalPiHardwareSPI = gpiozero.pins.local.LocalPiHardwareSPI(gpiozero.devices._default_pin_factory(), port=spiPort, device=spiCS)
        self.SpiDevice._interface.max_speed_hz = 1000000
        self.GyroPort = gyroPort
        self.GyroDevice = gyroCS
        self.ADCPort = adcPort
        self.ADCDevice = adcCS
        # if(self.SpiDevice._get_bits_per_word() != 8):
        #     self.SpiDevice._set_bits_per_word(8)      
        #self.SetDefaultMode()

    def GyroSetDefaultMode(self):
        self.SetDevice(self.GyroPort, self.GyroDevice)
        time.sleep(1) # Give the part time to start up
        self.RegWrite(ADIS16460Enum.MSC_CTRL, 0xC1)  # Enable Data Ready, set polarity
        usleep(20)
        self.RegWrite(ADIS16460Enum.FLTR_CTRL, 0x500) # Set digital filter
        usleep(20)
        self.RegWrite(ADIS16460Enum.DEC_RATE, 0x00) # Disable decimation
        usleep(20)

    def GryoRegWrite(self, regAddr, regData):
        self.SetDevice(self.GyroPort, self.GyroDevice)
        # if(self.SpiDevice._get_bits_per_word() != 16):
        #     self.SpiDevice._set_bits_per_word(16)
        # Write register address and data
        addr = (((regAddr & 0x7F) | 0x80) << 8) # Toggle sign bit, and check that the address is 8 bits
        lowWord = (addr | (regData & 0xFF)) # OR Register address (A) with data(D) (AADD)
        highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)) # OR Register address with data and increment address

        # # Split words into chars
        highBytehighWord = (highWord >> 8)
        lowBytehighWord = (highWord & 0xFF)
        highBytelowWord = (lowWord >> 8)
        lowBytelowWord = (lowWord & 0xFF)

        # Write highWord to SPI bus
        self.SpiDevice.transfer([highBytehighWord, lowBytehighWord]) 

        usleep(40) # Delay to not violate read rate (16 us)

        # Write lowWord to SPI bus
        self.SpiDevice.transfer([highBytelowWord, lowBytelowWord]) 

    def GyroRegRead(self, regAddr):
        self.SetDevice(self.GyroPort, self.GyroDevice)
        # if(self.SpiDevice._get_bits_per_word() != 16):
        #     self.SpiDevice._set_bits_per_word(16)

        # Write register address to be read
        self.SpiDevice.transfer([regAddr, 0x00]) # Write address over SPI bus
        # Write 0x00 to the SPI bus fill the 16 bit transaction requirement

        usleep(50) # Delay to not violate read rate (16 us)

        # Read data from requested register
        return self.SpiDevice.transfer([0x00,0x00])

    def GyroGetBurstData(self): #CLK rate ≤ 1 MHz.     
        self.SetDevice(self.GyroPort, self.GyroDevice)
        burstdata = [c_byte]
        burstwords = [c_short]

        burstTrigger = [0x3E,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00]
        
        burstdata = self.SpiDevice.transfer(burstTrigger)
        #cTime: datetime = datetime.datetime.now().time()
        #print(f"burst data: {burstdata}")
        
        #This is old
        #burstwords.append(((burstdata[2] << 8) | (burstdata[3] & 0xFF))) #DIAG_STAT
        #burstwords.append(((burstdata[4] << 8) | (burstdata[5] & 0xFF))) #XGYRO
        #burstwords.append(((burstdata[6] << 8) | (burstdata[7] & 0xFF))) #YGYRO
        #burstwords.append(((burstdata[8] << 8) | (burstdata[9] & 0xFF))) #ZGYRO
        #burstwords.append(((burstdata[10] << 8) | (burstdata[11] & 0xFF))) #XACCEL
        #burstwords.append(((burstdata[12] << 8) | (burstdata[13] & 0xFF))) #YACCEL
        #burstwords.append(((burstdata[14] << 8) | (burstdata[15] & 0xFF))) #ZACCEL
        #burstwords.append(((burstdata[16] << 8) | (burstdata[17] & 0xFF))) #TEMP_OUT
        #burstwords.append(((burstdata[18] << 8) | (burstdata[19] & 0xFF))) #SMPL_CNTR
        #burstwords.append(((burstdata[20] << 8) | (burstdata[21] & 0xFF))) #CHECKSUM

        #Endianness requires the bytes to be inverted
        burstwords.append(((burstdata[3] << 8) | (burstdata[2] & 0xFF))) #DIAG_STAT
        burstwords.append(((burstdata[5] << 8) | (burstdata[4] & 0xFF))) #XGYRO
        burstwords.append(((burstdata[7] << 8) | (burstdata[6] & 0xFF))) #YGYRO
        burstwords.append(((burstdata[9] << 8) | (burstdata[8] & 0xFF))) #ZGYRO
        burstwords.append(((burstdata[11] << 8) | (burstdata[10] & 0xFF))) #XACCEL
        burstwords.append(((burstdata[13] << 8) | (burstdata[12] & 0xFF))) #YACCEL
        burstwords.append(((burstdata[15] << 8) | (burstdata[14] & 0xFF))) #ZACCEL
        burstwords.append(((burstdata[17] << 8) | (burstdata[16] & 0xFF))) #TEMP_OUT
        burstwords.append(((burstdata[19] << 8) | (burstdata[18] & 0xFF))) #SMPL_CNTR
        burstwords.append(((burstdata[21] << 8) | (burstdata[20] & 0xFF))) #CHECKSUM
        #burstwords.append(cTime.strftime("%H:%M:%S"))

        #print(f"burst words: {burstwords}")

        return burstwords
    #Logic learned from https://github.com/labfruits/mcp3208
    def ADCRead(self, channel):
        self.SetDevice(self.ADCPort, self.ADCDevice)
        _fullByte: c_short = (0x0400 | (channel << 6))

        #first byte is simply talking the register, don't use the returned value [byte1, byte2, byte3]
        data:[c_byte] = self.SpiDevice.transfer([(_fullByte >> 8) & 0xFF, (_fullByte & 0xFF), 0x00])

        return ((data[1] & 0xFF) << 8 | (data[2] & 0xFF)) #combine the 2 usable bytes

    def ADCGetValues(self, channels):
        values = []
        for channel in channels:
            raw = ADCRead(channel)
            analog: c_uint = self.GetAnalog(raw)
            digital : c_uint = self.GetVoltage(raw)
            values.append(raw,analog,digital)

    def GetDigital(self, value):
        return value * 11 / 3.3

    def GetAnalog(self, value):
        return value * 3.3 / 11


#right_byte = short_val & 0xFF;
#left_byte = ( short_val >> 8 ) & 0xFF

#short_val = ( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )
def GetChecksum(self, burstArray):
    sum = 0
    for i in range(0,burstArray.len):	
        sum += (burstArray[i] & 0xFF)
        sum += ((burstArray[i] >> 8) & 0xFF)

    print(f"sum vs checksum: {sum}, {burstArray[9]}")

    if sum == burstArray[9]:
        return True
    else:
        return False


def PrintGyroStuff(burstArray):
    print(f"Gyro X Axis: {burstArray[1] * .005}")
    print(f"Gyro Y Axis: {burstArray[2] * .005}")
    print(f"Gyro Z Axis: {burstArray[3] * .005}")

    print(f"X Axis: {burstArray[4] * .00025}")
    print(f"Y Axis: {burstArray[5] * .00025}")
    print(f"Z Axis: {burstArray[6] * .00025}")
        