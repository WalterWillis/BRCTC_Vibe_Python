import gpiozero
import gpiozero.pins.local
import time
usleep = lambda x: time.sleep(x/1000000.0) #time.sleep works well-enough for microseconds above 20 uS https://stackoverflow.com/questions/1133857/how-accurate-is-pythons-time-sleep
from gpiozero.pins import SPI

class UnknownGyro(gpiozero.spi_devices.SPIDevice):
    """
    Represents an analog input device connected to SPI (serial interface).

    Typical analog input devices are `analog to digital converters`_ (ADCs).
    Several classes are provided for specific ADC chips, including
    :class:`MCP3004`, :class:`MCP3008`, :class:`MCP3204`, and :class:`MCP3208`.

    The following code demonstrates reading the first channel of an MCP3008
    chip attached to the Pi's SPI pins::

        from gpiozero import MCP3008

        pot = MCP3008(0)
        print(pot.value)

    The :attr:`value` attribute is normalized such that its value is always
    between 0.0 and 1.0 (or in special cases, such as differential sampling,
    -1 to +1). Hence, you can use an analog input to control the brightness of
    a :class:`PWMLED` like so::

        from gpiozero import MCP3008, PWMLED

        pot = MCP3008(0)
        led = PWMLED(17)
        led.source = pot.values

    The :attr:`voltage` attribute reports values between 0.0 and *max_voltage*
    (which defaults to 3.3, the logic level of the GPIO pins).

    .. _analog to digital converters: https://en.wikipedia.org/wiki/Analog-to-digital_converter
    """

    def __init__(self, max_voltage=3.3, **spi_args):
        self._min_value = -(2 ** bits)
        self._range = 2 ** (bits + 1) - 1
        test = gpiozero.pins.spi.SPISoftwareBus(1,2,3)
        test.clock
        if max_voltage <= 0:
            raise InputDeviceError('max_voltage must be positive')
        self._max_voltage = float(max_voltage)
        super(Gyro, self).__init__(shared=True, **spi_args)


    def _read(self):
        raise NotImplementedError

    @property
    def value(self):
        """
        The current value read from the device, scaled to a value between 0 and
        1 (or -1 to +1 for certain devices operating in differential mode).
        """
        return (2 * (self._read() - self._min_value) / self._range) - 1

    @property
    def raw_value(self):
        """
        The raw value as read from the device.
        """
        return self._read()

    @property
    def max_voltage(self):
        """
        The voltage required to set the device's value to 1.
        """
        return self._max_voltage

    @property
    def voltage(self):
        """
        The current voltage read from the device. This will be a value between
        0 and the *max_voltage* parameter specified in the constructor.
        """
        return self.value * self._max_voltage


#This one looks to inherit from gpiozero.pins.SPI, like InheritedGyro
#requires SpiDev to be initialized. Property: _interface is an instance of SpiDev
    #SpiDev is require to use transfer
#https://github.com/juchong/ADIS16460-Arduino-Teensy/blob/master/ADIS16460/
class OrigGyro(gpiozero.pins.local.LocalPiHardwareSPI):
    def __init__(self, factory, port, device):
        super(OrigGyro, self).__init__()

class ADIS16460():
     #Memory Map
    #region

    # User Register Memory Map from Table 6
    FLASH_CNT = [0x00] #Flash memory write count
    DIAG_STAT = [0x02] #Diagnostic and operational status
    X_GYRO_LOW= [0x04] #X-axis gyroscope output, lower word
    X_GYRO_OUT= [0x06] #X-axis gyroscope output, upper word
    Y_GYRO_LOW= [0x08] #Y-axis gyroscope output, lower word
    Y_GYRO_OUT= [0x0A] #Y-axis gyroscope output, upper word
    Z_GYRO_LOW= [0x0C] #Z-axis gyroscope output, lower word
    Z_GYRO_OUT= [0x0E] #Z-axis gyroscope output, upper word
    X_ACCL_LOW= [0x10] #X-axis accelerometer output, lower word
    X_ACCL_OUT= [0x12] #X-axis accelerometer output, upper word
    Y_ACCL_LOW= [0x14] #Y-axis accelerometer output, lower word
    Y_ACCL_OUT= [0x16] #Y-axis accelerometer output, upper word
    Z_ACCL_LOW= [0x18] #Z-axis accelerometer output, lower word
    Z_ACCL_OUT= [0x1A] #Z-axis accelerometer output, upper word
    SMPL_CNTR = [0x1C] #Sample Counter, MSC_CTRL[3:2]=11
    TEMP_OUT = [0x1E] #Temperature output (internal, not calibrated)
    X_DELT_ANG= [0x24] #X-axis delta angle output
    Y_DELT_ANG= [0x26] #Y-axis delta angle output
    Z_DELT_ANG= [0x28] #Z-axis delta angle output
    X_DELT_VEL= [0x2A] #X-axis delta velocity output
    Y_DELT_VEL= [0x2C] #Y-axis delta velocity output
    Z_DELT_VEL= [0x2E] #Z-axis delta velocity output
    MSC_CTRL = [0x32] #Miscellaneous control
    SYNC_SCAL = [0x34] #Sync input scale control
    DEC_RATE = [0x36] #Decimation rate control
    FLTR_CTRL = [0x38] #Filter control, auto-null record time
    GLOB_CMD = [0x3E] #Global commands
    XGYRO_OFF = [0x40] #X-axis gyroscope bias offset error
    YGYRO_OFF = [0x42] #Y-axis gyroscope bias offset error
    ZGYRO_OFF = [0x44] #Z-axis gyroscope bias offset factor
    XACCL_OFF = [0x46] #X-axis acceleration bias offset factor
    YACCL_OFF = [0x48] #Y-axis acceleration bias offset factor
    ZACCL_OFF = [0x4A] #Z-axis acceleration bias offset factor
    LOT_ID1= [0x52] #Lot identification number
    LOT_ID2= [0x54] #Lot identification number
    PROD_ID= [0x56] #Product identifier
    SERIAL_NUM= [0x58] #Lot-specific serial number
    CAL_SGNTR = [0x60] #Calibration memory signature value
    CAL_CRC= [0x62] #Calibration memory CRC values
    CODE_SGNTR= [0x64] #Code memory signature value
    CODE_CRC= [0x66] #Code memory CRC values

    #endregion

    def __init__(self, spiPort, spiCS):
        self.SpiDevice : gpiozero.pins.local.LocalPiHardwareSPI = gpiozero.pins.local.LocalPiHardwareSPI(gpiozero.devices._default_pin_factory(), spiPort, spiCS)       
        self.SetDefaultMode()

    def SetDefaultMode(self):
        time.sleep(1) # Give the part time to start up
        self.RegWrite(self.MSC_CTRL, 0xC1)  # Enable Data Ready, set polarity
        usleep(20)
        self.RegWrite(self.FLTR_CTRL, 0x500) # Set digital filter
        usleep(20)
        self.RegWrite(self.DEC_RATE, 0x00) # Disable decimation
        usleep(20)

    def RegWrite(self, regAddr, regData):
        # if(self.SpiDevice._get_bits_per_word() != 16):
        #     self.SpiDevice._set_bits_per_word(16)
        # Write register address and data
        addr = (((regAddr[0] & 0x7F) | 0x80) << 8) # Toggle sign bit, and check that the address is 8 bits
        lowWord = (addr | (regData & 0xFF)) # OR Register address (A) with data(D) (AADD)
        highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)) # OR Register address with data and increment address

        # Split words into chars
        highBytehighWord = [(highWord >> 8)]
        lowBytehighWord = [(highWord & 0xFF)]
        highBytelowWord = [(lowWord >> 8)]
        lowBytelowWord = [(lowWord & 0xFF)]

        # Write highWord to SPI bus
        self.SpiDevice.transfer(regAddr) # Write high byte from low word to SPI bus
        self.SpiDevice.transfer([regData]) # Write low byte from low word to SPI bus

        usleep(40) # Delay to not violate read rate (16 us)

        # Write lowWord to SPI bus
        #self.SpiDevice.transfer(highBytehighWord) # Write high byte from high word to SPI bus
        #self.SpiDevice.transfer(lowBytehighWord) # Write low byte from high word to SPI bus

    def RegRead(self, regAddr):
        # if(self.SpiDevice._get_bits_per_word() != 16):
        #     self.SpiDevice._set_bits_per_word(16)

          # Write register address to be read
        self.SpiDevice.transfer(regAddr) # Write address over SPI bus
        #self.SpiDevice.transfer([0x00]) # Write 0x00 to the SPI bus fill the 16 bit transaction requirement

        usleep(40) # Delay to not violate read rate (16 us)

        # Read data from requested register
       

     
        # Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.

        return self.SpiDevice.transfer([0x00])

    def GetBurstData(self): #CLK rate ≤ 1 MHz. 
        if(self.SpiDevice._get_bits_per_word() != 8):
            self.SpiDevice._set_bits_per_word(8)

        #self.SpiDevice._interface.max_speed_hz = 1000000 # May need to close and reopen spi device
        burstdata = []
        burstwords = []

        burstdata.append(self.SpiDevice.transfer([0x00])) #DIAG_STAT
        burstdata.append(self.SpiDevice.transfer([0x00]))
        burstdata.append(self.SpiDevice.transfer([0x00])) #XGYRO_OUT
        burstdata.append(self.SpiDevice.transfer([0x00]))
        burstdata.append(self.SpiDevice.transfer([0x00])) #YGYRO_OUT
        burstdata.append(self.SpiDevice.transfer([0x00]))
        burstdata.append(self.SpiDevice.transfer([0x00])) #ZGYRO_OUT
        burstdata.append(self.SpiDevice.transfer([0x00]))
        burstdata.append(self.SpiDevice.transfer([0x00])) #XACCEL_OUT
        burstdata.append(self.SpiDevice.transfer([0x00]))
        # burstdata.append(self.SpiDevice.transfer([0x00])) #YACCEL_OUT
        # burstdata.append(self.SpiDevice.transfer([0x00]))
        # burstdata.append(self.SpiDevice.transfer([0x00])) #ZACCEL_OUT
        # burstdata.append(self.SpiDevice.transfer([0x00]))
        # burstdata.append(self.SpiDevice.transfer([0x00])) #TEMP_OUT
        # burstdata.append(self.SpiDevice.transfer([0x00]))
        # burstdata.append(self.SpiDevice.transfer([0x00])) #SMPL_CNTR
        # burstdata.append(self.SpiDevice.transfer([0x00]))
        # burstdata.append(self.SpiDevice.transfer([0x00])) #CHECKSUM
        # burstdata.append(self.SpiDevice.transfer([0x00]))

	    # Join bytes into words
        # burstwords.append(((burstdata[0][0] << 8) | (burstdata[1][0] & 0xFF))) #DIAG_STAT
        # burstwords.append(((burstdata[2][0] << 8) | (burstdata[3][0] & 0xFF))) #XGYRO
        # burstwords.append(((burstdata[4][0] << 8) | (burstdata[5][0] & 0xFF))) #YGYRO
        # burstwords.append(((burstdata[6][0] << 8) | (burstdata[7][0] & 0xFF))) #ZGYRO
        # burstwords.append(((burstdata[8][0] << 8) | (burstdata[9][0] & 0xFF))) #XACCEL
        # burstwords.append(((burstdata[10][0] << 8) | (burstdata[11][0] & 0xFF))) #YACCEL
        # burstwords.append(((burstdata[12][0] << 8) | (burstdata[13][0] & 0xFF))) #ZACCEL
        # burstwords.append(((burstdata[14][0] << 8) | (burstdata[15][0] & 0xFF))) #TEMP_OUT
        # burstwords.append(((burstdata[16][0] << 8) | (burstdata[17][0] & 0xFF))) #SMPL_CNTR
        # burstwords.append(((burstdata[18][0] << 8) | (burstdata[19][0] & 0xFF))) #CHECKSUM

        return burstdata

class InheritedGyro(gpiozero.pins.SPI):   
    def __init__(self, factory, port, device):
        super(InheritedGyro, self).__init__()
       

def example():
    factory = gpiozero.devices._default_pin_factory()
    inheritedGyro = InheritedGyro(factory,0,0) # I believe port 0 is the default port
    inheritedGyro.transfer(None) # does nothing. needs to be overwritten.

    list = list(20)
    origGyro = OrigGyro(factory,0,0)
    origGyro._set_bits_per_word(16)
    origGyro.clock_mode(3)
    origGyro.lsb_first(False)
    origGyro._interface #No library available on windows. I think members of this object will have the spi speed getter/setters
    newlist = origGyro.transfer(list) # I think this works like the transfer function from wiring pi. Toss in a list of values, and an equal list of responses is returned.
