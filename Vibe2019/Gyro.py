import gpiozero
import gpiozero.pins.local
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


class OrigGyro(gpiozero.pins.local.LocalPiHardwareSPI):
    def __init__(self, factory, port, device):
        super(OrigGyro, self).__init__()



class InheritedGyro(gpiozero.pins.SPI):
    def __init__(self, factory, port, device):
        super(InheritedGyro, self).__init__()