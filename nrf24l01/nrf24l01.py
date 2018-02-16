"""

"""

import spidev
import RPi.GPIO as GPIO
from transitions import Machine, MachineError

# Use a monotonic clock if available to avoid unwanted side effects from clock
# changes
try:
    from time import monotonic
except ImportError:
    from time import time as monotonic

import time

from threading import Thread, Lock, Event
from queue import Queue


class NRF24L01(Thread):
    """NRF24L01 Radio

    """
    MAX_CHANNEL = 127
    MAX_PAYLOAD_SIZE = 32

    # PA Levels
    PA_MIN = 0x00
    PA_LOW = 0x01
    PA_HIGH = 0x02
    PA_MAX = 0x03
    PA_ERROR = 0x04

    # Bit rates
    BR_1MBPS = 0
    BR_2MBPS = 1
    BR_250KBPS = 2

    # CRC
    CRC_DISABLED = 0x0
    CRC_8 = 0x02
    CRC_16 = 0x04
    CRC_ENABLED = 0x08

    EN_CRC = 0x08
    CRCO = 0x04

    # Registers
    CONFIG = 0x00
    EN_AA = 0x01
    EN_RXADDR = 0x02
    SETUP_AW = 0x03
    SETUP_RETR = 0x04
    RF_CH = 0x05
    RF_SETUP = 0x06
    STATUS = 0x07
    OBSERVE_TX = 0x08
    RPD = 0x09
    RX_ADDR_P0 = 0x0A
    RX_ADDR_P1 = 0x0B
    RX_ADDR_P2 = 0x0C
    RX_ADDR_P3 = 0x0D
    RX_ADDR_P4 = 0x0E
    RX_ADDR_P5 = 0x0F
    TX_ADDR = 0x10
    RX_PW_P0 = 0x11
    RX_PW_P1 = 0x12
    RX_PW_P2 = 0x13
    RX_PW_P3 = 0x14
    RX_PW_P4 = 0x15
    RX_PW_P5 = 0x16
    FIFO_STATUS = 0x17
    DYNPD = 0x1C
    FEATURE = 0x1D

    # Bit Mnemonics */
    MASK_RX_DR = 0x40
    MASK_TX_DS = 0x20
    MASK_MAX_RT = 0x10

    PWR_UP = 0x02
    PRIM_RX = 0x01
    PLL_LOCK = 0x10
    RX_DR = 0x40
    TX_DS = 0x20
    MAX_RT = 0x10
    TX_FULL = 0x01

    EN_DPL = 0x04
    EN_ACK_PAY = 0x02
    EN_DYN_ACK = 0x01

    # Shift counts
    ARD = 4
    ARC = 0
    PLOS_CNT = 4
    ARC_CNT = 0
    RX_P_NO = 1

    TX_REUSE = 6
    FIFO_FULL = 5
    TX_EMPTY = 4
    RX_FULL = 1
    RX_EMPTY = 0

    DPL_P5 = 5
    DPL_P4 = 4
    DPL_P3 = 3
    DPL_P2 = 2
    DPL_P1 = 1
    DPL_P0 = 0

    #Masks
    RX_P_NO_MASK = 0x0E

    # Instruction Mnemonics
    R_REGISTER = 0x00
    W_REGISTER = 0x20
    REGISTER_MASK = 0x1F
    ACTIVATE = 0x50
    R_RX_PL_WID = 0x60
    R_RX_PAYLOAD = 0x61
    W_TX_PAYLOAD = 0xA0
    W_ACK_PAYLOAD = 0xA8
    FLUSH_TX = 0xE1
    FLUSH_RX = 0xE2
    REUSE_TX_PL = 0xE3
    NOP = 0xFF

    # Non-P omissions
    LNA_HCURR = 0x01
    LNA_ON = 1
    LNA_OFF = 0

    # P model bit Mnemonics
    RF_DR_LOW = 0x20
    RF_DR_HIGH = 0x08
    RF_PWR_LOW = 0x02
    RF_PWR_HIGH = 0x04

    datarate_e_str_P = ["1MBPS", "2MBPS", "250KBPS"]
    model_e_str_P = ["NRF24L01L01", "NRF24L01l01+"]
    crclength_e_str_P = ["Disabled", "", "8 bits", "", "16 bits"]
    pa_dbm_e_str_P = ["PA_MIN", "PA_LOW", "PA_HIGH", "PA_MAX"]

    states = ['power_down', 'start_up', 'standby_i', 'standby_ii', 'tx_setting', 'tx_mode', 'rx_setting', 'rx_mode']

    def __init__(self, major, minor, ce_pin, irq_pin):
        """Intialization

        :param int major:
        :param int minor:
        :param int ce_pin:
        :param int irq_pin:
        """
        super(NRF24L01, self).__init__(daemon=True, name='NRF24L01+')
        assert type(major) is int, 'Major needs to be of type int'
        assert type(minor) is int, 'Minor needs to be of type int'
        assert type(ce_pin) in int, 'CE_PIN needs to be of type int'
        assert type(irq_pin) in int, 'IRQ_PIN needs to be of type int'

        self.machine = Machine(model=self, state=NRF24L01.states, initial='power_down')

        # Locks and Queues
        self.__rx_queue = Queue()
        self.__tx_queue = Queue()
        self.__radio_lock = Lock()
        self.__running = False

        # Hardware setup
        self.__ce_pin = ce_pin
        self.__irq_pin = irq_pin
        self.__spidev = spidev.SpiDev()
        self.__spidev.open(major, minor)
        self.__spidev.bits_per_word = 8
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(True)
        GPIO.setup(self.__ce_pin, GPIO.OUT)
        GPIO.setup(self.__irq_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.__irq_pin, GPIO.FALLING, callback=self._handle_irq)
        try:
            self.__spidev.max_speed_hz = 10000000  # Maximum supported by NRF24L01L01+
        except IOError:
            pass  # Hardware does not support this speed

        self.__spidev.cshigh = False # Might need to be pulled low
        self.__spidev.mode = 0
        self.__spidev.loop = False
        self.__spidev.lsbfirst = False
        self.__spidev.threewire = False

        time.sleep(5 / 1000000.0)
        # Hardware Finished

        self.__pipe0_reading_address = None  # *< Last address set on pipe 0 for reading.
        self.__payload_size = None







        self.ack_payload_available = False      # *< Whether there is an ack payload waiting
        self.dynamic_payloads_enabled = False   # *< Whether dynamic payloads are enabled.
        self.ack_payload_length = 5             # *< Dynamic size of pending ack payload.

        self.last_error = 0

        self.auto_ack = 0x3F

        self.p_variant = False  # False for RF24L01 and true for RF24L01P
        # Reset radio configuration
        self.reset()

        # Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
        # WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
        # sizes must never be used. See documentation for a more complete explanation.
        self.setRetries(int('0101', 2), 15)

        # Restore our default PA level
        self.setPALevel(NRF24L01.PA_MAX)

        # Determine if this is a p or non-p RF24 module and then
        # reset our data rate back to default value. This works
        # because a non-P variant won't allow the data rate to
        # be set to 250Kbps.
        if self.setDataRate(NRF24L01.BR_250KBPS):
            self.p_variant = True

        # Then set the data rate to the slowest (and most reliable) speed supported by all
        # hardware.
        self.setDataRate(NRF24L01.BR_1MBPS)

        # Initialize CRC and request 2-byte (16bit) CRC
        self.setCRCLength(NRF24L01.CRC_16)

        # Disable dynamic payloads, to match dynamic_payloads_enabled setting
        self._write_reg(NRF24L01.DYNPD, 0)

        # Set up default configuration.  Callers can always change it later.
        # This channel should be universally safe and not bleed over into adjacent
        # spectrum.
        self.channel = 76

        self.setRetries(15, 15)

        # Flush buffers
        self._flush_rx()
        self._flush_tx()
        self._clear_irq_flags()

    def _handle_irq(self, gpio):
        pass

    def _read_reg(self, reg, length=1):
        """Read LL Register Value

        :param int reg:
        :param int length:
        :return:
        """
        buf = [NRF24L01.R_REGISTER | (NRF24L01.REGISTER_MASK & reg)]
        buf += [NRF24L01.NOP] * max(1, length)

        resp = self.__spidev.xfer2(buf)
        if length == 1:
            return resp[1]

        return resp[1:]

    def _write_reg(self, reg, value):
        """Write LL Register Value

        :param int reg:
        :param int value:
        :return:
        """
        buf = [NRF24L01.W_REGISTER | (NRF24L01.REGISTER_MASK & reg)]
        buf += self._to_8b_list(value)
        self.__spidev.xfer2(buf)

    def _ce(self, level, pulse=0):
        """CE pin used for transmission of packets or continual transmission mode

        :param int level: GPIO.HIGH or GPIO.LOW
        :param pulse:
        :return:
        """
        if self.__ce_pin is not None:
            GPIO.output(self.__ce_pin, level)
            if pulse > 0:
                time.sleep(pulse)
                GPIO.output(self.__ce_pin, ~level)

    def _flush_rx(self):
        """Flush RX FIFO, used in RX mode Should not be executed during transmission of
        acknowledge, that is, acknowledge package will not be completed.

        :return:
        """
        return self.__spidev.xfer2([NRF24L01.FLUSH_RX])[0]

    def _flush_tx(self):
        """Flush TX FIFO, used in TX mode

        :return:
        """
        return self.__spidev.xfer2([NRF24L01.FLUSH_TX])[0]

    def _toggle_features(self):
        buf = [NRF24L01.ACTIVATE, 0x73]
        self.__spidev.xfer2(buf)

    def _clear_irq_flags(self):
        self._write_reg(NRF24L01.STATUS, NRF24L01.RX_DR | NRF24L01.TX_DS | NRF24L01.MAX_RT)

    @staticmethod
    def print_single_status_line(name, value):
        print("{0:<16}= {1}".format(name, value))

    @staticmethod
    def _to_8b_list(data):
        """Convert an arbitrary iterable or single int to a list of ints
            where each int is smaller than 256."""
        if isinstance(data, str):
            data = [ord(x) for x in data]
        elif isinstance(data, int):
            data = data.to_bytes(2, byteorder='big')
        else:
            data = [int(x) for x in data]

        for byte in data:
            if byte < 0 or byte > 255:
                raise RuntimeError("Value %d is larger than 8 bits" % byte)
        return data

    @property
    def status(self):
        return self.__spidev.xfer2([NRF24L01.NOP])[0]

    # Config registers
    def enable_interrupt(self, interrupt):
        """Enable interrupt

        :param interrupt:
        :return:
        """
        config = self._read_reg(NRF24L01.CONFIG)
        # NRF24L01.RX_DR | NRF24L01.TX_DS | NRF24L01.MAX_RT
        if interrupt == NRF24L01.RX_DR:
            config |= NRF24L01.RX_DR
        elif interrupt == NRF24L01.TX_DS:
            config |= NRF24L01.TX_DS
        elif interrupt == NRF24L01.MAX_RT:
            config |= NRF24L01.MAX_RT

        self._write_reg(NRF24L01.CONFIG, config)

    def disable_interrupt(self, interrupt):
        """Disable interrupt

        :param interrupt:
        :return:
        """
        config = self._read_reg(NRF24L01.CONFIG)
        if interrupt == NRF24L01.RX_DR:
            config &= ~NRF24L01.RX_DR
        elif interrupt == NRF24L01.TX_DS:
            config &= ~NRF24L01.TX_DS
        elif interrupt == NRF24L01.MAX_RT:
            config &= ~NRF24L01.MAX_RT

        self._write_reg(NRF24L01.CONFIG, config)

    @property
    def crc(self):
        """CRC status

        :return:
        """
        config = self._read_reg(NRF24L01.CONFIG) & NRF24L01.EN_CRC
        return True if config else False

    @crc.setter
    def crc(self, enabled):
        """Enable CRC. Forced high if one of the bits in the EN_AA is high

        :return:
        """
        config = self._read_reg(NRF24L01.CONFIG)
        if enabled:
            config |= NRF24L01.EN_CRC
        else:
            config &= ~NRF24L01.EN_CRC
        self._write_reg(NRF24L01.CONFIG, config)

    @property
    def crc_length(self):
        """CRC length

        :return int: 1 or 2 bytes
        """
        config = self._read_reg(NRF24L01.CONFIG) & NRF24L01.CRCO

        return NRF24L01.CRC_16 if (config & NRF24L01.CRCO) else NRF24L01.CRC_8

    @crc_length.setter
    def crc_length(self, length):
        """Set CRC length

        :param int length: 1 or 2 bytes
        :return:
        """
        config = self._read_reg(NRF24L01.CONFIG)

        if length == NRF24L01.CRC_8:
            config &= ~NRF24L01.CRCO
            config |= NRF24L01.EN_CRC
            self._write_reg(NRF24L01.CONFIG, config)
        elif length == NRF24L01.CRC_16:
            config |= NRF24L01.CRCO
            config |= NRF24L01.EN_CRC
            self._write_reg(NRF24L01.CONFIG, config)

    def power_down(self):
        """Power down radio

        :return:
        """
        self._write_reg(NRF24L01.CONFIG, self._read_reg(NRF24L01.CONFIG) & ~NRF24L01.PWR_UP)

    def power_up(self):
        """Power up radio

        :return:
        """
        self._write_reg(NRF24L01.CONFIG, self._read_reg(NRF24L01.CONFIG) | NRF24L01.PWR_UP)
        time.sleep(150e-6)

    def start_listening(self):
        """Set Primary RX

        :return:
        """
        self._write_reg(NRF24L01.CONFIG, self._read_reg(NRF24L01.CONFIG) | NRF24L01.PWR_UP | NRF24L01.PRIM_RX)

        # Restore the pipe0 address, if exists
        if self.__pipe0_reading_address:
            self._write_reg(self.RX_ADDR_P0, self.__pipe0_reading_address)

        # Go!
        self._ce(GPIO.HIGH)

    def stop_listening(self):
        """Set Primary TX

        :return:
        """
        self._ce(GPIO.LOW)

        # Enable TX
        self._write_reg(NRF24L01.CONFIG, (self._read_reg(NRF24L01.CONFIG) | NRF24L01.PWR_UP) & ~NRF24L01.PRIM_RX)

        # Enable pipe 0 for auto-ack
        self._write_reg(NRF24L01.EN_RXADDR, self._read_reg(NRF24L01.EN_RXADDR) | 1)

    def set_auto_ack_pipe(self, pipe, enable=True):
        """Set auto acknowledgement

        :param int pipe:
        :param bool enable:
        :return:
        """
        if pipe <= 6:
            # Enhanced shockburst requires at least 1 byte crc
            if self.crc_length == 0:
                self.crc_length = NRF24L01.CRC_8

            en_aa = self._read_reg(NRF24L01.EN_AA)

            if enable:
                en_aa |= 1 << pipe
            else:
                en_aa &= ~1 << pipe

            self._write_reg(NRF24L01.EN_AA, en_aa)

    def open_rx_pipe(self, pipe, address, payload):
        """Open RX pipe

        :param int pipe:
        :param bytes address:
        :return:
        """
        if pipe >= 6:
            raise RuntimeError("Invalid pipe number")
        if (pipe >= 2 and len(address) > 1) or len(address) > 5:
            raise RuntimeError("Invalid address length")

        if pipe == 0:
            # If this is pipe 0, cache the address.  This is needed because
            # openWritingPipe() will overwrite the pipe 0 address, so
            # startListening() will have to restore it.
            self.__pipe0_reading_address = address

        self._write_reg(NRF24L01.RX_ADDR_P0 + pipe, address)
        self._write_reg(NRF24L01.EN_RXADDR, self._read_reg(NRF24L01.EN_RXADDR) | (1 << pipe))
        if not self.is_dynamic_payload(pipe):
            self._write_reg(NRF24L01.RX_PW_P0 + pipe, self.payload_size)

    def close_reading_pipe(self, pipe):
        """Closes RX Pipe

        :param int pipe:
        :return:
        """
        self._write_reg(NRF24L01.EN_RXADDR, self._read_reg(NRF24L01.EN_RXADDR) & ~(1 << pipe))

    def open_writing_pipe(self, value):
        # Note that the NRF24L01L01(+)
        # expects it LSB first.

        self._write_reg(NRF24L01.RX_ADDR_P0, value)
        self._write_reg(NRF24L01.TX_ADDR, value)
        if not self.dynamic_payloads_enabled:
            self._write_reg(NRF24L01.RX_PW_P0, self.payload_size)

    @property
    def address_width(self):
        """Address width

        :return int: 3 - 5 bytes
        """
        return self._read_reg(NRF24L01.SETUP_AW) + 2

    @address_width.setter
    def address_width(self, width):
        """Sets address width

        :param int width:
        :return:
        """
        if 2 <= width <= 5:
            self._write_reg(NRF24L01.SETUP_AW, width - 2)

    @property
    def retries(self):
        """Number of retries on non-ack packets

        :return int: 0 - 15
        """
        return self._read_reg(NRF24L01.SETUP_RETR) & 0x07

    @retries.setter
    def retries(self, delay, count):
        self._write_reg(NRF24L01.SETUP_RETR, (delay & 0xf) << NRF24L01.ARD | (count & 0xf) << NRF24L01.ARC)
        self.delay = delay * 0.000250
        self.retries = count
        self.max_timeout = (self.payload_size / float(self.data_rate_bits) + self.delay) * self.retries
        self.timeout = (self.payload_size / float(self.data_rate_bits) + self.delay)

    @property
    def channel(self):
        """Channel of 2.4GHz spectrum used to rx/tx

        :return int: value between 0 - 127
        """
        return self._read_reg(NRF24L01.RF_CH)

    @channel.setter
    def channel(self, channel):
        """Channel setting

        :param int channel: values 0 - 127
        :return:
        """
        if channel < 0 or channel > self.MAX_CHANNEL:
            raise RuntimeError("Channel number out of range")
        self._write_reg(NRF24L01.RF_CH, channel)

    # SECTION MISSING ON CONTUNOUS CARRIER & PLL LOCK

    @property
    def data_rate(self):
        """Data rate of the radio

        :return int:
        """
        dr = self._read_reg(NRF24L01.RF_SETUP) & (NRF24L01.RF_DR_LOW | NRF24L01.RF_DR_HIGH)
        # Order matters in our case below
        if dr == NRF24L01.RF_DR_LOW:
            # '10' = 250KBPS
            return NRF24L01.BR_250KBPS
        elif dr == NRF24L01.RF_DR_HIGH:
            # '01' = 2MBPS
            return NRF24L01.BR_2MBPS
        else:
            # '00' = 1MBPS
            return NRF24L01.BR_1MBPS

    @data_rate.setter
    def data_rate(self, speed):
        """Sets radio datarate

        :param int speed:
        :return:
        """
        setup = self._read_reg(NRF24L01.RF_SETUP)
        setup &= ~(NRF24L01.RF_DR_LOW | NRF24L01.RF_DR_HIGH)

        if speed == NRF24L01.BR_250KBPS:
            # Must set the RF_DR_LOW to 1 RF_DR_HIGH (used to be RF_DR) is already 0
            # Making it '10'.
            self.data_rate_bits = 250
            setup |= NRF24L01.RF_DR_LOW
        elif speed == NRF24L01.BR_2MBPS:
            # Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
            # Making it '01'
            self.data_rate_bits = 2000
            setup |= NRF24L01.RF_DR_HIGH
        else:
            # 1Mbs
            self.data_rate_bits = 1000

        self._write_reg(NRF24L01.RF_SETUP, setup)

    @property
    def pa_level(self):
        """PA Level of radio

        :return int: output of radio power
        """
        power = self._read_reg(NRF24L01.RF_SETUP) & (NRF24L01.RF_PWR_LOW | NRF24L01.RF_PWR_HIGH)
        if power == (NRF24L01.RF_PWR_LOW | NRF24L01.RF_PWR_HIGH):
            return NRF24L01.PA_MAX
        elif power == NRF24L01.RF_PWR_HIGH:
            return NRF24L01.PA_HIGH
        elif power == NRF24L01.RF_PWR_LOW:
            return NRF24L01.PA_LOW
        else:
            return NRF24L01.PA_MIN

    @pa_level.setter
    def pa_level(self, level):
        """Sets PA level of radio output

        :param int level:
        :return:
        """
        setup = self._read_reg(NRF24L01.RF_SETUP)
        setup &= ~(NRF24L01.RF_PWR_LOW | NRF24L01.RF_PWR_HIGH)

        if level == NRF24L01.PA_MAX:
            setup |= NRF24L01.RF_PWR_LOW | NRF24L01.RF_PWR_HIGH
        elif level == NRF24L01.PA_HIGH:
            setup |= NRF24L01.RF_PWR_HIGH
        elif level == NRF24L01.PA_LOW:
            setup |= NRF24L01.RF_PWR_LOW
        elif level == NRF24L01.PA_MIN:
            pass
        elif level == NRF24L01.PA_ERROR:
            # On error, go to maximum PA
            setup |= NRF24L01.RF_PWR_LOW | NRF24L01.RF_PWR_HIGH

        self._write_reg(NRF24L01.RF_SETUP, setup)

    @property
    def packet_loss_count(self):
        """Count lost packets. The counter is overflow protected to 15, and discontinues at max until reset.
        The counter is reset by writing to RF_CH

        :return int: 0 - 15
        """
        return (self._read_reg(NRF24L01.OBSERVE_TX) & 0xF0) >> 4

    @property
    def retrainsmit_count(self):
        """Count retransmitted packets. The counter is reset when transmission of a new
        packet starts.

        :return:
        """
        return self._read_reg(NRF24L01.OBSERVE_TX) & 0x0F

    @property
    def carrier_detect(self):
        """Received Power Detector. This register is called CD (Carrier Detect) in the nRF24L01. The name is
        different in nRF24L01+ due to the different input power level threshold for this bit

        :return int: 0 - 1
        """
        return self._read_reg(NRF24L01.RPD) & 0x01

    @property
    def payload_size(self):
        return self.__payload_size

    @payload_size.setter
    def payload_size(self, size):
        self.__payload_size = min(max(size, 1), NRF24L01.MAX_PAYLOAD_SIZE)

    def enable_dynamic_payload(self, pipe):
        if 5 > pipe > 0:
            raise ValueError('Pipe is from 0 to 5')
        # Enable dynamic payload throughout the system
        self._write_reg(NRF24L01.FEATURE, self._read_reg(NRF24L01.FEATURE) | NRF24L01.EN_DPL)

        # If it didn't work, the features are not enabled
        if not self._read_reg(NRF24L01.FEATURE):
            # So enable them and try again
            self._toggle_features()
            self._write_reg(NRF24L01.FEATURE, self._read_reg(NRF24L01.FEATURE) | NRF24L01.EN_DPL)

        self._write_reg(NRF24L01.DYNPD, self._read_reg(NRF24L01.DYNPD) | (1 << pipe))

    def is_dynamic_payload(self, pipe):
        if 5 < pipe < 0:
            raise ValueError('Pipe is from 0 to 5')
        return True if self._read_reg(NRF24L01.DYNPD) & (1 << pipe) else False

    def enable_ack_payload(self):
        # enable ack payload and dynamic payload features
        self._write_reg(NRF24L01.FEATURE, self._read_reg(NRF24L01.FEATURE) | NRF24L01.EN_ACK_PAY | NRF24L01.EN_DPL)

        # If it didn't work, the features are not enabled
        if not self._read_reg(NRF24L01.FEATURE):
            # So enable them and try again
            self._toggle_features()
            self._write_reg(NRF24L01.FEATURE, self._read_reg(NRF24L01.FEATURE) | NRF24L01.EN_ACK_PAY | NRF24L01.EN_DPL)

        # Enable dynamic payload on pipes 0 & 1
        self._write_reg(NRF24L01.DYNPD, self._read_reg(NRF24L01.DYNPD) | NRF24L01.DPL_P1 | NRF24L01.DPL_P0)




















































    def write_payload(self, buf):
        """ Writes data to the payload register, automatically padding it
            to match the required length. Returns the number of bytes
            actually written. """

        buf = self._to_8b_list(buf)
        if self.dynamic_payloads_enabled:
            if len(buf) > self.MAX_PAYLOAD_SIZE:
                raise RuntimeError("Dynamic payload is larger than the " +
                                   "maximum size.")
            blank_len = 0
        else:
            if len(buf) > self.payload_size:
                raise RuntimeError("Payload is larger than the fixed payload" +
                                   "size (%d vs. %d bytes)" % (len(buf), self.payload_size))
            blank_len = self.payload_size - len(buf)

        txbuffer = [NRF24L01.W_TX_PAYLOAD] + buf + ([0x00] * blank_len)
        self.__spidev.xfer2(txbuffer)
        return len(txbuffer) - 1

    def read_payload(self, buf, buf_len=-1):
        """ Reads data from the payload register and sets the
            DR bit of the STATUS register. """

        if buf_len < 0:
            buf_len = self.payload_size

        if not self.dynamic_payloads_enabled:
            data_len = min(self.payload_size, buf_len)
            blank_len = self.payload_size - data_len
        else:
            data_len = self.get_dynamic_payload_size()
            blank_len = 0

        txbuffer = [NRF24L01.R_RX_PAYLOAD] + [NRF24L01.NOP] * (blank_len + data_len + 1)

        payload = self.__spidev.xfer2(txbuffer)
        del buf[:]
        buf += payload[1:data_len + 1]

        self._write_reg(NRF24L01.STATUS, NRF24L01.RX_DR)

        return data_len

    def print_status(self, status):
        status_str = "0x{0:02x} RX_DR={1:x} TX_DS={2:x} MAX_RT={3:x} RX_P_NO={4:x} TX_FULL={5:x}".format(
            status,
            1 if status & NRF24L01.RX_DR else 0,
            1 if status & NRF24L01.TX_DS else 0,
            1 if status & NRF24L01.MAX_RT else 0,
            ((status >> NRF24L01.RX_P_NO) & int("111", 2)),
            1 if status & NRF24L01.TX_FULL else 0)

        self.print_single_status_line("STATUS", status_str)

    def print_observe_tx(self, value):
        tx_str = "OBSERVE_TX=0x{0:02x}: POLS_CNT={2:x} ARC_CNT={2:x}\r\n".format(
            value,
            (value >> NRF24L01.PLOS_CNT) & int("1111", 2),
            (value >> NRF24L01.ARC_CNT) & int("1111", 2))
        self.print_single_status_line("OBSERVE_TX", tx_str)

    def print_byte_register(self, name, reg, qty=1):
        registers = ["0x{:0>2x}".format(self._read_reg(reg+r)) for r in range(0, qty)]
        self.print_single_status_line(name, " ".join(registers))

    def print_address_register(self, name, reg, qty=1):
        address_registers = ["0x{0:>02x}{1:>02x}{2:>02x}{3:>02x}{4:>02x}".format(
            *self._read_reg(reg+r, 5))
            for r in range(qty)]

        self.print_single_status_line(name, " ".join(address_registers))

    def print_details(self):
        self.print_status(self.status)
        self.print_address_register("RX_ADDR_P0-1", NRF24L01.RX_ADDR_P0, 2)
        self.print_byte_register("RX_ADDR_P2-5", NRF24L01.RX_ADDR_P2, 4)
        self.print_address_register("TX_ADDR", NRF24L01.TX_ADDR)

        self.print_byte_register("RX_PW_P0-6", NRF24L01.RX_PW_P0, 6)
        self.print_byte_register("EN_AA", NRF24L01.EN_AA)
        self.print_byte_register("EN_RXADDR", NRF24L01.EN_RXADDR)
        self.print_byte_register("RF_CH", NRF24L01.RF_CH)
        self.print_byte_register("RF_SETUP", NRF24L01.RF_SETUP)
        self.print_byte_register("SETUP_AW", NRF24L01.SETUP_AW)
        self.print_byte_register("OBSERVE_TX", NRF24L01.OBSERVE_TX)
        self.print_byte_register("CONFIG", NRF24L01.CONFIG)
        self.print_byte_register("FIFO_STATUS", NRF24L01.FIFO_STATUS)
        self.print_byte_register("DYNPD", NRF24L01.DYNPD)
        self.print_byte_register("FEATURE", NRF24L01.FEATURE)

        self.print_single_status_line("Data Rate", NRF24L01.datarate_e_str_P[self.getDataRate()])
        self.print_single_status_line("Model", NRF24L01.model_e_str_P[self.isPVariant()])
        self.print_single_status_line("CRC Length", NRF24L01.crclength_e_str_P[self.getCRCLength()])
        self.print_single_status_line("PA Power", NRF24L01.pa_dbm_e_str_P[self.getPALevel()])

    def write(self, buf):
        self.last_error = None
        length = self.write_payload(buf)
        self._ce(GPIO.HIGH)

        sent_at = monotonic()
        packet_time = ((1 + length + self.crc_length + self.address_length) * 8 + 9)/(self.data_rate_bits * 1000.)

        if self.auto_ack != 0:
            packet_time *= 2

        if self.retries != 0 and self.auto_ack != 0:
            timeout = sent_at + (packet_time + self.delay)*self.retries
        else:
            timeout = sent_at + packet_time * 2  # 2 is empiric

        #while NRF24L01.TX_DS &  self.get_status() == 0:
        #    pass

        #print monotonic() - sent_at
        #print packet_time

        while monotonic() < timeout:
            time.sleep(packet_time)
            status = self.status
            if status & NRF24L01.TX_DS:
                self._ce(GPIO.LOW)
                return True

            if status & NRF24L01.MAX_RT:
                self.last_error = 'MAX_RT'
                self._ce(GPIO.LOW)
                break

        self._ce(GPIO.LOW)
        if self.last_error is None:
            self.last_error = 'TIMEOUT'

        self._flush_tx()  # Avoid leaving the payload in tx fifo ! What
        return False

    def start_fast_write(self, buf):
        """
            Do not wait for CE HIGH->LOW
        """
        # Send the payload
        self.write_payload(buf)

        self._ce(GPIO.HIGH)

    def start_write(self, buf):
        # Send the payload
        self.write_payload(buf)

        # Allons!
        self._ce(1, 10e-6)

    def get_dynamic_payload_size(self):
        return self.__spidev.xfer2([NRF24L01.R_RX_PL_WID, NRF24L01.NOP])[1]

    def available(self, pipe_num=None, irq_wait=False, irq_timeout=30000):
        status = self.status
        result = False

        # Sometimes the radio specifies that there is data in one pipe but
        # doesn't set the RX flag...
        if status & NRF24L01.RX_DR or (status & NRF24L01.RX_P_NO_MASK != NRF24L01.RX_P_NO_MASK):
            result = True
        else:
            if irq_wait:  # Will use IRQ wait
                if self._irqWait(irq_timeout):  # Do we have a packet?
                    status = self.status  # Seems like we do!
                    if status & NRF24L01.RX_DR or (status & NRF24L01.RX_P_NO_MASK != NRF24L01.RX_P_NO_MASK):
                        result = True


        if result and pipe_num is not None:
            del pipe_num[:]
            pipe_num.append((status & NRF24L01.RX_P_NO_MASK) >> NRF24L01.RX_P_NO)

        # Handle ack payload receipt
        if status & NRF24L01.TX_DS:
            self._write_reg(NRF24L01.STATUS, NRF24L01.TX_DS)

        return result

    def read(self, buf, buf_len=-1):
        # Fetch the payload
        self.read_payload(buf, buf_len)

        # was this the last of the data available?
        return self._read_reg(NRF24L01.FIFO_STATUS & NRF24L01.RX_EMPTY)

    def what_happened(self):
        # Read the status & reset the status in one easy call
        # Or is that such a good idea?
        self._write_reg(NRF24L01.STATUS, NRF24L01.RX_DR | NRF24L01.TX_DS | NRF24L01.MAX_RT)

        status = self.status
        self._clear_irq_flags()

        # Report to the user what happened
        tx_ok = status & NRF24L01.TX_DS
        tx_fail = status & NRF24L01.MAX_RT
        rx_ready = status & NRF24L01.RX_DR
        return {'tx_ok': tx_ok, "tx_fail": tx_fail, "rx_ready": rx_ready}

    def write_ack_payload(self, pipe, buf, buf_len):
        txbuffer = [NRF24L01.W_ACK_PAYLOAD | (pipe & 0x7)]

        max_payload_size = 32
        data_len = min(buf_len, max_payload_size)
        txbuffer.extend(buf[0:data_len])

        self.__spidev.xfer2(txbuffer)

    def is_ack_payload_available(self):
        result = self.ack_payload_available
        self.ack_payload_available = False
        return result

    def getMaxTimeout(self):
        return self.max_timeout

    def getTimeout(self):
        return self.timeout

    def reset(self):
        """ Make sure the NRF is in the same state as after power up
            to avoid problems resulting from left over configuration
            from other programs."""
        self._ce(GPIO.LOW)
        reset_values = {0: 0x08, 1: 0x3F, 2: 0x02, 3: 0x03, 4: 0x03, 5: 0x02, 6: 0x06,
                        0x0a: [0xe7, 0xe7, 0xe7, 0xe7, 0xe7],
                        0x0b: [0xc2, 0xc2, 0xc2, 0xc2, 0xc2],
                        0x0c: 0xc3, 0x0d: 0xc4, 0x0e: 0xc5, 0x0f: 0xc6,
                        0x10: [0xe7, 0xe7, 0xe7, 0xe7, 0xe7],
                        0x11: 0, 0x12: 0, 0x13: 0, 0x14: 0, 0x15: 0, 0x16: 0,
                        0x1c: 0, 0x1d: 0}
        for reg, value in reset_values.items():
            self._write_reg(reg, value)

        self._flush_rx()
        self._flush_tx()

    def start(self):
        self.__running = True
        while self.__running:
            time.sleep(1)

    def stop(self):
        self.__running = False
        if self.__spidev:
            self.__spidev.close()
            self.__spidev = None