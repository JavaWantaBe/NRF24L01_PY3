"""

"""

import spidev
import RPi.GPIO as GPIO
from transitions import Machine, MachineError

import logging

# Use a monotonic clock if available to avoid unwanted side effects from clock
# changes
try:
    from time import monotonic
except ImportError:
    from time import time as monotonic

import time
import math
from threading import Thread, Lock, Event
from queue import Queue

logging.basicConfig(level=logging.DEBUG)


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
    crclength_e_str_P = ["Disabled", "", "8 bits", "", "16 bits"]
    pa_dbm_e_str_P = ["PA_MIN", "PA_LOW", "PA_HIGH", "PA_MAX"]

    states = ['power_down', 'start_up', 'standby_i', 'standby_ii', 'tx_settling', 'tx_mode', 'rx_settling', 'rx_mode']
    transitions = [
        # Power Down -> Startup
        {'trigger': 'power_on', 'source': 'power_down', 'dest': 'start_up', 'after': '_startup_delay'},
        # Startup -> Standby I
        {'trigger': 'start_settled', 'source': 'start_up', 'dest': 'standby_i', 'prepare': 'print_details'},
        # Various states -> Power Down
        {'trigger': 'power_off', 'source': ['rx_settling', 'rx_mode', 'tx_mode', 'tx_settling', 'standby_ii', 'standby_i'], 'dest': 'power_down'},
        # Various states -> Standby I
        {'trigger': 'ce_low', 'source': ['rx_mode', 'rx_settling'], 'dest': 'standby_i'},
        # Standby I -> RX Settling
        {'trigger': 'ce_high', 'source': 'standby_i', 'dest': 'rx_settling', 'conditions': ['_is_prim_rx'], 'after': '_rx_settle_time'},
        # RX Settling -> RX Mode
        {'trigger': 'rx_settled', 'source': 'rx_settling', 'dest': 'rx_mode', 'after': 'process_rx'},
        # RX_MODE -> Standby I
        {'trigger': 'ce_low', 'source': 'rx_mode', 'dest': 'standby_i'},
        # Standby I -> Tx Settling
        {'trigger': 'ce_high', 'source': 'standby_i', 'dest': 'tx_settling', 'unless': ['_is_prim_rx', '_is_tx_fifo_empty'], 'after': '_tx_settle_time'},
        # Standby I -> Standby II
        {'trigger': 'ce_high', 'source': 'standby_i', 'dest': 'standby_ii', 'unless': ['_is_prim_rx'], 'conditions':['_is_tx_fifo_empty']},
        # Tx Settling -> TX Mode
        {'trigger': 'tx_settled', 'source': 'tx_settling', 'dest': 'tx_mode', 'after': 'process_tx'},
        # TX_MODE -> Standby I
        {'trigger': 'ce_low', 'source': 'tx_mode', 'dest': 'standby_i', 'conditions': ['_is_tx_fifo_empty']},
        # TX_MODE -> Standby II
        {'trigger': 'tx_empty', 'source': 'tx_mode', 'dest': 'standby_ii', 'conditions': ['_is_ce', '_is_tx_fifo_empty']},
        # Standby II -> Tx Settling
        {'trigger': 'tx_not_empty', 'source': 'standby_ii', 'dest': 'tx_settling', 'conditions': ['_is_ce'], 'after': '_tx_settle_time'}
    ]

    def __init__(self, major, minor, ce_pin, irq_pin=None, pri_mode_rx=True, incoming_data_cb=None):
        """Initialization

        :param int major:
        :param int minor:
        :param int ce_pin:
        :param int irq_pin: If provided, the radio will use interrupts for events
        :param bool pri_mode_rx: Primary mode, True is primary rx mode False is primary tx mode
        :param func incoming_data_cb: Function to be called when data is incoming
        """
        super(NRF24L01, self).__init__(daemon=True, name='NRF24L01+')
        assert type(major) is int, 'Major needs to be of type int'
        assert type(minor) is int, 'Minor needs to be of type int'
        assert type(ce_pin) is int, 'CE_PIN needs to be of type int'
        # State Machine
        self.machine = Machine(model=self,
                               states=self.states,
                               transitions=self.transitions,
                               initial='power_down')

        self.log = logging.getLogger('NRF24L01')
        self.log.setLevel(logging.DEBUG)

        # Locks and Queues
        self.__rx_queue = Queue()
        self.__tx_queue = Queue()
        self.__radio_lock = Lock()

        # Events
        self.__rx_event = Event()
        self.__tx_event = Event()
        self.__tx_error_event = Event()

        self.__running = False
        self.__pri_mode_rx = pri_mode_rx
        self.__rx_event_cb = incoming_data_cb if incoming_data_cb is not None else self._default_callback

        # Hardware setup
        self.__ce_pin = ce_pin
        self.__spidev = spidev.SpiDev()
        self.__spidev.open(major, minor)
        self.__spidev.bits_per_word = 8
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(True)
        GPIO.setup(self.__ce_pin, GPIO.OUT, initial=GPIO.LOW)
        self.__irq_pin = irq_pin

        # Interrupt driven events
        if self.__irq_pin is not None:
            try:
                GPIO.setup(self.__irq_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                GPIO.add_event_detect(self.__irq_pin, GPIO.FALLING, callback=self._handle_irq)
            except RuntimeError:
                self.log.debug('Unable to create interrupt mode')
                GPIO.cleanup(self.__irq_pin)
                self.__irq_pin = None
        try:
            self.__spidev.max_speed_hz = 10000000  # Maximum supported by NRF24L01L01+
        except IOError:
            self.__spidev.max_speed_hz = 2000000   # Hardware does not support 10MHz

        self.__spidev.cshigh = False # Might need to be pulled low
        self.__spidev.mode = 0
        self.__spidev.loop = False
        self.__spidev.lsbfirst = False
        self.__spidev.threewire = False

        self.__pipe0_reading_address = None  # *< Last address set on pipe 0 for reading.

        # Reset radio configuration
        """ Make sure the NRF is in the same state as after power up to avoid problems resulting from left over 
        configuration from other instances."""
        reset_values = {0x00: 0x08,
                        0x01: 0x3F,
                        0x02: 0x03,
                        0x03: 0x03,
                        0x04: 0x03,
                        0x05: 0x02,
                        0x06: 0x0E,
                        0x0a: [0xe7, 0xe7, 0xe7, 0xe7, 0xe7],
                        0x0b: [0xc2, 0xc2, 0xc2, 0xc2, 0xc2],
                        0x0c: 0xc3,
                        0x0d: 0xc4,
                        0x0e: 0xc5,
                        0x0f: 0xc6,
                        0x10: [0xe7, 0xe7, 0xe7, 0xe7, 0xe7],
                        0x11: 0,
                        0x12: 0,
                        0x13: 0,
                        0x14: 0,
                        0x15: 0,
                        0x16: 0,
                        0x1c: 0,
                        0x1d: 0}
        for reg, value in reset_values.items():
            self._write_reg(reg, value)

        # Flush buffers
        self._flush_rx()
        self._flush_tx()
        self.power_up()

    def _default_callback(self, address, packet_data):
        self.log.debug('Address {}\r\n{}'.format(address, packet_data))

    def _handle_irq(self, gpio):
        """Handles the interrupt

        :param gpio:
        :return:
        """
        if gpio != self.__irq_pin:
            return

        status = self._read_reg(NRF24L01.STATUS)

        if status & NRF24L01.RX_DR:
            self.__rx_event.set()
            status |= NRF24L01.RX_DR
        if status & NRF24L01.TX_DS:
            self.__tx_event.set()
            status |= NRF24L01.TX_DS
        if status & NRF24L01.MAX_RT:
            self.__tx_error_event.set()
            status |= NRF24L01.MAX_RT

        self._write_reg(NRF24L01.STATUS, status)

    def _read_reg(self, reg, length=1):
        """Read LL Register Value

        :param int reg:
        :param int length:
        :return:
        """
        buf = [NRF24L01.R_REGISTER | (NRF24L01.REGISTER_MASK & reg)]
        buf += [NRF24L01.NOP] * max(1, length)

        self.__radio_lock.acquire()
        resp = self.__spidev.xfer2(buf)
        self.__radio_lock.release()
        if length == 1:
            return resp[1]

        return resp[1:]

    def _write_reg(self, reg, data):
        """Write LL Register Value

        :param int reg:
        :param [int, list] data:
        :return:
        """
        if isinstance(data, str):
            data = [ord(x) for x in data]
        elif isinstance(data, int):
            data = list(data.to_bytes(1, byteorder='big'))
        else:
            data = [int(x) for x in data]

        for byte in data:
            if byte < 0 or byte > 255:
                raise RuntimeError("Value %d is larger than 8 bits" % byte)

        buf = [NRF24L01.W_REGISTER | (NRF24L01.REGISTER_MASK & reg)] + data
        self.__radio_lock.acquire()
        result = self.__spidev.xfer2(buf)
        self.__radio_lock.release()
        return result[0]

    def _ce(self, level, pulse=0):
        """CE pin used for transmission of packets or continual transmission mode

        :param int level: GPIO.HIGH or GPIO.LOW
        :param float pulse:
        :return:
        """
        GPIO.output(self.__ce_pin, level)
        self.ce_high() if level == GPIO.HIGH else self.ce_low()

        if pulse > 0:
            time.sleep(pulse)
            GPIO.output(self.__ce_pin, not level)
            self.ce_low() if level == GPIO.HIGH else self.ce_high()

    def _flush_rx(self):
        """Flush RX FIFO, used in RX mode Should not be executed during transmission of
        acknowledge, that is, acknowledge package will not be completed.

        :return:
        """
        self._write_reg(NRF24L01.FLUSH_RX, 0)

    def _flush_tx(self):
        """Flush TX FIFO, used in TX mode

        :return:
        """
        self._write_reg(NRF24L01.FLUSH_TX, 0)

    def _rx_settle_time(self):
        """Settling time after standby to rx_mode

        :return:
        """
        time.sleep(130e-9)
        self.rx_settled()

    def _tx_settle_time(self):
        """Settling time after standby to tx_mode

        :return:
        """
        time.sleep(130e-9)
        self.tx_settled()

    def _startup_delay(self):
        """Startup delay after power up

        :return:
        """
        time.sleep(150e-6)
        self.start_settled()

    def _toggle_features(self):
        """Toggles the special features

        :return:
        """
        self._write_reg(NRF24L01.ACTIVATE, 0x73)

    def _is_prim_rx(self):
        """Returns status of PRIM_RX. If primary RX mode returns True

        :return:
        """
        return True if self._read_reg(NRF24L01.CONFIG) & 0x01 else False

    def _is_tx_fifo_empty(self):
        """Returns True if the fifo is empty

        :return:
        """
        return True if self._read_reg(NRF24L01.FIFO_STATUS) & 0x10 else False

    def _is_ce(self):
        """Returns True if CE is HIGH

        :return:
        """
        return True if GPIO.input(self.__ce_pin) == GPIO.HIGH else False

    def _write_payload(self, buf):
        """Writes data to the payload register, automatically padding it
        to match the required length. Returns the number of bytes
        actually written.

        :param bytes buf:
        :return:
        """
        if self.is_dynamic_payload(0):
            if len(buf) > self.MAX_PAYLOAD_SIZE:
                raise RuntimeError("Dynamic payload is larger than the " + "maximum size.")
        else:
            payload_size = self.payload_size(0)

            if len(buf) > payload_size:
                raise RuntimeError("Payload is larger than the fixed payload" + "size (%d vs. %d bytes)" % (len(buf), payload_size))
            buf += ([0x00] * (payload_size - len(buf)))

        self._write_reg(NRF24L01.W_TX_PAYLOAD, buf)
        self._ce(GPIO.HIGH, 10e-6)
        return len(buf)

    def _read_payload(self):
        """Reads data from the payload register and sets the
        DR bit of the STATUS register.

        :return int: payload size
        """
        pipe = (self._read_reg(NRF24L01.STATUS) & NRF24L01.RX_P_NO_MASK) >> NRF24L01.RX_P_NO
        p_size = self._read_reg(NRF24L01.R_RX_PL_WID) if self.is_dynamic_payload(pipe) else self.payload_size(pipe)
        payload = self._read_reg(NRF24L01.R_RX_PAYLOAD, p_size)
        address_width = self.address_width

        if pipe <= 1:
            address = self._read_reg((NRF24L01.RX_ADDR_P0 + pipe), address_width)
        else:
            address = self._read_reg(NRF24L01.RX_ADDR_P1, address_width)
            address[address_width - 1] = self._read_reg((NRF24L01.RX_ADDR_P0 + pipe), 1)

        self.__rx_event_cb(address, payload)
        return p_size

    @property
    def status(self):
        """Returns the Status register

        :return byte: status
        """
        return self._read_reg(NRF24L01.NOP, 0)

    # Config registers
    def enable_interrupt(self, interrupt):
        """Enable interrupt

        :param int interrupt:
        :return:
        """
        config = self._read_reg(NRF24L01.CONFIG)
        if interrupt & NRF24L01.RX_DR:
            config |= NRF24L01.RX_DR
        if interrupt & NRF24L01.TX_DS:
            config |= NRF24L01.TX_DS
        if interrupt & NRF24L01.MAX_RT:
            config |= NRF24L01.MAX_RT

        self._write_reg(NRF24L01.CONFIG, config)

    def disable_interrupt(self, interrupt):
        """Disable interrupt

        :param int interrupt:
        :return:
        """
        config = self._read_reg(NRF24L01.CONFIG)
        if interrupt & NRF24L01.RX_DR:
            config &= ~NRF24L01.RX_DR
        if interrupt & NRF24L01.TX_DS:
            config &= ~NRF24L01.TX_DS
        if interrupt & NRF24L01.MAX_RT:
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
        self.power_off()

    def power_up(self):
        """Power up radio

        :return:
        """
        self._write_reg(NRF24L01.CONFIG, self._read_reg(NRF24L01.CONFIG) | NRF24L01.PWR_UP)
        self.power_on()

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
        :param list address:
        :param int payload:
        :return:
        """
        if pipe >= 6:
            raise RuntimeError("Invalid pipe number")
        if pipe >= 2 and len(address) > 1:
            raise RuntimeError("Invalid address length")
        if pipe <= 1 and len(address) != self.address_width:
            raise RuntimeError("Invalid address length")

        if pipe == 0:
            self.__pipe0_reading_address = address

        self._write_reg(NRF24L01.RX_ADDR_P0 + pipe, address)
        self._write_reg(NRF24L01.EN_RXADDR, self._read_reg(NRF24L01.EN_RXADDR) | (1 << pipe))

        if not self.is_dynamic_payload(pipe):
            self._write_reg(NRF24L01.RX_PW_P0 + pipe, payload)

    def close_rx_pipe(self, pipe):
        """Closes RX Pipe

        :param int pipe:
        :return:
        """
        self._write_reg(NRF24L01.EN_RXADDR, self._read_reg(NRF24L01.EN_RXADDR) & ~(1 << pipe))

    def open_tx_pipe(self, address, payload):
        """Opens a TX pipe for transmission as well as expected payload size based on if dynamic payloads have been
        enabled.

        :param bytes address:
        :param int payload:
        :return:
        """
        if len(address) != self.address_width:
            RuntimeError('Address length incorrect')
        # Note that the NRF24L01L01(+) expects it LSB first.
        payload = min(payload, NRF24L01.MAX_PAYLOAD_SIZE)
        self._write_reg(NRF24L01.RX_ADDR_P0, address)
        self._write_reg(NRF24L01.TX_ADDR, address)
        if not self.is_dynamic_payload(0):
            self._write_reg(NRF24L01.RX_PW_P0, payload)

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
        if 3 <= width <= 5:

            self._write_reg(NRF24L01.SETUP_AW, (width - 2))

    @property
    def retries(self):
        """Number of retries on non-ack packets

        :return int: 0 - 15
        """
        return self._read_reg(NRF24L01.SETUP_RETR) & 0x0F

    @retries.setter
    def retries(self, count):
        """Set number of retries before MAX_RT error occurs

        :param count:
        :return:
        """
        count &= 0x0F
        self._write_reg(NRF24L01.SETUP_RETR, (self._read_reg(NRF24L01.SETUP_RETR) & ~0x0F) | count)

    @property
    def delay(self):
        """Delay between retry transmissions

        :return:
        """
        return (self._read_reg(NRF24L01.SETUP_RETR) & 0xF0) >> NRF24L01.ARD

    @delay.setter
    def delay(self, delay_time):
        """Sets the delay between retry transmissions.

        :note: Values between 250us and 4000us

        :param delay_time:
        :return:
        """
        times = [250 * x for x in range(1, 17)]
        delay_time = math.ceil(delay_time / 250.0) * 250.0

        if 0 < delay_time <= 4000:
            if self.data_rate == NRF24L01.BR_250KBPS and delay_time < 500:
                delay_time = 500
            value = times.index(int(delay_time))
            self._write_reg(NRF24L01.SETUP_RETR, (self._read_reg(NRF24L01.SETUP_RETR) & ~0xF0) | (value << NRF24L01.ARD))

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
        if dr & NRF24L01.RF_DR_LOW:
            # '10' = 250KBPS
            return NRF24L01.BR_250KBPS

        if dr & NRF24L01.RF_DR_HIGH:
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
            setup |= NRF24L01.RF_DR_LOW
        elif speed == NRF24L01.BR_2MBPS:
            # Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
            # Making it '01'
            setup |= NRF24L01.RF_DR_HIGH

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
    def retransmit_count(self):
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

    def payload_size(self, pipe):
        """Gets the RX defined payload size

        :param int pipe:
        :return byte:
        """
        return self._read_reg(NRF24L01.RX_PW_P0 + pipe, 1)

    def enable_dynamic_payload(self, pipe):
        """Enables dynamic payload

        :param pipe:
        :return:
        """
        assert (6 > pipe >= 0), 'Pipe is from 0 to 5 not {}'.format(pipe)
        # Enable dynamic payload throughout the system
        self._write_reg(NRF24L01.FEATURE, self._read_reg(NRF24L01.FEATURE) | NRF24L01.EN_DPL)

        # If it didn't work, the features are not enabled
        if not self._read_reg(NRF24L01.FEATURE):
            # So enable them and try again
            self._toggle_features()
            self._write_reg(NRF24L01.FEATURE, self._read_reg(NRF24L01.FEATURE) | NRF24L01.EN_DPL)

        self._write_reg(NRF24L01.DYNPD, self._read_reg(NRF24L01.DYNPD) | (1 << pipe))

    def is_dynamic_payload(self, pipe):
        """Validates if dynamic payloads are enabled

        :param int pipe:
        :return bool:
        """
        assert (6 > pipe >= 0), 'Pipe is from 0 to 5 not {}'.format(pipe)
        return True if self._read_reg(NRF24L01.DYNPD) & (1 << pipe) else False

    def enable_ack_payload(self):
        """Enable auto Acknowledgement

        :return:
        """
        # enable ack payload and dynamic payload features
        self._write_reg(NRF24L01.FEATURE, self._read_reg(NRF24L01.FEATURE) | NRF24L01.EN_ACK_PAY | NRF24L01.EN_DPL)

        # If it didn't work, the features are not enabled
        if not self._read_reg(NRF24L01.FEATURE):
            # So enable them and try again
            self._toggle_features()
            self._write_reg(NRF24L01.FEATURE, self._read_reg(NRF24L01.FEATURE) | NRF24L01.EN_ACK_PAY | NRF24L01.EN_DPL)

        # Enable dynamic payload on pipes 0 & 1
        self._write_reg(NRF24L01.DYNPD, self._read_reg(NRF24L01.DYNPD) | NRF24L01.DPL_P1 | NRF24L01.DPL_P0)

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

    def write_ack_payload(self, pipe, buf):
        data_len = min(len(buf), NRF24L01.MAX_PAYLOAD_SIZE)
        buf = buf[:data_len]
        self._write_reg((NRF24L01.W_ACK_PAYLOAD | (pipe & 0x7)), buf)

    def is_ack_payload_available(self):
        result = self.ack_payload_available
        self.ack_payload_available = False
        return result

    def write_data(self, data, address=None, payload=None):
        if address is not None and payload is not None:
            self.open_tx_pipe(address=address, payload=payload)
        self._write_payload(data)

    def run(self):
        """Overridden run method from Thread class. Thread execution.

        :return:
        """
        if self.__pri_mode_rx:
            # PRIM_RX mode
            self._write_reg(NRF24L01.CONFIG, self._read_reg(NRF24L01.CONFIG) | (NRF24L01.PWR_UP | NRF24L01.PRIM_RX))
            self._ce(GPIO.HIGH)
        else:
            # Enable TX
            self._write_reg(NRF24L01.CONFIG, (self._read_reg(NRF24L01.CONFIG) | NRF24L01.PWR_UP) & ~NRF24L01.PRIM_RX)
            # Enable pipe 0 for auto-ack
            self._write_reg(NRF24L01.EN_RXADDR, self._read_reg(NRF24L01.EN_RXADDR) | 1)

        self.__running = True

        while self.__running:
            time.sleep(1)

        self.power_down()
        GPIO.cleanup(self.__ce_pin)
        if self.__irq_pin is not None:
            GPIO.remove_event_detect(self.__irq_pin)
            GPIO.cleanup(self.__irq_pin)
        if self.__spidev:
            self.__spidev.close()
            self.__spidev = None

    def stop(self):
        self.__running = False

    def process_tx(self):
        while not self._is_tx_fifo_empty():
            # TODO Transmit
            pass

    def process_rx(self):
        pass

    def write(self, buf):
        self.last_error = None
        length = self._write_payload(buf)
        self._ce(GPIO.HIGH)

        sent_at = monotonic()
        packet_time = ((1 + length + self.crc_length + self.address_width) * 8 + 9)/(self.data_rate * 1000.)

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
        self._flush_tx()  # Avoid leaving the payload in tx fifo ! What
        return False


    def print_details(self):
        def print_status(status):
            status_str = "0x{0:02x} RX_DR={1:x} TX_DS={2:x} MAX_RT={3:x} RX_P_NO={4:x} TX_FULL={5:x}".format(
                status,
                1 if status & NRF24L01.RX_DR else 0,
                1 if status & NRF24L01.TX_DS else 0,
                1 if status & NRF24L01.MAX_RT else 0,
                ((status >> NRF24L01.RX_P_NO) & int("111", 2)),
                1 if status & NRF24L01.TX_FULL else 0)

            print("{0:<16}= {1}".format("0x07 STATUS", status_str))

        def print_single_status_line(name, value):
            print("{0:<16}= {1}".format(name, value))

        def print_address_register(name, reg, qty=1):
            address_registers = ["0x{0:>02x}{1:>02x}{2:>02x}{3:>02x}{4:>02x}".format(
                *self._read_reg(reg + r, 5))
                for r in range(qty)]
            print_single_status_line(name, " ".join(address_registers))

        def print_byte_register(name, reg, qty=1):
            registers = ["0x{:0>2x}".format(self._read_reg(reg + r)) for r in range(0, qty)]
            print_single_status_line(name, " ".join(registers))

        def print_observe_tx(value):
            tx_str = "OBSERVE_TX=0x{0:02x}: POLS_CNT={2:x} ARC_CNT={2:x}".format(
                value,
                (value >> NRF24L01.PLOS_CNT) & int("1111", 2),
                (value >> NRF24L01.ARC_CNT) & int("1111", 2))
            print_single_status_line("0x08 OBSERVE_TX", tx_str)

        print_byte_register("\n0x00 CONFIG", NRF24L01.CONFIG)
        print_byte_register("0x01 EN_AA (Auto Acknowledge)", NRF24L01.EN_AA)
        print_byte_register("0x02 EN_RXADDR (Enabled RX Addresses)", NRF24L01.EN_RXADDR)
        print_byte_register("0x03 SETUP_AW", NRF24L01.SETUP_AW)
        print_byte_register("0x04 SETUP RETR", NRF24L01.SETUP_RETR)
        print_byte_register("0x05 RF_CH", NRF24L01.RF_CH)
        print_byte_register("0x06 RF_SETUP", NRF24L01.RF_SETUP)
        print_status(self.status)
        print_observe_tx(self._read_reg(NRF24L01.OBSERVE_TX))
        print_byte_register('0x09 RPD(Received Power Detector)', NRF24L01.RPD)
        print_address_register("0x0A-0x0B RX_ADDR_P0-1", NRF24L01.RX_ADDR_P0, 2)
        print_byte_register("0x0C-0x0F RX_ADDR_P2-5", NRF24L01.RX_ADDR_P2, 4)
        print_address_register("0x10 TX_ADDR", NRF24L01.TX_ADDR)
        print_byte_register("0x11-0x16 RX_PW_P0-6", NRF24L01.RX_PW_P0, 6)
        print_byte_register("0x16 FIFO_STATUS", NRF24L01.FIFO_STATUS)
        print_byte_register("0x1C DYNPD", NRF24L01.DYNPD)
        print_byte_register("0x1D FEATURE", NRF24L01.FEATURE)
        print_single_status_line("Data Rate", NRF24L01.datarate_e_str_P[self.data_rate])
        print_single_status_line("CRC Length", NRF24L01.crclength_e_str_P[self.crc_length])
        print_single_status_line("PA Power", NRF24L01.pa_dbm_e_str_P[self.pa_level])
