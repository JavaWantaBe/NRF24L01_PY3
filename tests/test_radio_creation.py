import unittest
from nrf24l01 import NRF24L01

class TestDeviceCreation(unittest.TestCase):
    pipes = [[0xe7, 0xd3, 0xf0, 0x35, 0x77],
             [0xc2, 0xc2, 0xc2, 0xc2, 0xc2],
             [0xc3],
             [0xc4],
             [0xc5],
             [0xc6]]

    def setUp(self):
        super().setUp()
        self.radio = NRF24L01(0, 0, 22, 18)
        self.radio.address_width = 5
        self.radio.crc = NRF24L01.CRC_ENABLED
        self.radio.crc_length = NRF24L01.CRC_16
        self.radio.pa_level = NRF24L01.PA_HIGH
        self.radio.data_rate = NRF24L01.BR_2MBPS
        self.radio.channel = 76
        self.radio.enable_interrupt(NRF24L01.RX_DR | NRF24L01.TX_DS | NRF24L01.MAX_RT)

    def test_valid(self):
        self.assertIsNotNone(self.radio, 'Initialization of radio failed')

    def test_read_mode(self):
        for k, v in enumerate(self.pipes):
            self.radio.open_rx_pipe(k, v, 32)

        self.radio.start_listening()

    def test_transmit_mode(self):
        for k, v in enumerate(self.pipes):
            self.radio.open_rx_pipe(k, v, 32)

        self.radio.stop_listening()


    def tearDown(self):
        self.radio.stop()


if __name__ == '__main__':
    unittest.main()


