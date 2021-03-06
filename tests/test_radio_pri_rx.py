import unittest
import time
from nrf24l01 import NRF24L01


def data_callback(incoming_data):
    for data in incoming_data:
        print(data)


class TestDeviceReceive(unittest.TestCase):
    pipes = [[0xe7, 0xd3, 0xf0, 0x35, 0x77],
             [0xc2, 0xc2, 0xc2, 0xc2, 0xc2],
             [0xc3],
             [0xc4],
             [0xc5],
             [0xc6]]

    def setUp(self):
        super().setUp()
        self.radio = NRF24L01(0, 0, 22, 18, pri_mode_rx=True, incoming_data_cb=data_callback)
        self.radio.address_width = 5
        self.radio.crc = NRF24L01.CRC_ENABLED
        self.radio.crc_length = NRF24L01.CRC_8
        self.radio.pa_level = NRF24L01.PA_LOW
        self.radio.data_rate = NRF24L01.BR_1MBPS
        self.radio.channel = 76
        self.radio.retries = 5
        self.radio.delay = 1000
        self.radio.enable_interrupt(NRF24L01.RX_DR | NRF24L01.TX_DS | NRF24L01.MAX_RT)
        self.assertEqual(self.radio.state, 'standby_i', 'Standby Mode was not SET')

    def test_read_mode(self):
        for k, v in enumerate(self.pipes):
            self.radio.open_rx_pipe(k, v, 32)
        self.radio.print_details()
        self.radio.start()
        time.sleep(1)
        self.assertEqual(self.radio.state, 'rx_mode', 'State was not RX_MODE')

    # def test_transmit_mode(self):
    #     self.radio.enable_dynamic_payload(0)
    #     self.radio.open_tx_pipe(self.pipes[0], 32)
    #
    #     self.radio.stop_listening()
    #     self.assertEqual(self.radio.state, 'standby_i', 'State was not STANDBY_I')

    # def test_autoack_mode(self):
    #     pass
    #
    # def test_dynamic_payload(self):
    #     pass
    #
    # def test_awk_with_payload(self):
    #     pass

    def tearDown(self):
        self.radio.stop()
        time.sleep(15)


if __name__ == '__main__':
    unittest.main()


