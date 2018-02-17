import unittest
from nrf24l01 import NRF24L01



class TestDeviceCreation(unittest.TestCase):

    def setUp(self):
        super().setUp()
        radio = NRF24L01(0, 1, 12, 17)
        if radio is None:
            print('bad')



if __name__ == '__main__':
    unittest.main()


