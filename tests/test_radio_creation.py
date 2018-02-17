import unittest
from nrf24l01 import NRF24L01



class TestDeviceCreation(unittest.TestCase):

    def setUp(self):
        super().setUp()
        self.radio = NRF24L01(0, 1, 12, 17)

    def testValid(self):
        if self.radio is not None:
            print('happy')
        else:
            print('sad')



if __name__ == '__main__':
    unittest.main()


