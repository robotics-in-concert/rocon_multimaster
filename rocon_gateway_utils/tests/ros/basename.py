#!/usr/bin/env python

import sys
import unittest
import rocon_gateway_utils
import rosunit

class TestBaseName(unittest.TestCase):

    def setUp(self):
        pass

    def test_basename(self):
        self.assertEquals("dude", rocon_gateway_utils.gateway_basename('dude1285014a28c74162bf19952d1481197e'))

    def tearDown(self):
        pass

NAME = 'test_basename'
if __name__ == '__main__':
    rosunit.unitrun('test_basename', NAME, TestBaseName, sys.argv, coverage_packages=['rocon_gateway_utils'])
