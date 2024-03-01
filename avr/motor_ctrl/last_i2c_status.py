#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

from smbus import SMBus


i2c = SMBus(2)
s = i2c.read_byte_data(0x50>>1, 0xa3)
print "0x%x" % s
