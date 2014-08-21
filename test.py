#! /usr/bin/env python

from struct import *
import sixaxis
import serial
import time

sixaxis.init("/dev/input/js0")
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

def setBit(var, value, off):
    if value == True:
        return var | (1 << off)
    else:
        return var & (1 << off)

def setBit(v, x, index):
  """Set the index:th bit of v to x, and return the new value."""
  mask = 1 << index
  v &= ~mask
  if x:
    v |= mask
  return v

try:
    while(1):
        state = sixaxis.get_state()
        #ser.write("")
        out = pack("b", state['leftx'])
        out += pack("b", state['lefty'])
        out += pack("b", state['rightx'])
        out += pack("b", state['righty'])

        mask = 0
        mask = setBit(mask, state['trig0'], 0)
        mask = setBit(mask, state['trig1'], 1)
        mask = setBit(mask, state['trig2'], 2)
        mask = setBit(mask, state['trig3'], 3)
        mask = setBit(mask, state['buttonup'], 4)
        mask = setBit(mask, state['buttondown'], 5)
        mask = setBit(mask, state['buttonleft'], 6)
        mask = setBit(mask, state['buttonright'], 7)
        mask = setBit(mask, state['triangle'], 8)
        mask = setBit(mask, state['circle'], 9)
        mask = setBit(mask, state['cross'], 10)
        mask = setBit(mask, state['square'], 11)
        mask = setBit(mask, state['select'], 12)
        mask = setBit(mask, state['start'], 13)
        mask = setBit(mask, state['ps'], 14)

        out += pack("h", mask)

        ser.write(out)
        if(state['triangle'] == True):
            break

        #print 'sending state'
        time.sleep(0.1)

    sixaxis.shutdown()
    ser.close
except KeyboardInterrupt:
    sixaxis.shutdown()
    ser.close()
