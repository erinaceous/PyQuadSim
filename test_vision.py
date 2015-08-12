#!/usr/bin/env python2
"""
    test_vision.py: Test my janky RGB->TCP->RGB inter process comms thing
    (Get a rendered image from v-rep, over the network, convert it to an
    OCV Matrix, and display it)

    Author: Owain Jones [contact@odj.me]
"""

from __future__ import print_function
import socket
import numpy
import cv2


SIZE = (480, 640, 3)
SIZE1D = numpy.product(SIZE)
HOST = ('127.0.0.1', 5011)

if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(HOST)
    while True:
        sock.send('OK')
        buf = sock.recv(SIZE1D, socket.MSG_WAITALL)
        arr = numpy.fromstring(buf[::-1], numpy.uint8)
        arr2d = cv2.flip(arr.reshape(SIZE), 1)
        cv2.imshow("test", arr2d)
        cv2.waitKey(1)
