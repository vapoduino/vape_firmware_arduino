#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys, os
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import serial
from collections import deque

SIZE = 250

app = QtGui.QApplication([])

p = pg.plot()
p.setWindowTitle('live plot from serial')
dataA = deque([0] * SIZE)
dataB = deque([0] * SIZE)
curveTemp = p.plot(pen=pg.mkPen('y'))
curveHeat = p.plot(pen=pg.mkPen('r'))

if len(sys.argv) != 2:
    ports = ["/dev/" + dev for dev in os.listdir("/dev/") if dev.startswith("ttyUSB")]

    if len(ports) == 1:
        port = ports[0]
    else:
        print("Please select port:")
        for i, port in enumerate(ports):
            print("[" + str(i) + "] " + port)

        try:
            selection = int(input())

            if selection not in range(len(ports)):
                raise ValueError

            port = ports[selection]
        except ValueError:
            print("Device not listed", file=sys.stderr)
            exit(1)
else:
    port = sys.argv[1]

print("Using port " + port)

raw = serial.Serial(port=port, baudrate=9600, timeout=3)

raw.close()
raw.open()


def update():
    try:
        global curveTemp, dataA
        line = raw.readline().decode()
        print(line)
        values = line[:-2].split(' ')
        try:
            dataA.append(float(values[0]))
            dataA.popleft()
            dataB.append(float(values[1]))
            dataB.popleft()
            curveTemp.setData(np.array(dataA, dtype='float64'))
            curveHeat.setData(np.array(dataB, dtype='float64'))
        except ValueError:
            pass

        app.processEvents()
    except KeyboardInterrupt:
        app.quit()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)

if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
