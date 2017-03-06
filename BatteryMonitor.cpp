/*
 * Created 12/27/2015 - BatteryMonitor.cpp
 * 
 * Vapoduino - Arduino based controller for a Vaporizer.
 * 
 * Copyright (C) 2014 Benedikt Schlagberger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 
#include "Arduino.h"
#include "BatteryMonitor.h"

BatteryMonitor::BatteryMonitor(int analogPin, double divider) {
  pinMode(analogPin, INPUT);
  _pin = analogPin;
  _divider = divider;
}

double BatteryMonitor::getVoltage() {
  double value = analogRead(_pin);
  return value * _divider;
}

int BatteryMonitor::getPercentage() {
  int percents = (getVoltage() - MINIMUM_VOLTAGE)/(MAXIMUM_VOLTAGE - MINIMUM_VOLTAGE) * 100;
  return percents;
}

