/*
 * Created 04/28/2014 - vapoduino.ino
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

#include <PID_v1.h>
#include <avr/sleep.h>
#include "TemperatureSensor.h"
#include "Fading.h"
#include "HeatingElement.h"
#include "BatteryMonitor.h"
#include "Charger.h"

#define MOSFET_GATE_PIN 3
#define LED_RED 9
#define LED_GREEN 6
#define LED_BLUE 5
#define MAX_POWER_PIN 8
#define MAX_CS_PIN 10
#define CHARGER_EN_PIN A0
#define CHARGER_STATUS_PIN A1
#define CHARGER_STANDBY_PIN A2
#define BAT_SENSE_PIN A7

#define PID_P 11          // counter-"force"
#define PID_I 6          // counter-offset
#define PID_D 8         // dampen controlling, react on fast changes
#define PID_P_HEATING 3.6
#define PID_I_HEATING 0.2
#define PID_D_HEATING 4
#define MAX_HEATING_POWER 255
#define AVG_VALUES_COUNT 5
#define VDIV 0.00437428243f // (voltage/raw_value)

double temp, output, desired_temp;
uint8_t too_much_draw_counter;

PID pid(&temp, &output, &desired_temp, PID_P, PID_I, PID_D, DIRECT);
TemperatureSensor tempSensor(MAX_CS_PIN, MAX_POWER_PIN, AVG_VALUES_COUNT);
HeatingElement heatingElement(MOSFET_GATE_PIN);
Charger charger(CHARGER_EN_PIN, CHARGER_STATUS_PIN, CHARGER_STANDBY_PIN);
BatteryMonitor batMonitor(BAT_SENSE_PIN, VDIV);

void setup() {
    Serial.begin(9600);
    Serial.println("#####################################");
    Serial.println("#####      VAPODUINO  v_0.2     #####");
    Serial.println("#####################################\n\n");
    
    // Set pinmodes
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    
    desired_temp = 180;
    
    too_much_draw_counter = 0;

    powerUp();

    temp = tempSensor.getTemp();
    heatUpChamber();

    temp = tempSensor.getTemp();

    pid = PID(&temp, &output, &desired_temp, PID_P, PID_I, PID_D, DIRECT);
    pid.SetOutputLimits(0, 255);
    pid.SetMode(AUTOMATIC);
    digitalWrite(LED_GREEN, LOW);
}

void loop() {
    temp = tempSensor.getTemp();

    pid.Compute();
    heatingElement.setHeat(output);
    if (output == 255) {
        if (too_much_draw_counter > 20) {
            digitalWrite(LED_GREEN, HIGH);
            digitalWrite(LED_RED, LOW);
        } else {
            too_much_draw_counter++;
        }
    } else {
        too_much_draw_counter = 0;
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_RED, HIGH);
    }

    delay(50);
    printStatus();
}

void heatUpChamber() {
    if (batMonitor.getPercentage() < 0) {
    	heatingElement.stopHeat();
        Serial.print("Battery low! ");
        Serial.println(batMonitor.getVoltage());

        // blink 10 times
        for (byte n = 0; n < 10; n++) {
            delay(200);
            digitalWrite(LED_RED, LOW);
            digitalWrite(LED_GREEN, HIGH);
            delay(200);
            digitalWrite(LED_RED, HIGH);
            digitalWrite(LED_GREEN, LOW);
        }

        digitalWrite(LED_GREEN, HIGH);

        // wait forever(!) if battery is to low
        while (!charger.isCharging()); 
    } else {
      // heat up chamber to desired_temp
      double heating_temp = desired_temp * 1.05;
      pid = PID(&temp, &output, &heating_temp, PID_P_HEATING, PID_I_HEATING, PID_D_HEATING, DIRECT);
      pid.SetOutputLimits(0, MAX_HEATING_POWER);
      pid.SetMode(AUTOMATIC);
      
      while (temp < desired_temp) {
          temp = tempSensor.getTemp();
          
          digitalWrite(LED_GREEN, HIGH);
          digitalWrite(LED_RED, LOW);
          pid.Compute();
          heatingElement.setHeat(output);
          printStatus();
          
          delay(50);
      }

      digitalWrite(LED_RED, HIGH);
    }
}


void printStatus() {
    Serial.print(temp);
    Serial.print(" ");
    Serial.print(output);
    Serial.print(" ");
    Serial.print(batMonitor.getPercentage());
    Serial.print(" ");
    Serial.println(batMonitor.getVoltage());
}

void powerUp() {
    // Starting up MAX
    while (!tempSensor.powerUp()) {
        Serial.println("Could not initialize sensor!");
    }
    
    // Bugfix, don't know why...
    temp = tempSensor.getTemp();
    while (temp < 0) {
        // retrying, sometimes it takes some time
    	Serial.println(" Reading negative temps... Retrying");
        temp = tempSensor.getTemp();
        Serial.print(temp);
        Serial.print(" ");
    }
    
    // Start PID control
    pid.SetMode(AUTOMATIC);
  	Serial.print("Battery: ");
  	Serial.print(batMonitor.getPercentage());
  	Serial.print("%,  RAW:");
    Serial.print(analogRead(BAT_SENSE_PIN));
    Serial.print(",  ");
  	Serial.println(batMonitor.getVoltage());
}

void wakeUp() {
    // Disable sleep and interrupt
    sleep_disable();
    detachInterrupt(0);
}
