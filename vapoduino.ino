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
#define BUTTON_PIN 2
#define LED_GREEN 5
#define LED_BLUE 6
#define MAX_POWER_PIN 8
#define MAX_CS_PIN 10
#define CHARGER_EN_PIN A0
#define CHARGER_STATUS_PIN A1
#define CHARGER_STANDBY_PIN A2
#define BAT_SENSE_PIN A7
#define VIBRATION_PIN 4

#define PID_P 11          // counter-"force"
#define PID_I 6        // counter-offset
#define PID_D 8          // dampen controlling, react on fast changes
#define PID_P_HEATING 3.6
#define PID_I_HEATING 0.2
#define PID_D_HEATING 4
#define MAX_HEATING_POWER 255
#define AVG_VALUES_COUNT 5
#define VDIV 0.004853875f

double temp, output, desired_temp;
boolean new_cycle;
uint8_t too_much_draw_counter;

PID pid(&temp, &output, &desired_temp, PID_P, PID_I, PID_D, DIRECT);
TemperatureSensor tempSensor(MAX_CS_PIN, MAX_POWER_PIN, AVG_VALUES_COUNT);
HeatingElement heatingElement(MOSFET_GATE_PIN);
Charger charger(CHARGER_EN_PIN, CHARGER_STATUS_PIN, CHARGER_STANDBY_PIN);
BatteryMonitor batMonitor(BAT_SENSE_PIN, VDIV);

void setup() {
    Serial.begin(9600);
    Serial.println("#####################################");
    Serial.println("#####      VAPODUINO  v_0.1     #####");
    Serial.println("#####################################\n\n");
    
    // Set pinmodes
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(VIBRATION_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);
    
    desired_temp = 180;
    
    new_cycle = true;
    too_much_draw_counter = 0;

    powerUp();
}

void loop() {
    temp = tempSensor.getTemp();

    if (digitalRead(BUTTON_PIN) == LOW) {
        if (new_cycle) {
            heatUpChamber();
            
            new_cycle = false;
            pid = PID(&temp, &output, &desired_temp, PID_P, PID_I, PID_D, DIRECT);
            pid.SetOutputLimits(0, 255);
            pid.SetMode(AUTOMATIC);
            digitalWrite(LED_GREEN, LOW);
        }
        
        pid.Compute();
        heatingElement.setHeat(output);
        if (output == 255) {
            if (too_much_draw_counter > 20) {
                digitalWrite(LED_GREEN, HIGH);
                digitalWrite(VIBRATION_PIN, HIGH);
                digitalWrite(LED_BLUE, LOW);
            } else {
                too_much_draw_counter++;
            }
        } else {
            too_much_draw_counter = 0;
            digitalWrite(LED_GREEN, LOW);
            digitalWrite(VIBRATION_PIN, LOW);
            digitalWrite(LED_BLUE, HIGH);
        }
    } else {
        powerDown();
        
        digitalWrite(LED_BLUE, LOW);
        powerUp();
        
        output = 0;
        temp = tempSensor.getTemp();
        new_cycle = true;
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
            digitalWrite(LED_BLUE, LOW);
            digitalWrite(LED_GREEN, HIGH);
            delay(200);
            digitalWrite(LED_BLUE, HIGH);
            digitalWrite(LED_GREEN, LOW);
        }

        digitalWrite(LED_GREEN, HIGH);

        while (!charger.isCharging()) {
            powerDown();
        }

        powerUp();
    } else {
      // heat up chamber to desired_temp
      double heating_temp = desired_temp * 1.05;
      pid = PID(&temp, &output, &heating_temp, PID_P_HEATING, PID_I_HEATING, PID_D_HEATING, DIRECT);
      pid.SetOutputLimits(0, MAX_HEATING_POWER);
      pid.SetMode(AUTOMATIC);
      
      while (digitalRead(BUTTON_PIN) == LOW) {
    	  if (temp > desired_temp) {
    	      digitalWrite(VIBRATION_PIN, HIGH);
    	      delay(200);
    	      digitalWrite(VIBRATION_PIN, LOW);
    	      break;
		  }

          temp = tempSensor.getTemp();
          
          digitalWrite(LED_GREEN, HIGH);
          digitalWrite(LED_BLUE, LOW);
          pid.Compute();
          heatingElement.setHeat(output);
          printStatus();
          
          delay(50);
      }

      digitalWrite(LED_BLUE, HIGH);
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

void powerDown() {
    Serial.println("Standby");
    
    heatingElement.stopHeat();
    tempSensor.powerDown();
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(VIBRATION_PIN, LOW);

    // wait for all outputs
    delay(200);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    attachInterrupt(0, wakeUp, LOW);
    sleep_mode();
    
    // Device woke up
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

void delay_with_interrupt(int time, int interrupting_button_state) {
  int counter = 0;
    while (digitalRead(BUTTON_PIN) != interrupting_button_state & counter < time) {
        counter++;
        delay(1);
    }
    // debounce
    delay(20);
}


