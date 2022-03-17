/*******************************************************************************
* SMW_SX1262M0 Bridge (v1.0)
* 
* Simple program to bridge the computer to the LoRaWAN module.
* This program uses the ATmega to communicate with the LoRaWAN module.
* 
* Copyright 2022 RoboCore.
* Written by Francois (11/03/2022).
* 
* 
* This file is part of the SMW_SX1262M0 library ("SMW_SX1262M0-lib").
* 
* "SMW_SX1262M0-lib" is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* "SMW_SX1262M0-lib" is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with "SMW_SX1262M0-lib". If not, see <https://www.gnu.org/licenses/>
*******************************************************************************/

// --------------------------------------------------
// Libraries

#include <RoboCore_SMW_SX1262M0.h>

#include <SoftwareSerial.h>

// --------------------------------------------------
// Variables

SoftwareSerial ss(10,11);
SMW_SX1262M0 lorawan(ss);

// --------------------------------------------------
// --------------------------------------------------

void setup() {
  // start the UART for the computer
  Serial.begin(9600);
  Serial.println(F("--- SMW_SX1262M0 Bridge ---"));
  
  // start the UART for the LoRaWAN module
  ss.begin(9600);
}

// --------------------------------------------------
// --------------------------------------------------

void loop() {
  // SMW_SX1262M0 to computer
  if(ss.available()){
    Serial.write(ss.read());
  }

  // computer to SMW_SX1262M0
  if(Serial.available()){
    ss.write(Serial.read());
  }
}

// --------------------------------------------------
// --------------------------------------------------