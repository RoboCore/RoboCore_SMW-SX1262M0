/*******************************************************************************
* SMW_SX1262M0 P2P (v1.0)
* 
* Simple program for a peer-to-peer communication between two
* LoRaWAN modules.
* This program uses the ATmega to communicate with the LoRaWAN module.
* 
* Copyright 2022 RoboCore.
* Written by Francois (15/03/2022).
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

#define TRANSMITTER // comment to use as receiver

// --------------------------------------------------
// Libraries

#include <RoboCore_SMW_SX1262M0.h>

#include <SoftwareSerial.h>

// --------------------------------------------------
// Variables

SoftwareSerial ss(10,11);
SMW_SX1262M0 lorawan(ss);

CommandResponse response;

const uint32_t P2P_FREQUENCY = 915200; // [kHz]

#ifdef TRANSMITTER
const unsigned long PAUSE_TIME = 60000; // [ms] (1 min)
unsigned long timeout;
#else
float rssi, snr;
Buffer data;
const uint32_t TIMEOUT_LISTEN = 1000; // [ms] (1 s)
#endif // TRANSMITTER

// --------------------------------------------------
// --------------------------------------------------

void setup() {
  // start the UART for the computer
  Serial.begin(9600);
  Serial.println(F("--- SMW_SX1262M0 P2P ---"));
  
  // start the UART for the LoRaWAN module
  ss.begin(9600);

  // reset the module
  lorawan.reset();

#ifdef TRANSMITTER
  // do nothing
#else
  // set the module to receive continuously
  lorawan.P2P_start(P2P_FREQUENCY, true);
#endif // TRANSMITTER
}

// --------------------------------------------------
// --------------------------------------------------

void loop() {
#ifdef TRANSMITTER
  if(timeout < millis()){
    // send a message
    response = lorawan.P2P_start(P2P_FREQUENCY, false, "My message"); // send only once
    if(response == CommandResponse::OK){
      Serial.println(F("Message sent"));
    } else {
      Serial.println(F("Error on sending the message"));
    }

    // update the timeout
    timeout = millis() + PAUSE_TIME;
  }
#else
  // listen for incoming message
  response = lorawan.P2P_listen(TIMEOUT_LISTEN, data, rssi, snr);

  if(response == CommandResponse::DATA){
    Serial.print("Data received: \"");
    while(data.available()){
      Serial.write(data.read());
    }
    Serial.print("\" (");
    Serial.print(rssi);
    Serial.print(',');
    Serial.print(snr);
    Serial.println(')');
  }
#endif // TRANSMITTER
}

// --------------------------------------------------
// --------------------------------------------------
