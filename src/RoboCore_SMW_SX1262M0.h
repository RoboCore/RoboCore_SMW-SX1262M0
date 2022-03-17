#ifndef SMW_SX1262M0_H
#define SMW_SX1262M0_H

/*******************************************************************************
* RoboCore SMW_SX1262M0 Library (v1.0)
* 
* Library to use the SMW_SX1262M0 LoRaWAN module.
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

#define SMW_SX1262M0_DEBUG

#define SMW_SX1262M0_BUFFER_SIZE            70
#define SMW_SX1262M0_DELAY_INCOMING_DATA    10 // [ms]
#define SMW_SX1262M0_TIMEOUT_READ          100 // [ms]
#define SMW_SX1262M0_TIMEOUT_RESET        3000 // [ms]
#define SMW_SX1262M0_TIMEOUT_WRITE         500 // [ms]


// --------------------------------------------------
// Libraries

#include <Arduino.h>

extern "C" {
  #include <stdarg.h>
  #include <stdint.h>
  #include <stdlib.h>
}

#include "Buffer.h"


// --------------------------------------------------
// Constants

const char CHAR_EOS = '\0'; // End Of String
const char CHAR_LF = 10; // Line Feed
const char CHAR_CR = 13; // Carriage Return
const char CHAR_SPACE = 32; // ' '
const char CHAR_PLUS = 43; // '+'
const char CHAR_COLON = 58; // ':'
const char CHAR_LT = 60; // '<'
const char CHAR_EQUAL = 61; // '='
const char CHAR_GT = 62; // '>'
const char CHAR_QUESTION = 63; // '?'


// --------------------------------------------------
// Constants (AT v2.14)

const char* const CMD_AT = "AT";

char* const CMD_NONE = nullptr;

const char* const CMD_RESET = "ATZ"; // Reset (3.1.3)

const char* const CMD_APPEUI = "APPEUI"; // Application EUI (3.2.1)
const char* const CMD_APPKEY = "APPKEY"; // Application Key (3.2.1)
const char* const CMD_APPSKEY = "APPSKEY"; // Application Session Key (3.2.3)
const char* const CMD_DADDR = "DADDR"; // Device Address (3.2.4)
const char* const CMD_DEVEUI = "DEUI"; // Device EUI (3.2.5)
const char* const CMD_NWKID = "NWKID"; // Network ID (3.2.6)
const char* const CMD_NWKSKEY = "NWKSKEY"; // Network Session Key (3.2.7)
const char* const CMD_CFM = "CFM"; // Confirm Mode (3.3.1)
const char* const CMD_CFS = "CFS"; // Confirm Status (3.3.2)
const char* const CMD_JOIN = "JOIN"; // Join (3.3.3)
const char* const CMD_NJM = "NJM"; // Join Mode (3.3.4)
const char* const CMD_NJS = "NJS"; // Join Status (3.3.5)
const char* const CMD_RECV = "RECV"; // Receive (3.3.6)
const char* const CMD_RECVB = "RECVB"; // Receive - Binary (3.3.7)
const char* const CMD_SEND = "SEND"; // Send (3.3.8)
const char* const CMD_SENDB = "SENDB"; // Send - Binary (3.3.9)
const char* const CMD_ADR = "ADR"; // Adaptive Data Rate (3.4.1)
const char* const CMD_CLASS = "CLASS"; // LoRaWAN Class (3.4.2)
const char* const CMD_DR = "DR"; // Data Rate (3.4.4)
const char* const CMD_TXP = "TXP"; // Transmit Power (3.4.12)
const char* const CMD_RSSI = "RSSI"; // RSSI (3.7.1)
const char* const CMD_SNR = "SNR"; // SNR (3.7.2)
const char* const CMD_VERSION = "VER"; // Version (3.7.4)
const char* const CMD_LORA_TX = "TXLRA"; // TX LoRa Test (3.8.1)
const char* const CMD_LORA_RX = "RXLRA"; // RX LoRa Test (3.8.4)
const char* const CMD_LORA_CONFIG = "TCONF"; // Configuration of LoRa Test (3.8.5)
const char* const CMD_LORA_OFF = "TOFF"; // Stop LoRa Test (3.8.6)
const char* const CMD_SAVE = "SAVE"; // Save configuration (3.10.1)
const char* const CMD_AJOIN = "AJOIN"; // Automatic Join (3.10.3)

const char* const RSPNS_OK = "OK";
const char* const RSPNS_ERROR = "AT_ERROR";
const char* const RSPNS_ERROR_BUSY = "AT_BUSY_ERROR";
const char* const RSPNS_ERROR_PARAMETER = "AT_PARAM_ERROR";
const char* const RSPNS_ERROR_PARAMETER_OVERFLOW = "AT_TEST_PARAM_OVERFLOW";
const char* const RSPNS_NO_NETWORK = "AT_NO_NETWORK_JOINED";


// --------------------------------------------------
// Constants

#define SMW_SX1262M0_ADR_OFF  0
#define SMW_SX1262M0_ADR_ON   1

#define SMW_SX1262M0_AUTOMATIC_JOIN_OFF  0
#define SMW_SX1262M0_AUTOMATIC_JOIN_ON   1

#define SMW_SX1262M0_JOIN_MODE_ABP  0
#define SMW_SX1262M0_JOIN_MODE_OTAA 1

#define SMW_SX1262M0_JOIN_STATUS_NOT_JOINED 0
#define SMW_SX1262M0_JOIN_STATUS_JOINED     1

enum class CommandAction : uint8_t { RUN , GET , SET , HELP };
enum class CommandResponse : uint8_t { OK , ERROR , BUSY , NO_NETWORK , DATA };


// --------------------------------------------------
// Helper Constants

#define SMW_SX1262M0_SIZE_APPEUI    16
#define SMW_SX1262M0_SIZE_APPKEY    32
#define SMW_SX1262M0_SIZE_APPSKEY   32
#define SMW_SX1262M0_SIZE_DEVEUI    16
#define SMW_SX1262M0_SIZE_DEVADDR    8
#define SMW_SX1262M0_SIZE_NWKSKEY   32
#define SMW_SX1262M0_SIZE_VERSION    3


// --------------------------------------------------
// Class

class SMW_SX1262M0 {
  public:
    SMW_SX1262M0(Stream (&));
    void flush(void);
    CommandResponse get_ADR(uint8_t (&));
    CommandResponse get_AJoin(uint8_t (&));
    CommandResponse get_AppEUI(char (&)[SMW_SX1262M0_SIZE_APPEUI]);
    CommandResponse get_AppKey(char (&)[SMW_SX1262M0_SIZE_APPKEY]);
    CommandResponse get_AppSKey(char (&)[SMW_SX1262M0_SIZE_APPSKEY]);
    void get_buffer(Buffer (&));
    CommandResponse get_DevAddr(char (&)[SMW_SX1262M0_SIZE_DEVADDR]);
    CommandResponse get_DevEUI(char (&)[SMW_SX1262M0_SIZE_DEVEUI]);
    CommandResponse get_DR(uint8_t (&));
    CommandResponse get_JoinMode(uint8_t (&));
    CommandResponse get_JoinStatus(uint8_t (&));
    CommandResponse get_NwkSKey(char (&)[SMW_SX1262M0_SIZE_NWKSKEY]);
    CommandResponse get_RSSI(float (&));
    CommandResponse get_SNR(float (&));
    CommandResponse get_Version(uint8_t (&)[SMW_SX1262M0_SIZE_VERSION]);
    bool isConnected(void);
    CommandResponse join(void);
    CommandResponse P2P_listen(uint32_t, Buffer (&));
    CommandResponse P2P_listen(uint32_t, Buffer (&), float (&), float (&));
    CommandResponse P2P_start(uint32_t = 915200, bool = false, const char * = nullptr);
    CommandResponse P2P_stop(void);
    CommandResponse ping(void);
    CommandResponse readT(void);
    CommandResponse readT(Buffer (&));
    CommandResponse readT(uint8_t (&), Buffer (&));
    CommandResponse readX(void);
    CommandResponse readX(Buffer (&));
    CommandResponse readX(uint8_t (&), Buffer (&));
    CommandResponse reset(void);
    CommandResponse save(void);
    CommandResponse sendT(uint8_t, const char *);
    CommandResponse sendT(uint8_t, const String);
    CommandResponse sendX(uint8_t, const char *);
    CommandResponse sendX(uint8_t, const String);
    CommandResponse set_ADR(uint8_t);
    CommandResponse set_AJoin(uint8_t);
    CommandResponse set_AppEUI(const char *);
    CommandResponse set_AppKey(const char *);
    CommandResponse set_AppSKey(const char *);
    CommandResponse set_DevAddr(const char *);
    CommandResponse set_DR(uint8_t);
    CommandResponse set_JoinMode(uint8_t);
    CommandResponse set_NwkSKey(const char *);

#ifdef SMW_SX1262M0_DEBUG
    void set_debugger(Stream *);
#endif

  private:
    Stream* _stream;
    Buffer _buffer;
    
#ifdef SMW_SX1262M0_DEBUG
    Stream* _stream_debug;
#endif

    void _delay(uint32_t);
    CommandResponse _read_response(uint32_t);
    void _send_command(const char *,CommandAction, uint8_t = 0, ...);
};

// --------------------------------------------------
// --------------------------------------------------

#define FILTER_PRINTABLE    0
#define FILTER_ALPHANUMERIC 1
#define FILTER_ALPHA        2
#define FILTER_HEX          3
#define FILTER_NUMERIC      4

void filter_string(char *, uint8_t, const char *, uint8_t = FILTER_ALPHANUMERIC);

// --------------------------------------------------

void * memmem(const void *, size_t, const void *, size_t);

// --------------------------------------------------

#endif // SMW_SX1262M0_H
