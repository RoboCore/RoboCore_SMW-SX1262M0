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

// --------------------------------------------------
// Libraries

#include "RoboCore_SMW_SX1262M0.h"

extern "C" {
  #include <string.h>
}

// --------------------------------------------------
// --------------------------------------------------

// Default constructor
//  @param (stream) : the stream to send the data to [Stream *]
SMW_SX1262M0::SMW_SX1262M0(Stream &stream) :
  _stream(&stream),
  _buffer(SMW_SX1262M0_BUFFER_SIZE)
  {
#ifdef SMW_SX1262M0_DEBUG
    _stream_debug = nullptr;
#endif
}

// --------------------------------------------------
// --------------------------------------------------

// Flush the buffered data in the stream
void SMW_SX1262M0::flush(void){
  while(_stream->available()){
    _stream->read();
  }
}

// --------------------------------------------------

// Get the Adaptive Data Rate
//  @param (adr) : the variable to store the result [uint8_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_ADR(uint8_t (&adr)){
  // send the command and read the response
  _send_command(CMD_ADR, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      adr = _buffer.read() - '0';
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Automatic Join
//  @param (ajoin) : the variable to store the result [uint8_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_AJoin(uint8_t (&ajoin)){
  // send the command and read the response
  _send_command(CMD_AJOIN, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      ajoin = _buffer.read() - '0';
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Application EUI
//  @param (appeui) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_AppEUI(char (&appeui)[SMW_SX1262M0_SIZE_APPEUI]){
  // send the command and read the response
  _send_command(CMD_APPEUI, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    uint8_t index = 0;
    for(uint8_t i=0 ; i < SMW_SX1262M0_SIZE_APPEUI ; i++){
      if(isxdigit(_buffer[index])){
        appeui[i] = _buffer[index++];
      } else {
        index++; // go the next
        i--; // repeat the loop with the same index
      }

      // check for end of buffer
      if(index >= length){
        break;
      }
    }
    if(length < SMW_SX1262M0_SIZE_APPEUI){
      appeui[length] = CHAR_EOS;
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Application Key
//  @param (appkey) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_AppKey(char (&appkey)[SMW_SX1262M0_SIZE_APPKEY]){
  // send the command and read the response
  _send_command(CMD_APPKEY, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    uint8_t index = 0;
    for(uint8_t i=0 ; i < SMW_SX1262M0_SIZE_APPKEY ; i++){
      if(isxdigit(_buffer[index])){
        appkey[i] = _buffer[index++];
      } else {
        index++; // go the next
        i--; // repeat the loop with the same index
      }

      // check for end of buffer
      if(index >= length){
        break;
      }
    }
    if(length < SMW_SX1262M0_SIZE_APPKEY){
      appkey[length] = CHAR_EOS;
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Application Session Key
//  @param (appskey) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_AppSKey(char (&appskey)[SMW_SX1262M0_SIZE_APPSKEY]){
  // send the command and read the response
  _send_command(CMD_APPSKEY, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    uint8_t index = 0;
    for(uint8_t i=0 ; i < SMW_SX1262M0_SIZE_APPSKEY ; i++){
      if(isxdigit(_buffer[index])){
        appskey[i] = _buffer[index++];
      } else {
        index++; // go the next
        i--; // repeat the loop with the same index
      }

      // check for end of buffer
      if(index >= length){
        break;
      }
    }
    if(length < SMW_SX1262M0_SIZE_APPSKEY){
      appskey[length] = CHAR_EOS;
    }
  }

  return res;
}

// --------------------------------------------------

// Get the buffered data
//  @param (buffer) : the variable to store the result [Buffer(&)]
void SMW_SX1262M0::get_buffer(Buffer (&buffer)){
  buffer = _buffer;
}

// --------------------------------------------------

// Get the Device Address
//  @param (devaddr) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_DevAddr(char (&devaddr)[SMW_SX1262M0_SIZE_DEVADDR]){
  // send the command and read the response
  _send_command(CMD_DADDR, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    uint8_t index = 0;
    for(uint8_t i=0 ; i < SMW_SX1262M0_SIZE_DEVADDR ; i++){
      if(isxdigit(_buffer[index])){
        devaddr[i] = _buffer[index++];
      } else {
        index++; // go the next
        i--; // repeat the loop with the same index
      }

      // check for end of buffer
      if(index >= length){
        break;
      }
    }
    if(length < SMW_SX1262M0_SIZE_DEVADDR){
      devaddr[length] = CHAR_EOS;
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Device EUI
//  @param (deveui) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_DevEUI(char (&deveui)[SMW_SX1262M0_SIZE_DEVEUI]){
  // send the command and read the response
  _send_command(CMD_DEVEUI, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    uint8_t index = 0;
    for(uint8_t i=0 ; i < SMW_SX1262M0_SIZE_DEVEUI ; i++){
      if(isxdigit(_buffer[index])){
        deveui[i] = _buffer[index++];
      } else {
        index++; // go the next
        i--; // repeat the loop with the same index
      }

      // check for end of buffer
      if(index >= length){
        break;
      }
    }
    if(length < SMW_SX1262M0_SIZE_DEVEUI){
      deveui[length] = CHAR_EOS;
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Data Rate
//  @param (dr) : the variable to store the result [uint8_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_DR(uint8_t (&dr)){
  // send the command and read the response
  _send_command(CMD_DR, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      dr = _buffer.read() - '0';
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Join Mode
//  @param (mode) : the variable to store the result [uint8_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_JoinMode(uint8_t (&mode)){
  // send the command and read the response
  _send_command(CMD_NJM, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      mode = _buffer.read() - '0';
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Join Status
//  @param (status) : the variable to store the result [uint8_t (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_JoinStatus(uint8_t (&status)){
  // send the command and read the response
  _send_command(CMD_NJS, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    if(_buffer.available()){
      uint8_t b = _buffer.read();
      status = b - '0';

//      _connected = (b == '1') ? true : false; // update (ASCII of SMW_SX1262M0_JOIN_STATUS_JOINED)
    }
  }

  return res;
}

// --------------------------------------------------

// Get the Network Session Key
//  @param (nwkskey) : the array to store the result [char[n]]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_NwkSKey(char (&nwkskey)[SMW_SX1262M0_SIZE_NWKSKEY]){
  // send the command and read the response
  _send_command(CMD_NWKSKEY, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    uint8_t length = _buffer.available();
    uint8_t index = 0;
    for(uint8_t i=0 ; i < SMW_SX1262M0_SIZE_NWKSKEY ; i++){
      if(isxdigit(_buffer[index])){
        nwkskey[i] = _buffer[index++];
      } else {
        index++; // go the next
        i--; // repeat the loop with the same index
      }

      // check for end of buffer
      if(index >= length){
        break;
      }
    }
    if(length < SMW_SX1262M0_SIZE_NWKSKEY){
      nwkskey[length] = CHAR_EOS;
    }
  }

  return res;
}

// --------------------------------------------------

// Get the RSSI of the last received data
//  @param (rssi) : the variable to store the result [float (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_RSSI(float (&rssi)){
  // send the command and read the response
  _send_command(CMD_RSSI, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    char buffer_value[6] = { '0' }; // default value is 0
    uint8_t index = 0;
    while(_buffer.available()){
      char c = _buffer.read();
      if(isdigit(c) || (c == '-')){
        buffer_value[index++] = c;
      }
    }
    rssi = atof(buffer_value);
  }

  return res;
}

// --------------------------------------------------

// Get the SNR of the last received data
//  @param (snr) : the variable to store the result [float (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::get_SNR(float (&snr)){
  // send the command and read the response
  _send_command(CMD_SNR, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    char buffer_value[6] = { '0' }; // default value is 0
    uint8_t index = 0;
    while(_buffer.available()){
      char c = _buffer.read();
      if(isdigit(c) || (c == '-')){
        buffer_value[index++] = c;
      }
    }
    snr = atof(buffer_value);
  }

  return res;
}

// --------------------------------------------------

// Get the Version
//  @param (version) : the array to store the result [uint8_t[n]]
//  @returns the type of the response [CommandResponse]
//  NOTE: currently tested only with "v1.2 build 38".
CommandResponse SMW_SX1262M0::get_Version(uint8_t (&version)[SMW_SX1262M0_SIZE_VERSION]){
  // send the command and read the response
  _send_command(CMD_VERSION, CommandAction::GET);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_READ);
  
#ifdef SMW_SX1262M0_DEBUG
  _buffer.print(_stream_debug);
#endif

  if(res == CommandResponse::OK){
    // copy the buffer
    uint8_t length = _buffer.available();
    uint8_t data[length];
    _buffer.copy(data);

    // reset the parameter
    for(uint8_t i=0 ; i < SMW_SX1262M0_SIZE_VERSION ; i++){
      version[i] = 0;
    }

    // define the substrings to find
    const char* STR_MAIN = "SX1262";
    const char* STR_BUILD = "Build";

    // check for STR_MAIN
    void *ptr = memmem(data, length, STR_MAIN, strlen(STR_MAIN));
    if(ptr){
      uint8_t index = (ptrdiff_t)ptr - (ptrdiff_t)data;
      index += strlen(STR_MAIN) + 2; // +"_V"
      uint8_t vindex = 0;
      while(index < length){
        if(isdigit(data[index])){
          version[vindex] *= 10;
          version[vindex] += data[index] - '0';
        } else if(data[index] == '.'){
          vindex++; // update
        } else {
          break; // not supported, exit the loop
        }
        
        index++; // update
      }
    }

    // check for STR_BUILD
    ptr = memmem(data, length, STR_BUILD, strlen(STR_BUILD));
    if(ptr){
      uint8_t index = (ptrdiff_t)ptr - (ptrdiff_t)data;
      index += strlen(STR_BUILD) + 1; // +Space
      uint8_t vindex = SMW_SX1262M0_SIZE_VERSION - 1;
      while(index < length){
        if(isdigit(data[index])){
          version[vindex] *= 10;
          version[vindex] += data[index] - '0';
        } else {
          break; // not supported, exit the loop
        }
        
        index++; // update
      }
    }
  }

  return res;
}

// --------------------------------------------------

// Check if the module is connected to the network
//  @returns true if the device is connected [bool]
//  NOTE: it is a wrapper around the <get_JoinStatus()> command
bool SMW_SX1262M0::isConnected(void){
  uint8_t status = SMW_SX1262M0_JOIN_STATUS_NOT_JOINED; // default
  if(get_JoinStatus(status) == CommandResponse::OK){
    if(status == SMW_SX1262M0_JOIN_STATUS_JOINED){
      return true;
    }
  }
  return false; // default
}

// --------------------------------------------------

// Join the network
//  @returns the type of the response [CommandResponse]
//  NOTE: the confirmation is asynchronous (<get_JoinStatus()>)
CommandResponse SMW_SX1262M0::join(void){
  _send_command(CMD_JOIN, CommandAction::RUN);
  return _read_response(SMW_SX1262M0_TIMEOUT_READ);
}

// --------------------------------------------------

// Listen for incoming data in the P2P communication (LoRa Test)
//  @param (timeout) : the time to wait, in [ms] [uint32_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::P2P_listen(uint32_t timeout, Buffer (&buffer)){
  float rssi, snr;
  return P2P_listen(timeout, buffer, rssi, snr);
}

// --------------------------------------------------

// Listen for incoming data in the P2P communication (LoRa Test)
//  @param (timeout) : the time to wait, in [ms] [uint32_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::P2P_listen(uint32_t timeout, Buffer (&buffer), float (&rssi), float (&snr)){
  CommandResponse res = CommandResponse::OK; // default

  const char *str_rssi = "RSSI=";
  const char *str_snr = "SNR=";
  const char *str_data = "-> ";
  
  uint8_t index_rssi[] = {0,0}; // { max , current }
  index_rssi[0] = strlen(str_rssi); // separated to avoid narrowing conversion warning
  index_rssi[0] -= 1;
  uint8_t index_snr[] = {0,0}; // { max , current }
  index_snr[0] = strlen(str_snr); // separated to avoid narrowing conversion warning
  index_snr[0] -= 1;
  uint8_t index_data[] = {0,0}; // { max , current }
  index_data[0] = strlen(str_data); // separated to avoid narrowing conversion warning
  index_data[0] -= 1;

  // assign default values
  rssi = 0;
  snr = 0;

  uint8_t b;
  char str[5];
  uint8_t index_str = 0;
  enum { NOTHING , RSSI , SNR , DATA };
  uint8_t store = NOTHING;
  uint32_t stop_time = millis() + timeout;
  while(millis() < stop_time){
    if(_stream->available()){
      b = _stream->read();
      
      // store
      switch(store){
        case RSSI: {
          if((b == '-') || isdigit(b)){
            if(index_str < 4){
              str[index_str++] = b; // store
            }
          } else { // end of value
            str[index_str] = CHAR_EOS;
            rssi = atoi(str);
            store = NOTHING; // reset
          }
          break;
        }
        
        case SNR: {
          if((b == '-') || isdigit(b)){
            if(index_str < 4){
              str[index_str++] = b; // store
            }
          } else { // end of value
            str[index_str] = CHAR_EOS;
            snr = atoi(str);
            store = NOTHING; // reset
          }
          break;
        }
        
        case DATA: {
          if(b >= CHAR_SPACE){
            _buffer.append(b); // store
          } else { // end of text
            buffer = _buffer;
            res = CommandResponse::DATA;

            // flush the data
            while((_stream->peek() == CHAR_LF) || (_stream->peek() == CHAR_CR)){
              _stream->read();
            }

            // exit the timing loop
            stop_time = 0;
          }
          break;
        }
        
        default: {
          // do nothing
          break;
        }
      }

      // check RSSI
      if(store != DATA){
        if(b == str_rssi[index_rssi[1]]){
          // check the length
          if(index_rssi[1] == index_rssi[0]){
            store = RSSI;
            index_str = 0; // reset
          } else {
            index_rssi[1]++; // update
          }
        } else {
          index_rssi[1] = 0; // reset
        }
      }
      
      // check SNR
      if(store != DATA){
        if(b == str_snr[index_snr[1]]){
          // check the length
          if(index_snr[1] == index_snr[0]){
            store = SNR;
            index_str = 0; // reset
          } else {
            index_snr[1]++; // update
          }
        } else {
          index_snr[1] = 0; // reset
        }
      }
      
      // check Data
      if(store != DATA){
        if(b == str_data[index_data[1]]){
          // check the length
          if(index_data[1] == index_data[0]){
            store = DATA;
            _buffer.reset(); // reset
          } else {
            index_data[1]++; // update
          }
        } else {
          index_data[1] = 0; // reset
        }
      }
    }
  }

  return res;
}

// --------------------------------------------------

// Start the P2P communication (LoRa Test)
//  @param (frequency) : the frequency to use for the wireless communication, in kHz [uint32_t]
//         (continuous) : TRUE to make the communication persistent [bool]
//         (data) : the data to send [char *]. If this parameter is omitted, the module will enter Rx mode instead of Tx.
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::P2P_start(uint32_t frequency, bool continuous, const char *data){
  // check the frequency
  if((frequency < 902000) || (frequency > 927800)){
    return CommandResponse::ERROR;
  }
  if((frequency > 907400) && (frequency < 915200)){
    return CommandResponse::ERROR;
  }

  // convert the frequency
  char sfreq[7];
  sfreq[6] = CHAR_EOS;
  for(int8_t i=5 ; i >=0 ; i--){
    sfreq[i] = frequency % 10;
    sfreq[i] += '0';
    frequency /= 10;
  }

  // convert the mode
  char mode[2];
  mode[0] = (continuous) ? '1' : '0';
  mode[1] = CHAR_EOS;

  // check the data
  if(data == nullptr){
    _send_command(CMD_LORA_RX, CommandAction::SET, 2, sfreq, mode);
  } else {
    _send_command(CMD_LORA_TX, CommandAction::SET, 3, sfreq, mode, data);
  }
  
  return _read_response(SMW_SX1262M0_TIMEOUT_READ);
}

// --------------------------------------------------

// Stop the P2P communication (LoRa Test)
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::P2P_stop(void){
  // flush the serial buffer
  // NOTE: the serial buffer might have some data because of the P2P output (for Tx and Rx).
  if(_stream->available()){
    _stream->read(); // read the incoming byte
  }
  
  _send_command(CMD_LORA_OFF, CommandAction::RUN);
  return _read_response(SMW_SX1262M0_TIMEOUT_READ);
}

// --------------------------------------------------

// Ping the module
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::ping(void){
  _send_command(nullptr, CommandAction::RUN); // the command already sends "AT"
  return _read_response(SMW_SX1262M0_TIMEOUT_READ);
}

// --------------------------------------------------

// Read a text message from the module
//  @returns the type of the response [CommandResponse]
//  NOTE: the data must be obtained from the buffer
CommandResponse SMW_SX1262M0::readT(void){
  // send the command and read the response
  _send_command(CMD_RECV, CommandAction::GET);
  return _read_response(SMW_SX1262M0_TIMEOUT_READ);
}

// --------------------------------------------------

// Read a text message from the module
//  @param (buffer) : the buffer to store the payload [Buffer (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::readT(Buffer (&buffer)){
  uint8_t port;
  return readT(port, buffer);
}

// --------------------------------------------------

// Read a text message from the module
//  @param (port) : the application port [uint8_t (&)]
//         (buffer) : the buffer to store the payload [Buffer (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::readT(uint8_t (&port), Buffer (&buffer)){
  CommandResponse res = readT(); // read the message

  // parse the message
  uint8_t b;
  bool payload = false;
  char sport[4]; // 0 to 999
  uint8_t index = 0;
  while (_buffer.available()){
    b = _buffer.read();

    // check for delimitter
    if(b == CHAR_COLON){
      payload = true; // set
      buffer.resize(_buffer.available()); // resize the buffer
      continue;
    }

    // parse
    if(!payload){
      if((index < 3) && isdigit(b)){
        sport[index++] = b;
        sport[index] = CHAR_EOS;
      }
    } else {
      buffer.append(b);
    }
  }
  port = atoi(sport); // convert

  return res;
}

// --------------------------------------------------

// Read an hexadecimal message from the module
//  @returns the type of the response [CommandResponse]
//  NOTE: the data must be obtained from the buffer
CommandResponse SMW_SX1262M0::readX(void){
  // send the command and read the response
  _send_command(CMD_RECVB, CommandAction::GET);
  return _read_response(SMW_SX1262M0_TIMEOUT_READ);
}
// --------------------------------------------------

// Read an hexadecimal message from the module
//  @param (buffer) : the buffer to store the payload [Buffer (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::readX(Buffer (&buffer)){
  uint8_t port;
  return readX(port, buffer);
}

// --------------------------------------------------

// Read an hexadecimal message from the module
//  @param (port) : the application port [uint8_t (&)]
//         (buffer) : the buffer to store the payload [Buffer (&)]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::readX(uint8_t (&port), Buffer (&buffer)){
  CommandResponse res = readX(); // read the message

  // parse the message
  uint8_t b;
  bool payload = false;
  char sport[4]; // 0 to 999
  uint8_t index = 0;
  while (_buffer.available()){
    b = _buffer.read();

    // check for delimitter
    if(b == CHAR_COLON){
      payload = true; // set
      buffer.resize(_buffer.available()); // resize the buffer
      continue;
    }

    // parse
    if(!payload){
      if((index < 3) && isdigit(b)){
        sport[index++] = b;
        sport[index] = CHAR_EOS;
      }
    } else {
      buffer.append(b);
    }
  }
  port = atoi(sport); // convert

  return res;
}

// --------------------------------------------------

// Reset the module
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::reset(void){
  // do a software reset
#ifdef SMW_SX1262M0_DEBUG
  if(_stream_debug){
    _stream_debug->write('[');
    _stream_debug->write(CMD_RESET);
    _stream_debug->write(']');
  }
#endif
  _stream->write(CMD_RESET);
  _stream->write(CHAR_CR);

//  _reset = false; // reset
  CommandResponse res = CommandResponse::ERROR; // default
  
  // read the incoming data
  uint8_t c;
  const char* const STR_TO_FIND = "ATtention";
  uint32_t stop_time = millis() + SMW_SX1262M0_TIMEOUT_RESET;
  while(millis() < stop_time){
    if(_stream->available()){
      c = _stream->read(); // read the incoming byte
      
#ifdef SMW_SX1262M0_DEBUG
      // debug
      if(_stream_debug){
        if(c > 32){
          _stream_debug->write(c);
        } else {
          _stream_debug->print('(');
          _stream_debug->print(c, HEX);
          _stream_debug->print(')');
        }
      }
#endif

      // check if already found
      if(res != CommandResponse::OK){
        // append the character or seach the string
        if((c > 31) && (c < 127)){
          _buffer.append(c);
        } else if((c == CHAR_CR) || (c == CHAR_LF)){
            // get a copy of the buffer
            uint8_t data_length = _buffer.available();
            uint8_t data[data_length];
            _buffer.copy(data);
            
            // search for the string
            void *ptr = memmem(data, data_length, STR_TO_FIND, strlen(STR_TO_FIND));
            if(ptr){
              if(data_length > strlen(STR_TO_FIND)){
                res = CommandResponse::OK; // set
              }
            }

            _buffer.reset(); // reset the buffer
        }
      }
      
    } else {
      _delay(SMW_SX1262M0_DELAY_INCOMING_DATA); // give some time for data to arrive
    }
  }

  return res;
}

// --------------------------------------------------

// Save the current configuration
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::save(void){
  _send_command(CMD_SAVE, CommandAction::RUN);
  return _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes some time to reply
}

// --------------------------------------------------

// Send a text message
//  @param (port) : the application port [uint8_t]
//         (data) : the text data to send [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::sendT(uint8_t port, const char *data){
  // parse the port
  uint8_t aport = port; // auxiliary variable for <port>
  uint8_t temp[3];
  temp[0] = aport / 100;
  aport %= 100;
  temp[1] = aport / 10;
  temp[2] = aport % 10;

  // set the header (port)
  char sport[4]; // port stringified (0 to 999)
  uint8_t index = 0;
  aport = 0; // reset
  for(uint8_t i=0 ; i < 3 ; i++){
    if((temp[i] > 0) || (aport > 0)){
      sport[index++] = temp[i] + '0'; // convert to ASCII character
    }
    aport += temp[i]; // update (simple)
  }
  sport[index] = CHAR_EOS;
  
  // send the command and read the response
  _send_command(CMD_SEND, CommandAction::SET, 2, sport, data);
  return _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes some time to reply
}

// --------------------------------------------------

// Send a text message
//  @param (port) : the application port [uint8_t]
//         (data) : the text data to send [String]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::sendT(uint8_t port, const String data){
  uint8_t length = data.length() + 1;
  char cdata[length]; // create a temporary string
  data.toCharArray(cdata, length); // copy the data (with automatic EOS)
  return sendT(port, cdata);
}

// --------------------------------------------------

// Send an hexadecimal message
//  @param (port) : the application port [uint8_t]
//         (data) : the text data to send [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::sendX(uint8_t port, const char *data){
  // parse the port
  uint8_t aport = port; // auxiliary variable for <port>
  uint8_t temp[3];
  temp[0] = aport / 100;
  aport %= 100;
  temp[1] = aport / 10;
  temp[2] = aport % 10;

  // set the header (port)
  uint8_t sport[4]; // port stringified (0 to 999)
  uint8_t index = 0;
  aport = 0; // reset
  for(uint8_t i=0 ; i < 3 ; i++){
    if((temp[i] > 0) || (aport > 0)){
      sport[index++] = temp[i] + '0'; // convert to ASCII character
    }
    aport += temp[i]; // update (simple)
  }
  sport[index] = CHAR_EOS;
  
  // send the command and read the response
  _send_command(CMD_SENDB, CommandAction::SET, 2, sport, data);
  return _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes some time to reply
}

// --------------------------------------------------

// Send an hexadecimal message
//  @param (port) : the application port [uint8_t]
//         (data) : the text data to send [String]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::sendX(uint8_t port, const String data){
  uint8_t length = data.length() + 1;
  char cdata[length]; // create a temporary string
  data.toCharArray(cdata, length); // copy the data (with automatic EOS)
  return sendX(port, cdata);
}

// --------------------------------------------------

// Set the Adaptive Data Rate
//  @param (adr) : the data to be sent [uint8_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::set_ADR(uint8_t adr){
  adr = (adr == SMW_SX1262M0_ADR_ON) ? SMW_SX1262M0_ADR_ON : SMW_SX1262M0_ADR_OFF; // force binary value
  adr += '0'; // convert to ASCII character, without a narrowing conversion
  unsigned char data[] = { adr , CHAR_EOS};
  
  // send the command and read the response
  _send_command(CMD_ADR, CommandAction::SET, 1, data);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes almost X s to reply

  return res;
}

// --------------------------------------------------

// Set the Automatic Join
//  @param (ajoin) : the data to be sent [uint8_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::set_AJoin(uint8_t ajoin){
  ajoin = (ajoin == SMW_SX1262M0_AUTOMATIC_JOIN_ON) ? SMW_SX1262M0_AUTOMATIC_JOIN_ON : SMW_SX1262M0_AUTOMATIC_JOIN_OFF; // force binary value
  ajoin += '0'; // convert to ASCII character, without a narrowing conversion
  unsigned char data[] = { ajoin , CHAR_EOS};
  
  // send the command and read the response
  _send_command(CMD_AJOIN, CommandAction::SET, 1, data);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes almost X s to reply

  return res;
}

// --------------------------------------------------

// Set the Application EUI
//  @param (appeui) : the array with the data to be sent [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::set_AppEUI(const char *appeui){
  // filter the data
  uint8_t length = SMW_SX1262M0_SIZE_APPEUI + 8; // +1 for EOS and +7 for ':'
  char str[length];
  str[length - 1] = CHAR_EOS;
  filter_string(str, SMW_SX1262M0_SIZE_APPEUI, appeui, FILTER_HEX);

  // format the string ("xx:xx:xx:xx:xx:xx:xx:xx")
  for(uint8_t i=2 ; i < (length - 1) ; i += 3){ // ignore EOS
    // shift the data (start from the end)
    for(uint8_t j=(length - 2) ; j > i ; j--){ // ignore EOS
      str[j] = str[j-1];
    }

    str[i] = CHAR_COLON; // insert the colon
  }
  
  // send the command and read the response
  _send_command(CMD_APPEUI, CommandAction::SET, 1, str);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes almost X s to reply

  return res;
}

// --------------------------------------------------

// Set the Application Key
//  @param (appkey) : the array with the data to be sent [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::set_AppKey(const char *appkey){
  // filter the data
  uint8_t length = SMW_SX1262M0_SIZE_APPKEY + 16; // +1 for EOS and +15 for ':'
  char str[length];
  str[length - 1] = CHAR_EOS;
  filter_string(str, SMW_SX1262M0_SIZE_APPKEY, appkey, FILTER_HEX);

  // format the string ("xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx")
  for(uint8_t i=2 ; i < (length - 1) ; i += 3){ // ignore EOS
    // shift the data (start from the end)
    for(uint8_t j=(length - 2) ; j > i ; j--){ // ignore EOS
      str[j] = str[j-1];
    }

    str[i] = CHAR_COLON; // insert the colon
  }
  
  // send the command and read the response
  _send_command(CMD_APPKEY, CommandAction::SET, 1, str);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes almost X s to reply

  return res;
}

// --------------------------------------------------

// Set the Application Session Key
//  @param (appskey) : the array with the data to be sent [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::set_AppSKey(const char *appskey){
  // filter the data
  uint8_t length = SMW_SX1262M0_SIZE_APPSKEY + 16; // +1 for EOS and +15 for ':'
  char str[length];
  str[length - 1] = CHAR_EOS;
  filter_string(str, SMW_SX1262M0_SIZE_APPSKEY, appskey, FILTER_HEX);

  // format the string ("xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx")
  for(uint8_t i=2 ; i < (length - 1) ; i += 3){ // ignore EOS
    // shift the data (start from the end)
    for(uint8_t j=(length - 2) ; j > i ; j--){ // ignore EOS
      str[j] = str[j-1];
    }

    str[i] = CHAR_COLON; // insert the colon
  }
  
  // send the command and read the response
  _send_command(CMD_APPSKEY, CommandAction::SET, 1, str);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes almost X s to reply

  return res;
}

// --------------------------------------------------

// Set the debugger of the object
//  @param (debugger) : the stream to print to [Stream *]
#ifdef SMW_SX1262M0_DEBUG
void SMW_SX1262M0::set_debugger(Stream *debugger){
  _stream_debug = debugger;
}
#endif

// --------------------------------------------------

// Set the Device Address
//  @param (devaddr) : the array with the data to be sent [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::set_DevAddr(const char *devaddr){
  // filter the data
  uint8_t length = SMW_SX1262M0_SIZE_DEVADDR + 4; // +1 for EOS and +3 for ':'
  char str[length];
  str[length - 1] = CHAR_EOS;
  filter_string(str, SMW_SX1262M0_SIZE_DEVADDR, devaddr, FILTER_HEX);

  // format the string ("xx:xx:xx:xx")
  for(uint8_t i=2 ; i < (length - 1) ; i += 3){ // ignore EOS
    // shift the data (start from the end)
    for(uint8_t j=(length - 2) ; j > i ; j--){ // ignore EOS
      str[j] = str[j-1];
    }

    str[i] = CHAR_COLON; // insert the colon
  }
  
  // send the command and read the response
  _send_command(CMD_DADDR, CommandAction::SET, 1, str);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes almost X s to reply

  return res;
}

// --------------------------------------------------

// Set the Data Rate
//  @param (dr) : the data to be sent [uint8_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::set_DR(uint8_t dr){
  // check the value
  if(dr > 6){
    return CommandResponse::ERROR;
  }

  dr += '0'; // convert to ASCII character, without a narrowing conversion
  unsigned char data[] = { dr , CHAR_EOS};
  
  // send the command and read the response
  _send_command(CMD_DR, CommandAction::SET, 1, data);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes almost X s to reply

  return res;
}

// --------------------------------------------------

// Set the Join Mode
//  @param (mode) : the data to be sent [uint8_t]
//  @returns the type of the response [CommandResponse]
//  NOTE: this command resets the module, but returns "OK" after completion
CommandResponse SMW_SX1262M0::set_JoinMode(uint8_t mode){
  // check the value
  if(mode > SMW_SX1262M0_JOIN_MODE_OTAA){
    return CommandResponse::ERROR;
  }

  mode += '0'; // convert to ASCII character, without a narrowing conversion
  unsigned char data[] = { mode , CHAR_EOS};
  
  // send the command and read the response
  _send_command(CMD_NJM, CommandAction::SET, 1, data);
  
  // read the incoming data
  char c;
  const char* const STR_TO_FIND = "AppKey";
  uint8_t index = 0;
  uint8_t length = strlen(STR_TO_FIND);
  uint32_t stop_time = millis() + SMW_SX1262M0_TIMEOUT_RESET;
  while(millis() < stop_time){
    if(_stream->available()){
      c = _stream->read(); // read the incoming byte
      
#ifdef SMW_SX1262M0_DEBUG
      // debug
      if(_stream_debug){
        if(c > 32){
          _stream_debug->write(c);
        } else {
          _stream_debug->print('(');
          _stream_debug->print(c, HEX);
          _stream_debug->print(')');
        }
      }
#endif

      if(c == STR_TO_FIND[index]){
        index++; // update

        // check for completion
        if(index >= length){
          break;
        }
      } else if((c == 'S') && (index == 3)){ // "AppKey" vs. "AppSKey"
        continue;
      } else {
        index = 0; // reset
      }
      
    } else {
      _delay(SMW_SX1262M0_DELAY_INCOMING_DATA); // give some time for data to arrive
    }
  }
  
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes almost X s to reply
  return res;
}

// --------------------------------------------------

// Set the Network Session Key
//  @param (nwkskey) : the array with the data to be sent [char *]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::set_NwkSKey(const char *nwkskey){
  // filter the data
  uint8_t length = SMW_SX1262M0_SIZE_NWKSKEY + 16; // +1 for EOS and +15 for ':'
  char str[length];
  str[length - 1] = CHAR_EOS;
  filter_string(str, SMW_SX1262M0_SIZE_NWKSKEY, nwkskey, FILTER_HEX);

  // format the string ("xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx")
  for(uint8_t i=2 ; i < (length - 1) ; i += 3){ // ignore EOS
    // shift the data (start from the end)
    for(uint8_t j=(length - 2) ; j > i ; j--){ // ignore EOS
      str[j] = str[j-1];
    }

    str[i] = CHAR_COLON; // insert the colon
  }
  
  // send the command and read the response
  _send_command(CMD_NWKSKEY, CommandAction::SET, 1, str);
  CommandResponse res = _read_response(SMW_SX1262M0_TIMEOUT_WRITE); // this command takes some time to reply

  return res;
}

// --------------------------------------------------
// --------------------------------------------------

// Custom delay in miliseconds
//  @param (duration) : the duration of the delay in miliseconds [uint32_t]
void SMW_SX1262M0::_delay(uint32_t duration){
  uint32_t stop_time = millis() + duration;
  while(millis() < stop_time){
#if defined(ARDUINO_ESP8266_GENERIC) || defined(ARDUINO_ESP8266_NODEMCU) || defined(ARDUINO_ESP8266_THING) || defined(ARDUINO_ESP32_DEV)
// ESP8266 Generic / NodeMCU / Sparkfun The Thing / ESP32 Dev
    yield(); // custom function for a non blocking execution with the ESP family
#else
    // do nothing
#endif
  }
}

// --------------------------------------------------

// Read the response of a command
//  @param (timeout) : the time to wait for the response in miliseconds [uint32_t]
//  @returns the type of the response [CommandResponse]
CommandResponse SMW_SX1262M0::_read_response(uint32_t timeout){
  _buffer.reset(); // reset for storing the new response
  
  // read the incoming data
  uint8_t c;
  uint32_t stop_time = millis() + timeout;
  while(millis() < stop_time){
    if(_stream->available()){
      c = _stream->read(); // read the incoming byte
      
#ifdef SMW_SX1262M0_DEBUG
      // debug
      if(_stream_debug){
        if(c > 32){
          _stream_debug->write(c);
        } else {
          _stream_debug->print('(');
          _stream_debug->print(c, HEX);
          _stream_debug->print(')');
        }
      }
#endif

      if((c > 31) && (c < 127)){
        _buffer.append(c);
      } else if((c == CHAR_CR) || (c == CHAR_LF)){
        _buffer.append(c);
      }
    } else {
#if defined(ARDUINO_ESP8266_GENERIC) || defined(ARDUINO_ESP8266_NODEMCU) || defined(ARDUINO_ESP8266_THING) || defined(ARDUINO_ESP32_DEV)
// ESP8266 Generic / NodeMCU / Sparkfun The Thing / ESP32 Dev
      yield(); // custom function for a non blocking execution with the ESP family
#else
      // do nothing
#endif
    }
  }

  // get the status of the message
  Buffer buffer_status(25);
  uint8_t buffer_length = _buffer.available();
  if(buffer_length > 4){ // (the status is returned as "<CR><LF>Status<CR><LF>")
    uint8_t status = 0;
    
    for(int16_t i=(buffer_length - 1) ; i >= 0 ; i--){
      c = _buffer[i]; // get the current byte
      
      if((c == CHAR_CR) || (c == CHAR_LF)){
        status++; // update

        // check for the end of the status message
        if(status == 4){
          // get the status and shift the main buffer
          for(uint8_t j=i ; j < buffer_length ; j++){
            c = _buffer[i]; // get the current byte

            // store the byte if valid
            if((c > 31) && (c < 127)){
              buffer_status.append(c);
            }

            _buffer.remove(i); // remove the byte
          }
          
          break; // exit the loop
        }
      }
    }
  }

  // check for a valid buffer
  if(!buffer_status.available()){
    return CommandResponse::ERROR; // wrong result
  }

  // trim the main buffer (if necessary)
  buffer_length = _buffer.available();
  if(buffer_length){
    for(int16_t i=(buffer_length - 1) ; i >= 0 ; i--){
      c = _buffer[i]; // get the current byte
      
      if((c == CHAR_CR) || (c == CHAR_LF)){
        _buffer.remove(i);
      } else {
        break; // exit the loop (remove only the trailing characters)
      }
    }
  }

  // get a copy of the buffer
  uint8_t data_length = buffer_status.available();
  uint8_t data[data_length];
  buffer_status.copy(data);
  
#ifdef SMW_SX1262M0_DEBUG
  // debug
  if(_stream_debug){
    buffer_status.print(_stream_debug);
  }
#endif

  // check for OK
  if(memcmp(data, RSPNS_OK, strlen(RSPNS_OK)) == 0){
    return CommandResponse::OK;
  }

  // check for ERROR
  void *ptr = memmem(data, data_length, RSPNS_ERROR, strlen(RSPNS_ERROR));
  if(ptr){
    if(data_length > strlen(RSPNS_ERROR)){
      return CommandResponse::ERROR;
    }
  }

  // check for ERROR - Parameter
  ptr = memmem(data, data_length, RSPNS_ERROR_PARAMETER, strlen(RSPNS_ERROR_PARAMETER));
  if(ptr){
    if(data_length > strlen(RSPNS_ERROR_PARAMETER)){
      return CommandResponse::ERROR;
    }
  }

  // check for ERROR - Parameter overflow
  ptr = memmem(data, data_length, RSPNS_ERROR_PARAMETER_OVERFLOW, strlen(RSPNS_ERROR_PARAMETER_OVERFLOW));
  if(ptr){
    if(data_length > strlen(RSPNS_ERROR_PARAMETER_OVERFLOW)){
      return CommandResponse::ERROR;
    }
  }

  // check for ERROR - Network busy
  ptr = memmem(data, data_length, RSPNS_ERROR_BUSY, strlen(RSPNS_ERROR_BUSY));
  if(ptr){
    if(data_length > strlen(RSPNS_ERROR_BUSY)){
      return CommandResponse::BUSY;
    }
  }

  // check for NO NETWORK
  ptr = memmem(data, data_length, RSPNS_NO_NETWORK, strlen(RSPNS_NO_NETWORK));
  if(ptr){
    if(data_length > strlen(RSPNS_NO_NETWORK)){
      return CommandResponse::NO_NETWORK;
    }
  }

  return CommandResponse::ERROR; // wrong result
}

// --------------------------------------------------

// Send a command to the module
//  @param (command) : the command to send [char *]
//         (action)  : the type of action for the command [CommandAction]
//         (qty)     : the quantity of other parameters to send [uint8_t]
//         (...)     : optional and variable data to send [char *]
void SMW_SX1262M0::_send_command(const char *command, CommandAction action, uint8_t qty, ...){
  flush(); // flush the data before sendig the command
  // (it could be done in <readResponse()>, but it might flush some data in some cases - not verified)
  
#ifdef SMW_SX1262M0_DEBUG
  if(_stream_debug){
    _stream_debug->write('[');
    _stream_debug->write(CMD_AT);
  }
#endif
  _stream->write(CMD_AT); // send the <AT> prefix
  
  // check if there is another command
  if(command){
    char cmd_action[] = {0,0,0}; // EOS included
    switch(action){
      case CommandAction::RUN: {
        // nothing to do
        break;
      }

      case CommandAction::GET: {
        cmd_action[0] = CHAR_EQUAL;
        cmd_action[1] = CHAR_QUESTION;
        break;
      }

      case CommandAction::SET: {
        cmd_action[0] = CHAR_EQUAL;
        break;
      }

      case CommandAction::HELP:
      default: {
        cmd_action[0] = CHAR_QUESTION;
        break;
      }
    }
#ifdef SMW_SX1262M0_DEBUG
    if(_stream_debug){
      _stream_debug->write(CHAR_PLUS);
      _stream_debug->write(command);
      _stream_debug->write(cmd_action);
    }
#endif
    _stream->write(CHAR_PLUS);
    _stream->write(command);
    _stream->write(cmd_action);

    // check if there are paramenters to send
    if(qty){
      // get the list of parameters
      va_list arg_list;
      va_start(arg_list, qty);

      // write the parameters
      for(uint8_t i=0 ; i < qty ; i++){
        char *data = va_arg(arg_list, char *);
      
#ifdef SMW_SX1262M0_DEBUG
        if(_stream_debug){
          _stream_debug->write(data);
        }
#endif
        _stream->write(data);

        // add the separator if necessary
        if(i < (qty - 1)){
#ifdef SMW_SX1262M0_DEBUG
          if(_stream_debug){
            _stream_debug->write(CHAR_COLON);
          }
#endif
          _stream->write(CHAR_COLON);
        }
      }

      // end using variable argument list
      va_end(arg_list);
    }
  }
  
#ifdef SMW_SX1262M0_DEBUG
  if(_stream_debug){
    _stream_debug->write(CHAR_CR);
    _stream_debug->write(']');
  }
#endif
  _stream->write(CHAR_CR);
}

// --------------------------------------------------
// --------------------------------------------------

// Filter the characters of a string
//  @param (output) : the output string, already initialized [char *]
//         (length) : the length of the output string [uint8_t]
//         (input)  : the input string to filter [char *]
//         (format) : the format of the filter [uint8_t] (default: FILTER_ALPHANUMERIC)
void filter_string(char *output, uint8_t length, const char *input, uint8_t format){
  uint8_t length_input = strlen(input); // get the length of the input

  // copy the input string
  for(uint8_t i=0 ; i < length ; i++){
    output[i] = CHAR_EOS; // <switch:default> doesn't seem to work for out of bounds access
    
    if(i < length_input){
      switch(format){
        case FILTER_PRINTABLE: {
          if(!iscntrl(input[i])){
            output[i] = input[i];
          }
          break;
        }
        
        case FILTER_ALPHANUMERIC: {
          if(isalnum(input[i])){
            output[i] = input[i];
          }
          break;
        }
        
        case FILTER_ALPHA: {
          if(isalpha(input[i])){
            output[i] = input[i];
          }
          break;
        }
        
        case FILTER_HEX: {
          if(isxdigit(input[i])){
            output[i] = input[i];
          }
          break;
        }
        
        case FILTER_NUMERIC: {
          if(isdigit(input[i])){
            output[i] = input[i];
          }
          break;
        }
      }
    }
  }
}

// --------------------------------------------------

// Find the fist occurrence of a block of data in another block of data
//  @param (haystack) : the block of data for the search [void *]
//         (hlen)     : the length of the haystack block [size_t]
//         (needle)   : the data to search for [void *]
//         (nlen)     : the length of the needle block [size_t]
//  @returns the pointer to the beginning of the sub block or a null pointer [void *]
//  NOTE: credits to <caf> ( https://stackoverflow.com/questions/2188914/c-searching-for-a-string-in-a-file )
void * memmem(const void *haystack, size_t hlen, const void *needle, size_t nlen){
  // check for valid needle length
  if (!nlen){
    return nullptr;
  }
  
  // check for valid haystack length
  if(hlen < nlen){
    return nullptr;
  }
  
  const uint8_t *p = (const uint8_t *)haystack; // [void *] cannot be incremented
  const void *p_void; // auxiliary variable for functions
  size_t plen = hlen;
  
  uint8_t needle_first;
  needle_first = *(uint8_t *)needle;

  // search for the first character of the needle
  while ((plen >= nlen) && (p_void = memchr(p, needle_first, (plen - nlen + 1)))){
    if (!memcmp(p_void, needle, nlen)){
      return (void *)p_void;
    }

    // go to the next address
    p = (const uint8_t *)p_void;
    p++;

    size_t addr_p = reinterpret_cast<size_t>(p); // get the address of <p>
    size_t addr_h = reinterpret_cast<size_t>(haystack); // get the address of <haystack>
    plen = hlen - (addr_p - addr_h); // update the length
  }
  
  return nullptr;
}

// --------------------------------------------------
