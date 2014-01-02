/*
    Mudbus.h - an Arduino library for a Modbus TCP slave.
    Copyright (C) 2011  Dee Wykoff

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//#define MbDebug

#include "Arduino.h"

#include <SPI.h>
#include <Ethernet.h>

#ifndef Mudbus_h
#define Mudbus_h

//If a request is made which is out of the below allocated range, no response is sent.
//The device handles it as a timeout, which has not caused any issues in testing.
//Error Checking added Jan 2014 - Andrew Frahn / Emmertex
//Fix ported over from https://github.com/emmertex/Modbus-Library

#define MB_N_C_0x 100 //Max coils for Modbus is 100 due to limited memory
#define MB_N_I_1x 100 //Max inputs for Modbus is 100 due to limited memory
#define MB_N_IR_3x 64 //Max 16 bit input registers is 64 due to limited memory
#define MB_N_HR_4x 64 //Max 16 bit holding registers is 64 due to limited memory
#define MB_PORT 502

enum MB_FC {
    MB_FC_NONE                        = 0,
    MB_FC_READ_COILS_0x               = 1,
    MB_FC_READ_INPUTS_1x              = 2,
    MB_FC_READ_REGISTERS_4x           = 3,
    MB_FC_READ_INPUT_REGISTERS_3x     = 4,
    MB_FC_WRITE_COIL_0x               = 5,
    MB_FC_WRITE_REGISTER_4x           = 6,
    MB_FC_WRITE_MULTIPLE_COILS_0x     = 15,
    MB_FC_WRITE_MULTIPLE_REGISTERS_4x = 16
};

class Mudbus
{
public:
    Mudbus();
    void Run();
    bool C[MB_N_C_0x];
    bool I[MB_N_I_1x];
    int  IR[MB_N_IR_3x];
    int  R[MB_N_HR_4x];
    bool Active, JustReceivedOne;
    unsigned long PreviousActivityTime;
    int Runs, Reads, Writes, TotalMessageLength, MessageStart, NoOfBytesToSend;
private:
    uint8_t ByteReceiveArray[160];
    uint8_t ByteSendArray[160];
    uint8_t SaveArray[160];
    MB_FC FC;
    void SetFC(int fc);
    void PopulateSendBuffer(uint8_t *SendBuffer, int NoOfBytes);
    void buffer_restore();
    void buffer_save();
};

#endif

/* Speculations on Modbus message structure:
**********************************************
**********Master(PC) request frames***********
00 ID high              0
01 ID low               1
02 Protocol high        0
03 Protocol low         0
04 Message length high  0
05 Message length low   6 (6 bytes after this)
06 Slave number         1
07 Function code
08 Start address high   maybe 0
09 Start address low    maybe 0
10 Length high          maybe 125 or Data high if write
11 Length low           maybe 125 or Data low if write
**********************************************
**********Slave(Arduino) response frames******
00 ID high              echo  /           0
01 ID low               echo  /  slave ID 1
02 Protocol high        echo
03 Protocol low         echo
04 Message length high  echo
05 Message length low   num bytes after this
06 Slave number         echo
07 Function code        echo
08 Start address high   num bytes of data
09 Data high
10 Data low
**********************************************
*/
