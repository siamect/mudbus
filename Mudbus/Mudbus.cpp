/*
    Mudbus.cpp - an Arduino library for a Modbus TCP slave.
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
//#define MbRunsDebug
#include "Mudbus.h"

EthernetServer MbServer(MB_PORT);

Mudbus::Mudbus()
{
}

void Mudbus::Run()
{
    Runs = 1 + Runs * (Runs < 999);

    //****************** Read from socket ****************
    EthernetClient client = MbServer.available();
    if(client.available())
    {
        Reads = 1 + Reads * (Reads < 999);
        int i = 0;
        while(client.available())
        {
            ByteReceiveArray[i] = client.read();
            i++;

        }
        TotalMessageLength = i;
        NoOfBytesToSend = 0;
        MessageStart = 0;
#ifdef MbDebug
        for (i=0; i<TotalMessageLength; i++) {
            Serial.print(ByteReceiveArray[i],HEX);
            Serial.print(" ");
        }
        Serial.println(" received");
        Serial.print("MessageLength = ");
        Serial.println(TotalMessageLength);
#endif
        SetFC(ByteReceiveArray[7]);  //Byte 7 of request is FC
        JustReceivedOne = true;
        if(!Active)
        {
            Active = true;
            PreviousActivityTime = millis();
#ifdef MbDebug
            Serial.println("Mb active");
#endif
        }
    }
    if(millis() > (PreviousActivityTime + 60000))
    {
        if(Active)
        {
            Active = false;
#ifdef MbDebug
            Serial.println("Mb not active");
#endif
        }
    }

    int Start, WordDataLength, ByteDataLength, CoilDataLength, MessageLength;

    while (FC != 0) {
        //**1**************** Read Coils **********************
        if(FC == MB_FC_READ_COILS_0x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            CoilDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = CoilDataLength / 8;
            if(ByteDataLength * 8 < CoilDataLength) ByteDataLength++;
            CoilDataLength = ByteDataLength * 8;
#ifdef MbDebug
            Serial.print(" MB_FC_READ_COILS_0x S=");
            Serial.print(Start);
            Serial.print(" L=");
            Serial.println(CoilDataLength);
#endif
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 3; //Number of bytes after this one.
            ByteReceiveArray[8 + MessageStart] = ByteDataLength;     //Number of bytes after this one (or number of bytes of data).
            for(int i = 0; i < ByteDataLength ; i++)
            {
                for(int j = 0; j < 8; j++)
                {
                    bitWrite(ByteReceiveArray[9 + i + MessageStart], j, C[Start + i * 8 + j]);
                }
            }
            MessageLength = ByteDataLength + 9;
            PopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }

        //**2**************** Read descrete Inputs **********************
        else if(FC == MB_FC_READ_INPUTS_1x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            CoilDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = CoilDataLength / 8;
            if(ByteDataLength * 8 < CoilDataLength) ByteDataLength++;
            CoilDataLength = ByteDataLength * 8;
#ifdef MbDebug
            Serial.print(" MB_FC_READ_INPUTS_1x S=");
            Serial.print(Start);
            Serial.print(" L=");
            Serial.println(CoilDataLength);
#endif
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 3; //Number of bytes after this one.
            ByteReceiveArray[8 + MessageStart] = ByteDataLength;     //Number of bytes after this one (or number of bytes of data).
            for(int i = 0; i < ByteDataLength ; i++)
            {
                for(int j = 0; j < 8; j++)
                {
                    bitWrite(ByteReceiveArray[9 + i + MessageStart], j, I[Start + i * 8 + j]);
                }
            }
            MessageLength = ByteDataLength + 9;
            PopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }

        //**3**************** Read Holding Registers ******************
        else if(FC == MB_FC_READ_REGISTERS_4x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            WordDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = WordDataLength * 2;
#ifdef MbDebug
            Serial.print(" MB_FC_READ_REGISTERS_4x S=");
            Serial.print(Start);
            Serial.print(" L=");
            Serial.println(WordDataLength);
#endif
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 3; //Number of bytes after this one.
            ByteReceiveArray[8 + MessageStart] = ByteDataLength;     //Number of bytes after this one (or number of bytes of data).
            for(int i = 0; i < WordDataLength; i++)
            {
                ByteReceiveArray[ 9 + i * 2 + MessageStart] = highByte(R[Start + i]);
                ByteReceiveArray[10 + i * 2 + MessageStart] =  lowByte(R[Start + i]);
            }
            MessageLength = ByteDataLength + 9;
            PopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }

        //**4**************** Read Input Registers ******************
        else if(FC == MB_FC_READ_INPUT_REGISTERS_3x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            WordDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = WordDataLength * 2;
#ifdef MbDebug
            Serial.print(" MB_FC_READ_INPUT_REGISTERS_3x S=");
            Serial.print(Start);
            Serial.print(" L=");
            Serial.println(WordDataLength);
#endif
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 3; //Number of bytes after this one.
            ByteReceiveArray[8 + MessageStart] = ByteDataLength;     //Number of bytes after this one (or number of bytes of data).
            for(int i = 0; i < WordDataLength; i++)
            {
                ByteReceiveArray[ 9 + i * 2 + MessageStart] = highByte(IR[Start + i]);
                ByteReceiveArray[10 + i * 2 + MessageStart] =  lowByte(IR[Start + i]);
            }
            MessageLength = ByteDataLength + 9;
            PopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }



        //**5**************** Write Coil **********************
        else if(FC == MB_FC_WRITE_COIL_0x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            C[Start] = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]) > 0;
#ifdef MbDebug
            Serial.print(" MB_FC_WRITE_COIL_0x C");
            Serial.print(Start);
            Serial.print("=");
            Serial.println(C[Start]);
#endif
            ByteReceiveArray[5 + MessageStart] = 2; //Number of bytes after this one.
            MessageLength = 8;
            PopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }

        //**6**************** Write Single Register ******************
        else if(FC == MB_FC_WRITE_REGISTER_4x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            R[Start] = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
#ifdef MbDebug
            Serial.print(" MB_FC_WRITE_REGISTER_4x R");
            Serial.print(Start);
            Serial.print("=");
            Serial.println(R[Start]);
#endif
            ByteReceiveArray[5 + MessageStart] = 6; //Number of bytes after this one.
            MessageLength = 12;
            PopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }


        //**15**************** Write Multiple Coils **********************
        else if(FC == MB_FC_WRITE_MULTIPLE_COILS_0x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            CoilDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = CoilDataLength / 8;
            if(ByteDataLength * 8 < CoilDataLength) ByteDataLength++;
            CoilDataLength = ByteDataLength * 8;
#ifdef MbDebug
            Serial.print(" MB_FC_WRITE_MULTIPLE_COILS_0x S=");
            Serial.print(Start);
            Serial.print(" L=");
            Serial.println(CoilDataLength);
#endif
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 5; //Number of bytes after this one.
            for(int i = 0; i < ByteDataLength ; i++)
            {
                for(int j = 0; j < 8; j++)
                {
                    C[Start + i * 8 + j] = bitRead( ByteReceiveArray[13 + i + MessageStart], j);
                }
            }
            MessageLength = 12;
            PopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }


        //**16**************** Write Multiple Registers ******************
        else if(FC == MB_FC_WRITE_MULTIPLE_REGISTERS_4x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            WordDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = WordDataLength * 2;
#ifdef MbDebug
            Serial.print(" MB_FC_WRITE_MULTIPLE_REGISTERS_4x S=");
            Serial.print(Start);
            Serial.print(" L=");
            Serial.println(WordDataLength);
#endif
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 4; //Number of bytes after this one.
            for(int i = 0; i < WordDataLength; i++)
            {
                R[Start + i] =  word(ByteReceiveArray[ 13 + i * 2 + MessageStart],ByteReceiveArray[14 + i * 2 + MessageStart]);
            }
            MessageLength = 12;
            PopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }


        if (JustReceivedOne) {
            int i;
            MessageStart = MessageStart + 7 + ByteReceiveArray[5 + MessageStart];
            if (MessageStart+5<TotalMessageLength) SetFC(ByteReceiveArray[7 + MessageStart]);
            else {
                JustReceivedOne = false;
                FC = MB_FC_NONE;
                for (i=0; i<NoOfBytesToSend; i++) {
                    Serial.print(ByteSendArray[i],HEX);
                    Serial.print(" ");
                }
                client.write(ByteSendArray,NoOfBytesToSend);
                Serial.println(" sent");
                NoOfBytesToSend = 0;
                MessageStart = 0;
            }
#ifdef MbDebug
            Serial.print("TotalMessageLength = ");
            Serial.println(TotalMessageLength);
            Serial.print(" MessageStart = ");
            Serial.print(MessageStart);
            Serial.print(" FC = ");
            Serial.println(FC);


#endif
        }

    }

#ifdef MbRunsDebug
    Serial.print("Mb runs: ");
    Serial.print(Runs);
    Serial.print("  reads: ");
    Serial.print(Reads);
    Serial.print("  writes: ");
    Serial.print(Writes);
    Serial.println();
#endif

}

void Mudbus::PopulateSendBuffer(uint8_t *SendBuffer, int NoOfBytes)
{
    int i;
    i=0;
    while(i<NoOfBytes)
    {
        ByteSendArray[NoOfBytesToSend] = SendBuffer[i];
        NoOfBytesToSend++;
        i++;

    }
}
void Mudbus::SetFC(int fc)
{

// Read coils (FC 1) 0x
    if(fc == 1) FC = MB_FC_READ_COILS_0x;

// Read input discretes (FC 2) 1x
    else if(fc == 2) FC = MB_FC_READ_INPUTS_1x;

// Read multiple registers (FC 3) 4x
    else if(fc == 3) FC = MB_FC_READ_REGISTERS_4x;

// Read input registers (FC 4) 3x
    else if(fc == 4) FC = MB_FC_READ_INPUT_REGISTERS_3x;

// Write coil (FC 5) 0x
    else if(fc == 5) FC = MB_FC_WRITE_COIL_0x;

// Write single register (FC 6) 4x
    else if(fc == 6) FC = MB_FC_WRITE_REGISTER_4x;

// Read exception status (FC 7) we skip this one

// Force multiple coils (FC 15) 0x
    else if(fc == 15) FC = MB_FC_WRITE_MULTIPLE_COILS_0x;

// Write multiple registers (FC 16) 4x
    else if(fc == 16) FC = MB_FC_WRITE_MULTIPLE_REGISTERS_4x;

// Read general reference (FC 20)  we skip this one

// Write general reference (FC 21)  we skip this one

// Mask write register (FC 22)  we skip this one

// Read/write registers (FC 23)  we skip this one

// Read FIFO queue (FC 24)  we skip this one
    else {
        Serial.print(" FC not supported: ");
        Serial.print(fc);
        Serial.println();

    }
}



