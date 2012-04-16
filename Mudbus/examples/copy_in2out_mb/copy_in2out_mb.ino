#include <SPI.h>
#include <Ethernet.h>

#include "Mudbus.h"

//#define DEBUG
static unsigned long time;
Mudbus Mb;
//Function codes 
//Read coils (FC 1) 0x
//Read input discretes (FC 2) 1x
//Read multiple registers (FC 3) 4x
//Read input registers (FC 4) 3x
//Write coil (FC 5) 0x
//Write single register (FC 6) 4x
//Force multiple coils (FC 15) 0x
//Write multiple registers (FC 16) 4x
//    bool C[MB_N_C_0x];
//    bool I[MB_N_I_1x];
//    int  IR[MB_N_IR_3x];
//    int  R[MB_N_HR_4x];

//Port 502 (defined in Mudbus.h) MB_PORT

void setup()
{
  uint8_t mac[]     = { 
    0x90, 0xA2, 0xDA, 0x00, 0x71, 0x17   };
  uint8_t ip[]      = { 
    192, 168, 1, 8   };
  uint8_t gateway[] = { 
    192, 168, 1, 1   };
  uint8_t subnet[]  = { 
    255, 255, 255, 0   };
  Ethernet.begin(mac, ip, gateway, subnet);
  //Avoid pins 4,10,11,12,13 when using ethernet shield

  delay(5000);  //Time to open the terminal
  Serial.begin(115200);
  time = millis()+1000;


}

void loop()
{

word location;

  Mb.Run();

  for (location=0 ; location< MB_N_C_0x; location++) { 
    Mb.I[location] = Mb.C[location];
  }
  for (location=0 ; location< MB_N_IR_3x; location++) { 
    Mb.IR[location] = Mb.R[location];
  }

    if ((long)( millis() - time ) > 0) { 
    time += 1000;
    
    Serial.print("\n\nIR0=");
    Serial.print(Mb.IR[0]);
    Serial.print(" IR1=");
    Serial.print(Mb.IR[1]);
    Serial.print(" IR2=");
    Serial.print(Mb.IR[2]);
    Serial.print(" IR3=");
    Serial.print(Mb.IR[3]);
    Serial.print(" IR4=");
    Serial.print(Mb.IR[4]);
    Serial.print(" IR5=");
    Serial.print(Mb.IR[5]);


    Serial.print("\n\nI0=");
    Serial.print(Mb.I[0]);
    Serial.print(" I1=");
    Serial.print(Mb.I[1]);
    Serial.print(" I2=");
    Serial.print(Mb.I[2]);
    Serial.print(" I3=");
    Serial.print(Mb.I[3]);
    Serial.print(" I4=");
    Serial.print(Mb.I[4]);
    Serial.print(" I5=");
    Serial.print(Mb.I[5]);

    Serial.print("\n\nA0=");
    Serial.print(Mb.R[0]);
    Serial.print(" A1=");
    Serial.print(Mb.R[1]);
    Serial.print(" A2=");
    Serial.print(Mb.R[2]);
    Serial.print(" A3=");
    Serial.print(Mb.R[3]);
    Serial.print(" A4=");
    Serial.print(Mb.R[4]);
    Serial.print(" A5=");
    Serial.print(Mb.R[5]);


    Serial.print("\n\nC0=");
    Serial.print(Mb.C[0]);
    Serial.print(" C1=");
    Serial.print(Mb.C[1]);
    Serial.print(" C2=");
    Serial.print(Mb.C[2]);
    Serial.print(" C3=");
    Serial.print(Mb.C[3]);
    Serial.print(" C4=");
    Serial.print(Mb.C[4]);
    Serial.print(" C5=");
    Serial.print(Mb.C[5]);

  }

}

/*
A minimal Modbus TCP slave for Arduino. 
It has function codes 

Read coils (FC 1) 0x
Read input discretes (FC 2) 1x
Read multiple registers (FC 3) 4x
Read input registers (FC 4) 3x
Write coil (FC 5) 0x
Write single register (FC 6) 4x
Force multiple coils (FC 15) 0x
Write multiple registers (FC 16) 4x

It is set up to use as a library, 
so the Modbus related stuff is 
separate from the main sketch. 
  
 Mb.C[0-63] bool  
 Digital outputs
 
 Mb.I[0-63] bool 
 Digital inputs
 
 Mb.IR[0-15] signed int 
 Input registers (for AD converters, counters etc)
 
 Mb.R[0-15] signed int 
 Holding registers (for DA converters, frequency outputs etc)

 Modpoll commands
 
 Read the registers Mb.R[3], Mb.R[4], and Mb.R[5]
 ./modpoll -m tcp -t4 -r 4 -c 6 192.168.1.8
 */


