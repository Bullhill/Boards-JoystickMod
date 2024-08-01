//Joystick - Ole Andreas Oksås - Cut and paste from everywhere


//-----------------------------------------------------------------------------------------------
// Change this number to reset and reload default parameters To EEPROM
#define EEP_Ident 0x5425  

//the default network address
struct ConfigIP {
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 5;
};  ConfigIP networkAddress;   //3 bytes
//-----------------------------------------------------------------------------------------------

#include <EEPROM.h> 
#include <Wire.h>
#include "EtherCard_AOG.h"
#include <IPAddress.h>


// ethernet interface ip address
static uint8_t myip[] = { 0,0,0,130 };

// gateway ip address
static uint8_t gwip[] = { 0,0,0,1 };

//DNS- you just need one anyway
static uint8_t myDNS[] = { 8,8,8,8 };

//mask
static uint8_t mask[] = { 255,255,255,0 };

//this is port of this autosteer module
uint16_t portMy = 5130;


//sending back to where and which port
static uint8_t ipDestination[] = { 0,0,0,255 };
uint16_t portDestination = 8888; //AOG port that listens

// ethernet mac address - must be unique on your network
static uint8_t mymac[] = { 0x00,0x00,0x56,0x00,0x00,0x82 };

uint8_t Ethernet::buffer[200]; // udp send and receive buffer


//Program counter reset
void(*resetFunc) (void) = 0;

//ethercard 10,11,12,13 Nano = 10 depending how CS of ENC28J60 is Connected
#define CS_Pin 10


#define JoystickX A7
#define JoystickY A6


int workingFrequens = 100; //1000ms / 10hz

uint8_t maxSpeed = 128;
bool invertXAxis = true;
bool invertYAxis = true;

int16_t JoystickPos = 0; //-255 to 255
uint8_t Deadband = 50; //For adjusting deadband in joystick
float JoystickPowValue = 2.5; //Exponent to make it easy to fine steer. 1 = linear


int8_t YAxisStatus = 0;
int YAxisLimit = 500;
int YAxisReset = 200;


long MaxPowValue = (long) pow(512-Deadband, JoystickPowValue);


int JoystickReading;

//JoystickPNGs
uint8_t SteerChangePGN[] = { 128, 129, 130, 177, 1, 0, 0, 0 };
int8_t SteerChangePGN_Size = sizeof(SteerChangePGN) - 1;
uint8_t ButtonClickPGN[] = { 128, 129, 130, 178, 1, 0, 0 };
int8_t ButtonClickPGN_Size = sizeof(ButtonClickPGN) - 1;


//Comm checks
uint8_t serialResetTimer = 0; //if serial buffer is getting full, empty it


//Communication with AgOpenGPS
int16_t temp, EEread = 0;

//Parsing PGN
bool isPGNFound = false, isHeaderFound = false;
uint8_t pgn = 0, dataLength = 0, idx = 0;
int16_t tempHeader = 0;

//The variables used for storage
float gpsSpeed;

void setup()
{

    //set the baud rate
    Serial.begin(38400);
    //while (!Serial) { ; } // wait for serial port to connect. Needed for native USB

    if (ether.begin(sizeof Ethernet::buffer, mymac, CS_Pin) == 0)
        Serial.println(F("Failed to access Ethernet controller"));

    //grab the ip from EEPROM
    myip[0] = networkAddress.ipOne;
    myip[1] = networkAddress.ipTwo;
    myip[2] = networkAddress.ipThree;

    gwip[0] = networkAddress.ipOne;
    gwip[1] = networkAddress.ipTwo;
    gwip[2] = networkAddress.ipThree;

    ipDestination[0] = networkAddress.ipOne;
    ipDestination[1] = networkAddress.ipTwo;
    ipDestination[2] = networkAddress.ipThree;

    //set up connection
    ether.staticSetup(myip, gwip, myDNS, mask);
    ether.printIp("_IP_: ", ether.myip);
    ether.printIp("GWay: ", ether.gwip);
    ether.printIp("AgIO: ", ipDestination);

    //register to port 10555
    ether.udpServerListenOnPort(&udpSteerRecv, 10555);


    Serial.println("Setup complete, waiting for AgOpenGPS");

}


void loop() {

//int16_t JoystickPos = 0; //-255 to 255
//uint8_t Deadband = 50; //For adjusting deadband in joystick
//float JoystickPowValue = 3.5; //Exponent to make it easy to fine steer. 1 = linear
//int JoystickReading;

  int YAxis = analogRead(JoystickY);
  if(invertYAxis)
    YAxis = 1024 - YAxis;

  if(YAxis < 512 + YAxisReset && YAxis > 512 - YAxisReset)
    YAxisStatus = 0;
  else if(YAxisStatus == 0 && YAxis > (512 + YAxisLimit))
  {
    YAxisStatus = 1;
    ButtonClickPGN[5] = 0x02;

    //checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < ButtonClickPGN_Size; i++)
    {
        CK_A = (CK_A + ButtonClickPGN[i]);
    }
    ButtonClickPGN[ButtonClickPGN_Size] = CK_A;

    //off to AOG
    ether.sendUdp(ButtonClickPGN, sizeof(ButtonClickPGN), portMy, ipDestination, portDestination);
    
    Serial.println("Enable autosteer!");
  }
  else if(YAxisStatus == 0 && YAxis < (512 - YAxisLimit))
  {
    YAxisStatus = -1;
    ButtonClickPGN[5] = 0x01;

    //checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < ButtonClickPGN_Size; i++)
    {
        CK_A = (CK_A + ButtonClickPGN[i]);
    }
    ButtonClickPGN[ButtonClickPGN_Size] = CK_A;

    //off to AOG
    ether.sendUdp(ButtonClickPGN, sizeof(ButtonClickPGN), portMy, ipDestination, portDestination);
    
    Serial.println("Enable joystickSteer!");
  }
  
  Serial.print(YAxis);
  Serial.print(" - ");
  Serial.println(YAxisStatus);
  

  JoystickReading = analogRead(JoystickX);
  if(invertXAxis)
    JoystickReading = 1024 - JoystickReading;

  if(JoystickReading > 512 - Deadband && JoystickReading < 512 + Deadband)
  {
    JoystickPos = 0;
  }
  else
  {
    int calculatedValue = JoystickReading - 512;

    if(calculatedValue < 0)
      calculatedValue = calculatedValue * -1;

    calculatedValue = calculatedValue - Deadband;

    double PowerdValue = pow((float)calculatedValue, JoystickPowValue);
    
    JoystickPos = (int16_t) ((double)(PowerdValue / MaxPowValue) * maxSpeed);

    if(JoystickReading < 512)
      JoystickPos = JoystickPos * -1;
    
  }
  SteerChangePGN[5] = (uint8_t)JoystickPos;
  SteerChangePGN[6] = JoystickPos >> 8;
  

  
  
  //checksum
  int16_t CK_A = 0;
  for (uint8_t i = 2; i < SteerChangePGN_Size; i++)
  {
      CK_A = (CK_A + SteerChangePGN[i]);
  }
  SteerChangePGN[SteerChangePGN_Size] = CK_A;

  //off to AOG
  ether.sendUdp(SteerChangePGN, sizeof(SteerChangePGN), portMy, ipDestination, portDestination);


  delay(workingFrequens);

}

/*
//Button press hanlding function
void pressHandler (BfButton *btn, BfButton::press_pattern_t pattern) {
  switch (pattern) {
    case BfButton::SINGLE_PRESS:
      ButtonClickPGN[5] = 0x01;
      //Serial.println("Single push");
      break;
      
    case BfButton::DOUBLE_PRESS:
      SelectedSpeed = (SelectedSpeed + 1)%3;
      //Serial.print("Double push, Speed selected: ");
      //Serial.println(Speed[SelectedSpeed]);
      ButtonClickPGN[5] = 0x02;
      break;
      
    case BfButton::LONG_PRESS:
      //Serial.println("Long push");
      ButtonClickPGN[5] = 0x03;
      break;
  }

      
    //checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < ButtonClickPGN_Size; i++)
    {
        CK_A = (CK_A + ButtonClickPGN[i]);
    }
    ButtonClickPGN[ButtonClickPGN_Size] = CK_A;

    //off to AOG
    ether.sendUdp(ButtonClickPGN, sizeof(ButtonClickPGN), portMy, ipDestination, portDestination);
}*/

void udpSteerRecv(uint16_t dest_port, uint8_t src_ip[IP_LEN], uint16_t src_port, uint8_t* udpData, uint16_t len)
{
    /* IPAddress src(src_ip[0],src_ip[1],src_ip[2],src_ip[3]);
    Serial.print("dPort:");  Serial.print(dest_port);
    Serial.print("  sPort: ");  Serial.print(src_port);
    Serial.print("  sIP: ");  ether.printIp(src_ip);  Serial.println("  end");

    //for (int16_t i = 0; i < len; i++) {
    //Serial.print(udpData[i],HEX); Serial.print("\t"); } Serial.println(len);
    */

    /*if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) //Data
    {

        if (udpData[3] == 239)  //machine data
        {
            uTurn = udpData[5];
            gpsSpeed = (float)udpData[6];//actual speed times 4, single uint8_t

            hydLift = udpData[7];
            tramline = udpData[8];  //bit 0 is right bit 1 is left

            relayLo = udpData[11];          // read relay control from AgOpenGPS
            relayHi = udpData[12];

            if (aogConfig.isRelayActiveHigh)
            {
                tramline = 255 - tramline;
                relayLo = 255 - relayLo;
                relayHi = 255 - relayHi;
            }

            //Bit 13 CRC

            //reset watchdog
            watchdogTimer = 0;
        }

        else if (udpData[3] == 200) // Hello from AgIO
        {
            if (udpData[7] == 1)
            {
                relayLo -= 255;
                relayHi -= 255;
                watchdogTimer = 0;
            }

            helloFromMachine[5] = relayLo;
            helloFromMachine[6] = relayHi;

            ether.sendUdp(helloFromMachine, sizeof(helloFromMachine), portMy, ipDestination, portDestination);
        }


        else if (udpData[3] == 238)
        {
            aogConfig.raiseTime = udpData[5];
            aogConfig.lowerTime = udpData[6];
            aogConfig.enableToolLift = udpData[7];

            //set1 
            uint8_t sett = udpData[8];  //setting0     
            if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;

            aogConfig.user1 = udpData[9];
            aogConfig.user2 = udpData[10];
            aogConfig.user3 = udpData[11];
            aogConfig.user4 = udpData[12];

            //crc

            //save in EEPROM and restart
            EEPROM.put(6, aogConfig);
            //resetFunc();
        }

        else if (udpData[3] == 201)
        {
            //make really sure this is the subnet pgn
            if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201)
            {
                networkAddress.ipOne = udpData[7];
                networkAddress.ipTwo = udpData[8];
                networkAddress.ipThree = udpData[9];

                //save in EEPROM and restart
                EEPROM.put(50, networkAddress);
                resetFunc();
            }
        }

        //Scan Reply
        else if (udpData[3] == 202)
        {
            //make really sure this is the subnet pgn
            if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202)
            {
                uint8_t scanReply[] = { 128, 129, 123, 203, 7, 
                    networkAddress.ipOne, networkAddress.ipTwo, networkAddress.ipThree, 123,
                    src_ip[0], src_ip[1], src_ip[2], 23   };

                //checksum
                int16_t CK_A = 0;
                for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
                {
                    CK_A = (CK_A + scanReply[i]);
                }
                scanReply[sizeof(scanReply)-1] = CK_A;

                static uint8_t ipDest[] = { 255,255,255,255 };
                uint16_t portDest = 9999; //AOG port that listens

                //off to AOG
                ether.sendUdp(scanReply, sizeof(scanReply), portMy, ipDest, portDest);
            }
        }

        else if (udpData[3] == 236) //EC Relay Pin Settings 
        {
            for (uint8_t i = 0; i < 24; i++)
            {
                pin[i] = udpData[i + 5];
            }

            //save in EEPROM and restart
            EEPROM.put(20, pin);
        }
    }*/
}
