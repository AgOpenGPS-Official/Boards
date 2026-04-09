
// Serial Ports
#define SerialAOG Serial                //AgIO USB conection
#define SerialRTK Serial3               //RTK radio

#if defined(__IMXRT1062__)
  HardwareSerialIMXRT* SerialGPS = &Serial7; //Main postion receiver (GGA)
  HardwareSerialIMXRT* SerialGPS2 = &Serial2; //Dual heading receiver 
  HardwareSerialIMXRT* SerialIMU = &Serial5; //IMU
#else
  HardwareSerial* SerialGPS = &Serial7;
  HardwareSerial* SerialGPS2 = &Serial2;
  HardwareSerial* SerialIMU = &Serial5;
#endif

constexpr int serial_buffer_size = 1023;

const int32_t baudGPS = 460800;
const int32_t baudRTK = 115200;     // most are using Xbee radios with default of 115200

#define ImuWire Wire        //SCL=19:A5 SDA=18:A4
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

//Status LED's
#define GGAReceivedLED 13         //Teensy onboard LED
#define Power_on_LED 5            //Red
#define Ethernet_Active_LED 6     //Green
#define GPSRED_LED 9              //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
#define GPSGREEN_LED 10           //Green (Flashing = Dual bad, ON = Dual good)
#define AUTOSTEER_STANDBY_LED 11  //Red
#define AUTOSTEER_ACTIVE_LED 12   //Green

/*****************************************************************/

#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO_RVC.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <FlexCAN_T4.h>

//Roomba Vac mode for BNO085 and data
BNO_rvc rvc = BNO_rvc();
BNO_rvcData bnoData;
elapsedMillis bnoTimer;
bool bnoTrigger = false;
bool useBNO08xRVC = false;

//I2C BNO085
#define i2cBNO_REPORT_INTERVAL 20
elapsedMillis i2cBNOTimer = 0;
#include "BNO08x_AOG.h"
bool useBNO08xI2C = false;

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

// Keya Support
// CRX1/CTX1 on Teensy are CAN1 on Tony's board
// CRX2/CTX2 on Teensy are CAN2 on AIO board, CAN2 on Tony's board
// CRX3/CTX3 on Teensy are CAN1 on AIO board, CAN3 on Tony's board
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Keya_Bus;

elapsedMillis lastKeyaHeatbeat;
bool keyaDetected = false;
bool keyaIntendToSteer;
int16_t keyaSteeringPosition;
int16_t keyaCurrentSetSpeed;
int16_t keyaCurrentActualSpeed;

#define wheelBase 3.20
int8_t workingDir;
float wheelAngleGPS;

struct ConfigIP
{
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 5;
};  ConfigIP networkAddress;   //3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = { 0, 0, 0, 0}; //This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy = 5120;                         // port of this module
unsigned int AOGNtripPort = 2233;                   // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;               // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;                // Port of AOG that listens
char Eth_NTRIP_packetBuffer[serial_buffer_size];    // buffer for receiving ntrip data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;     //Out port 5544
EthernetUDP Eth_udpNtrip;     //In port 2233
EthernetUDP Eth_udpAutoSteer; //In & Out Port 8888

IPAddress Eth_ipDestination;

byte CK_A = 0;
byte CK_B = 0;
int relposnedByteCount = 0;

//Speed pulse output
elapsedMillis speedPulseUpdateTimer = 0;
byte velocityPWM_Pin = 36;      // Velocity (MPH speed) PWM pin

//Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

bool useDual = false;
bool dualReadyGGA = false;
bool dualReadyRelPos = false;

elapsedMillis GGAReadyTime = 10000;
elapsedMillis ethernetLinkCheck = 1000;

//Dual
double headingcorr = 900;  //90deg heading correction (90deg*10)

double baseline = 0;
double rollDual = 0;
double relPosD = 0;
double heading = 0;
double headingVTG = 0;
double headingRate = 0;
int8_t GPS_Hz = 10;
int  solQualityHPR;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t GPSrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];    //Extra serial tx buffer
uint8_t GPS2rxbuffer[serial_buffer_size];   //Extra serial rx buffer
uint8_t GPS2txbuffer[serial_buffer_size];   //Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];    //Extra serial rx buffer

/* A parser is declared with 3 handlers at most */
NMEAParser<3> parser;

bool isTriggered = false;
bool blink = false;

bool Autosteer_running = true; //Auto set off in autosteer setup

float roll = 0;
float pitch = 0;
float yaw = 0;

// Setup procedure ------------------------
void setup()
{
    delay(500);                       //Small delay so serial can monitor start up
    set_arm_clock(450000000);         //Set CPU speed to 150mhz
    Serial.println("\r\n** AIO v4 Firmware 27.04.2025 (Inc Keya CANBUS & HPR Dual **\r\n");
    Serial.print("CPU speed set to: ");
    Serial.println(F_CPU_ACTUAL);

    pinMode(GGAReceivedLED, OUTPUT);
    pinMode(Power_on_LED, OUTPUT);
    pinMode(Ethernet_Active_LED, OUTPUT);
    pinMode(GPSRED_LED, OUTPUT);
    pinMode(GPSGREEN_LED, OUTPUT);
    pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
    pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);
    
    // the dash means wildcard
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA", GGA_Handler);
    parser.addHandler("G-VTG", VTG_Handler);
    parser.addHandler("G-HPR", HPR_Handler);

    delay(10);
    Serial.println("Start setup");

    SerialGPS->begin(baudGPS);
    SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

    delay(10);
    SerialRTK.begin(baudRTK);
    SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

    delay(10);
    SerialGPS2->begin(baudGPS);
    SerialGPS2->addMemoryForRead(GPS2rxbuffer, serial_buffer_size);
    SerialGPS2->addMemoryForWrite(GPS2txbuffer, serial_buffer_size);

    Serial.println("SerialAOG, SerialRTK, SerialGPS and SerialGPS2 initialized");

    Serial.println("\r\nStarting AutoSteer...");
    autosteerSetup();
  
    Serial.println("\r\nStarting Ethernet...");
    EthernetStart();

    Serial.println("\r\nStarting CANBUS...");
    CAN_Setup();

    SerialIMU->begin(115200);
    rvc.begin(SerialIMU);

    static elapsedMillis rvcBnoTimer = 0;
    Serial.println("\r\nChecking for serial BNO08x");
    while (rvcBnoTimer < 1000)
    {
        //check if new bnoData
        if (rvc.read(&bnoData))
        {
            useBNO08xRVC = true;
            Serial.println("Serial BNO08x Good To Go :-)");
            imuHandler();
            break;
        }
    }
    if (!useBNO08xRVC)
    {
        Serial.println("No Serial BNO08x not Connected or Found");

        Serial.println("Checking for i2c BNO08x");

        ImuWire.begin();

        uint8_t error;

        for (int16_t i = 0; i < nrBNO08xAdresses; i++)
        {
            bno08xAddress = bno08xAddresses[i];

            ImuWire.beginTransmission(bno08xAddress);
            error = ImuWire.endTransmission();

            if (error == 0)
            {
                //Serial.println("Error = 0");
                Serial.print("0x");
                Serial.print(bno08xAddress, HEX);
                Serial.println(" BNO08X Ok.");

                // Initialize BNO080 lib
                if (bno08x.begin(bno08xAddress, ImuWire)) //??? Passing NULL to non pointer argument, remove maybe ???
                {
                    //Increase I2C data rate to 400kHz
                    ImuWire.setClock(400000);

                    delay(300);

                    bno08x.enableGameRotationVector(i2cBNO_REPORT_INTERVAL);
                    i2cBNOTimer = 0;
                    useBNO08xI2C = true;
                }
                else
                {
                    Serial.println("BNO080 not detected at given I2C address.");
                }
            }
            else
            {
                //Serial.println("Error = 4");
                Serial.print("0x");
                Serial.print(bno08xAddress, HEX);
                Serial.println(" BNO08X not Connected or Found");
            }
            if (useBNO08xI2C) break;
        }
    }

  Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
}

void loop()
{
    // Read incoming nmea from GPS
    if (SerialGPS->available())
    {
        parser << SerialGPS->read();
    }

    // Check for RTK via Radio
    if (SerialRTK.available())
    {
        SerialGPS->write(SerialRTK.read());
    }

    // Check for RTK via UDP
    unsigned int packetLength = Eth_udpNtrip.parsePacket();

    if (packetLength > 0)
    {
        if (packetLength > serial_buffer_size) packetLength = serial_buffer_size;
        Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, packetLength);
        SerialGPS->write(Eth_NTRIP_packetBuffer, packetLength);
    }

    // If both dual messages are ready, send to AgOpen
    if (dualReadyGGA == true && dualReadyRelPos == true)
    {
        BuildNmea();
        dualReadyGGA = false;
        dualReadyRelPos = false;
    }
    
    // If anything comes in SerialGPS2 RelPos data
    if (SerialGPS2->available())
    {
        uint8_t incoming_char = SerialGPS2->read();  //Read RELPOSNED from F9P

        // Just increase the byte counter for the first 3 bytes
        if (relposnedByteCount < 4 && incoming_char == ackPacket[relposnedByteCount])
        {
            relposnedByteCount++;
        }
        else if (relposnedByteCount > 3)
        {
            // Real data, put the received bytes in the buffer
            ackPacket[relposnedByteCount] = incoming_char;
            relposnedByteCount++;
        }
        else
        {
            // Reset the counter, becaues the start sequence was broken
            relposnedByteCount = 0;
        }
    }
    
    // Check the message when the buffer is full
    if (relposnedByteCount > 71)
    {
        if (calcChecksum())
        {
            //if(deBug) Serial.println("RelPos Message Recived");
            digitalWrite(GPSRED_LED, LOW);   //Turn red GPS LED OFF (we are now in dual mode so green LED)
            useDual = true;
            relPosDecode();
        }
        relposnedByteCount = 0;
    }

    //RVC BNO08x
    if (rvc.read(&bnoData)) useBNO08xRVC = true;

    if (useBNO08xRVC && bnoTimer > 70 && bnoTrigger)
    {
        bnoTrigger = false;
        imuHandler();   //Get IMU data ready
    }

    //i2c BNO08x
    if (useBNO08xI2C && i2cBNOTimer > 20)
    {
        i2cBNOTimer = 0;
        readBNO();
    }
    
    if (Autosteer_running) autosteerLoop();
    else ReceiveUdp();
    
    //GGA timeout, turn off GPS LED's etc
    if (GGAReadyTime > 10000) //GGA age over 10sec
    {
        digitalWrite(GPSRED_LED, LOW);
        digitalWrite(GPSGREEN_LED, LOW);
        useDual = false;
    }

    if (ethernetLinkCheck > 10000)
    {
        if (Ethernet.linkStatus() == LinkON)
        {
            ethernetLinkCheck = 0;
            digitalWrite(Power_on_LED, 0);
            digitalWrite(Ethernet_Active_LED, 1);
        }
        else
        {
            digitalWrite(Power_on_LED, 1);
            digitalWrite(Ethernet_Active_LED, 0);
        }
    }

}//End Loop
//**************************************************************************

bool calcChecksum()
{
  CK_A = 0;
  CK_B = 0;

  for (int i = 2; i < 70; i++)
  {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  return (CK_A == ackPacket[70] && CK_B == ackPacket[71]);
}
