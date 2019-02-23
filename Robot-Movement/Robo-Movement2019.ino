#include <SoftwareSerial.h>
#include <SPI.h>
#include <mcp_can.h>

#define controller_one 4
#define controller_two 2
#define controller_three 3 
#define controller_four 1
#define MAX_SPD 500.0

//pin for the CS
const int spiCSPin = 53;
MCP_CAN CAN(spiCSPin);
// Verticle and Horizontal Levels
int verL, horL, rotL; 
//Motor Input Levels
int FLs, FRs, BLs, BRs; 
unsigned char dataFL[8] = {0,0,0,0,0,0,0,0};
unsigned char dataFR[8] = {0,0,0,0,0,0,0,0};
unsigned char dataBL[8] = {0,0,0,0,0,0,0,0};
unsigned char dataBR[8] = {0,0,0,0,0,0,0,0};

//frame which is used for percent output.
const uint32_t CONTROL = 0x040080;

//ID used for sending the enable frame
const uint32_t ENABLE = 0x0401BF;

//build the control CANbus frame.  The first three bytes is the demand value, 
//which typically is the output value [-1023,+1023]

//data for the ENABLE frame
unsigned char enable[8] = {1, 0, 0, 0, 0, 0, 0, 0};

//Recieving byte
byte incomingByte = 0;
bool shooting = false;

//Storage for bytes for decoding
byte incoming_command[2];
int queue_len = 0;

//storage for values being sent [button / joy] [value]
//97-100 are joystick values
//101-108 are button values
int current_vals[12];

int axis = 0;

void setup()
{
  Serial.begin(115200);

  Serial.println("initializing...");

  //set the baudrate and let it know we are using the 8MHz oscilator on the CAN module
    while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_8MHz))
    {
        Serial.println("CAN BUS init Failed");
        delay(100);
    }
    Serial.println("CAN BUS Shield Init OK!");
  Serial1.begin(9600);
  Serial.println("Starting Recieve Code");
}

void check_command()
{
  //incoming_command[0] is the encoded value being converted to decimal
  if (incoming_command[0] <= 100 && incoming_command[0] >= 97)
  {
    axis = incoming_command[0];
    current_vals[(axis - 97)] = map(int(incoming_command[1]), 0, 200, -500, 500);
    Serial.print("Joystick ");
    Serial.print(incoming_command[0], HEX);
    Serial.print(" ");
    Serial.print(map(incoming_command[1], 0, 200, -500, 500));
    Serial.println();
  }
}

int SpeedControl(double SPD) //Creates a exponential growth speed
{
  int NewSPD;
  double temp2 = SPD / 500.0;
  double temp =pow(temp2,2);
  NewSPD = temp * SPD;
  return NewSPD;
}

void funcTest(unsigned char data[8], int SPD) //Allows Can Bus to Read speeds.
{
  //encode output into bytes
  byte first_byte = (byte)(SPD >> 0x10);
  byte second_byte = (byte)(SPD >> 0x08);
  byte third_byte = (byte)(SPD);

  //build the control CANbus frame.  The first three bytes is the demand value, 
  //which typically is the output value [-1023,+1023]

  //data for the CONTROL frame
  data[0] = first_byte;
  data[1] = second_byte;
  data[2] = third_byte;
}


void loop()
{
  if (Serial1.available() > 0)
  {
    incomingByte = Serial1.read();
    if (incomingByte == 0x58)
    {
      //Serial.println();
      check_command();
      queue_len = 0;
    }
    else
    {
      if (queue_len >= 2)
      {
        queue_len = 0;
      }
      incoming_command[queue_len] = incomingByte;
      queue_len++;
      //Serial.print(incomingByte, HEX);
     // Serial.print(" ");
    }
    //Serial.println(incomingByte, HEX);
  }

  verL = SpeedControl(current_vals[1]); //Runs function to create speed components
  //Serial.println(verL);
  horL = SpeedControl(current_vals[2]);
  //Serial.println(horL);
  rotL = SpeedControl(current_vals[3]);
  //Serial.println(rotL);
  
  FRs = -verL - horL - rotL; //Creates the speeds for each motor.
  BRs = -verL + horL - rotL;
  FLs = verL - horL - rotL;
  BLs = verL + horL - rotL; 

  FRs = constrain(FRs, -500, 500); //Restricts The output to motor
  BRs = constrain(BRs, -500, 500);
  FLs = constrain(FLs, -500, 500);
  BLs = constrain(BLs, -500, 500);

  //Outputs the speeds to each motor.
  funcTest(dataFL, FLs);
  CAN.sendMsgBuf(CONTROL | controller_one | 0x01040000, 1, 8, dataFL); //0x01040000 for Victor, 0x02040000 for Talon
  Serial.print(FLs);
  Serial.print("  ");
  
  funcTest(dataFR, FRs);
  CAN.sendMsgBuf(CONTROL | controller_two | 0x01040000, 1, 8, dataFR); //0x01040000 for Victor, 0x02040000 for Talon
  Serial.print(FRs);
  Serial.print("  ");
  
  funcTest(dataBR, BRs);
  CAN.sendMsgBuf(CONTROL | controller_three | 0x01040000, 1, 8, dataBL); //0x01040000 for Victor, 0x02040000 for Talon
  Serial.print(BLs);
  Serial.print("  ");
  
  funcTest(dataBL, BLs);
  CAN.sendMsgBuf(CONTROL | controller_four | 0x01040000, 1, 8, dataBR); //0x01040000 for Victor, 0x02040000 for Talon
  Serial.println(BRs); 
  
  CAN.sendMsgBuf(ENABLE, 1, 8, enable);
  delay(20);
}