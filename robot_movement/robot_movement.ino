/******************************************************************************
 * Arduino Due
 * Contains the code for the robot's main controller
******************************************************************************/
/******************************************************************************
 * TODO
 *      -remove delay
 *      -change deadman's to
 *          use millis()
 *      -add toggle for debug messages
******************************************************************************/
#include <SoftwareSerial.h>
#include <SPI.h>
#include <mcp_can.h>

/******************************************************************************
 * Defines
******************************************************************************/
//SPX Victor Motor Controllers
#define controller_one 4
#define controller_two 2
#define controller_three 3 
#define controller_four 1

//Max speed for robot movement
#define MAX_SPD 500.0

//cs pin for SPI and CAN Bus
#define spiCSPin 53

//Address for control on RoboClaw
//motor controllers
#define rc_lift 0x80
#define rc_corral 0x81
#define rc_door_right 0x82
#define rc_door_left 0x83

/******************************************************************************
 * Globals
******************************************************************************/

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

//Mortor Controller Calibraiton
const int CALIBRAITON_FL_FOR = 24;
const int CALIBRATION_FL_REV = 32;
const int CALIBRATION_BR_FOR = 50;
const int CALIBRATION_BR_REV = 40;

//data for the ENABLE frame
unsigned char enable[8] = {1, 0, 0, 0, 0, 0, 0, 0};

//Recieving byte
byte incomingByte = 0;

//Storage for bytes for decoding
byte incoming_command[2];
int queue_len = 0;

//storage for values being sent [button / joy] [value]
//97-100 are joystick values
//101-108 are button values
int current_vals[12];

int axis = 0;

//Test Time
int timer = 0;
int oldspeed;

/******************************************************************************
 * Object Declaration
******************************************************************************/

//create CAN object
MCP_CAN CAN(spiCSPin);

//create roboclaw object
RoboClaw rc_driver(&Serial3, 10000);

void setup()
{
    //pc debug
    Serial.begin(115200);

    Serial.println("initializing CAN Bus...");

    //set the baudrate and let it know we are using the 8MHz oscilator on the CAN module
    while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_8MHz))
    {
        Serial.println("CAN BUS init Failed");
        delay(100);
    }
    Serial.println("CAN BUS Init OK!");   
    
    //xBee communication
    Serial.println("Starting Recieve Code");
    Serial1.begin(9600);
 
    //RoboClaw
    rc_driver,begin(38400);
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
    else if(incoming_command[0] == 'A')
    {
        //Right thumb button
    }
    else if(incoming_command[0] == 'B')
    {
        //right bank, right button
    }
    else if(incoming_command[0] == 'C')
    {
        //right bank, left button
    }
    else if(incoming_command[0] == 'D')
    {
        //right bank, top button
    }
    else if(incoming_command[0] == 'E')
    {
        //right bank, bottom button
    }
    else if(incoming_command[0] == 'F')
    {
        //left thumb button
    }
    else if(incoming_command[0] == 'G')
    {
        //left bank, bottom button
    }
    else if(incoming_command[0] == 'H')
    {
        //left bank, left button
    }
    else if(incoming_command[0] == 'I')
    {
        //left bank, top button
    }
    else if(incoming_command[0] == 'J')
    {
        //left bank, right button
    }
    else if(incoming_command[0] == 'K')
    {
        //top bank, 
    }
    else if(incoming_command[0] == 'L')
    {
        //top bank, 
    }
    else if(incoming_command[0] == 'M')
    {
        //top bank, 
    }
    else if(incoming_command[0] == 'N')
    {
        //top bank, 
    }
}

int SpeedControl(double spd) //Creates a exponential growth speed
{
    int NewSPD;
    double temp2 = spd / MAX_SPD;
    double temp = pow(temp2,2);
    NewSPD = temp * spd;
    return NewSPD;
}

void byte_encoder(unsigned char data[8], int SPD) //Allows Can Bus to Read speeds.
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

    if (FLs > 0)
    {
        FLs += CALIBRAITON_FL_FOR;
    }
    if (FRs < 0)
    {
        FLs -= CALIBRATION_FL_REV;
    }

    if (BRs > 0)
    {
        BRs += CALIBRATION_BR_FOR;
    }
    if (BRs < 0)
    {
        BRs -= CALIBRATION_BR_REV;
    }

    FRs = constrain(FRs, -MAX_SPD, MAX_SPD); //Restricts The output to motor
    BRs = constrain(BRs, -MAX_SPD, MAX_SPD);
    FLs = constrain(FLs, -MAX_SPD, MAX_SPD);
    BLs = constrain(BLs, -MAX_SPD, MAX_SPD);

    //Outputs the speeds to each motor.
    byte_encoder(dataFL, FLs);
    CAN.sendMsgBuf(CONTROL | controller_one | 0x01040000, 1, 8, dataFL); //0x01040000 for Victor, 0x02040000 for Talon
    Serial.print(FLs);
    Serial.print("  ");

    byte_encoder(dataFR, FRs);
    CAN.sendMsgBuf(CONTROL | controller_two | 0x01040000, 1, 8, dataFR); //0x01040000 for Victor, 0x02040000 for Talon
    Serial.print(FRs);
    Serial.print("  ");

    byte_encoder(dataBR, BRs);
    CAN.sendMsgBuf(CONTROL | controller_three | 0x01040000, 1, 8, dataBL); //0x01040000 for Victor, 0x02040000 for Talon
    Serial.print(BLs);
    Serial.print("  ");

    byte_encoder(dataBL, BLs);
    CAN.sendMsgBuf(CONTROL | controller_four | 0x01040000, 1, 8, dataBR); //0x01040000 for Victor, 0x02040000 for Talon
    Serial.print(BRs); 

    CAN.sendMsgBuf(ENABLE, 1, 8, enable);

    Serial.print("  ");
    Serial.print(timer);
    Serial.print("  ");
    Serial.println(oldspeed);

    //Deadman's switch for bot movement
    if (timer == 0)
    {
        oldspeed = FLs;
    }
    timer = timer + 1;
    if (timer > 100)
    {
        if (oldspeed == FLs)
        {
            current_vals[1] = 0;
            current_vals[2] = 0;
            current_vals[3] = 0;
        }
    timer = 0;
    }

    delay(20);
}