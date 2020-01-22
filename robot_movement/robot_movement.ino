/******************************************************************************
 * Arduino Due
 * Contains the code for the robot's main controller
 * 
 * Can Bus Library - https://github.com/Seeed-Studio/CAN_BUS_Shield
 * LED Strip Library - https://github.com/sdsmt-robotics/ASME-2019/tree/master/led_strip_control
 * RoboClaw Library - https://www.basicmicro.com/downloads
******************************************************************************/
/******************************************************************************
 * TODO
 *      -remove delay
 *      -change deadman's to
 *          use millis()
 *      -add toggle for debug messages
******************************************************************************/
//#include <SoftwareSerial.h>
#include <SPI.h>
#include <mcp_can.h>
#include "RoboClaw.h"
#include "LedStrip.h"
/******************************************************************************
 * Defines
******************************************************************************/
//SPX Victor Motor Controllers
#define controller_one 3
#define controller_two 2
#define controller_three 4 
#define controller_four 1
#define controller_arm 5

//Max speed for robot movement
#define MAX_SPD 375.0
//#define MAX_SPD 10.0

//cs pin for SPI and CAN Bus
#define spiCSPin 53

//Address for control on RoboClaw
//motor controllers
#define rc_lift 0x80
#define rc_corral 0x81
#define rc_door 0x80
#define rc_door_bottom 0x83

//limit switch Defines
#define lim_bucket_top 27
#define lim_door_bottom_left 22
#define lim_door_top_left 23
#define lim_corral_right 24
#define lim_door_top_right 25
#define lim_door_bottom_right 26
#define lim_bucket_bottom 29


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

unsigned long delay_killswitch = 2500;
unsigned long millis_lastmove = 0;

int axis = 0;

//Test Time
int old_time = 0;
int timer = 0;
int oldspeed;

//corral / door variables
bool corral_moving = false;
bool door_moving = false;

unsigned long millis_current = 0;
unsigned long millis_door = 0;
unsigned long millis_corral = 0;
unsigned long delay_corral = 1500;
unsigned long delay_door = 1500;

bool rainbow = true;
unsigned long millis_led = 0;
unsigned long delay_fixed_led = 1500;
unsigned long millis_fixed_led = 0;
int led_r = 255;
int led_g = 0;
int led_b = 0;

short led_change_rate = 10;
short r_rate = 0;
short g_rate = 1;
short b_rate = 0;

//led contro
int brightness = 128;

/******************************************************************************
 * Object Declaration
******************************************************************************/

//create CAN object
MCP_CAN CAN(spiCSPin);

//create roboclaw objects
//RoboClaw rc_driver_door(&Serial3, 10000);
RoboClaw rc_driver_door(&Serial1, 10000);
RoboClaw rc_driver_corral(&Serial2, 10000);

//creat light control object
LedStrip ledStrip(5, 6, 7);

void setup()
{
    //pc debug
    Serial.begin(9600);

    ledStrip.setBrightness(brightness);
    ledStrip.setColorHEX(LedStrip::Red);
    //Serial.println("initializing CAN Bus...");
    //set the baudrate and let it know we are using the 8MHz oscilator on the CAN module
    while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_8MHz))
    {
        //Serial.println("CAN BUS init Failed");
        delay(100);
    }
    //Serial.println("CAN BUS Init OK!");   
    
    //xBee communication
    Serial.println("Starting Recieve Code");
    Serial3.begin(9600);
    // Clean up the xBee buffer so to remove lag.
    while ( Serial3.available() ) {
      Serial3.read();
    }
 
    //RoboClaw
    rc_driver_door.begin(38400);
    rc_driver_corral.begin(38400);

    millis_current = millis();
    millis_lastmove = millis();

    fix_led_color(0,255,255);

    millis_led = millis();
}

bool fix_led_color(int red, int green, int blue)
{
  // Only one fixed color at a time
  if(millis_fixed_led>0) {
    Serial.println("Already fixed color!");
    return false;
  }
  
  Serial.print("Fixing color to ");
  Serial.print(red);
  Serial.print(", ");
  Serial.print(green);
  Serial.print(", ");
  Serial.print(blue);
  Serial.println("!");
  
  millis_fixed_led = millis_current;
  ledStrip.setColorRGB(red, green, blue);
  return true;
}

void check_command()
{
//incoming_command[0] is the encoded value being converted to decimal
    if (incoming_command[0] <= 100 && incoming_command[0] >= 97)
    {
        axis = incoming_command[0];
        current_vals[(axis - 97)] = map(int(incoming_command[1]), 0, 200, -500, 500);
        millis_lastmove = millis();
        
        Serial.print("Joystick ");
        Serial.print(incoming_command[0], HEX);
        Serial.print(" ");
        Serial.print(map(incoming_command[1], 0, 200, -500, 500));
        Serial.println();
    }
    else if(incoming_command[0] == 'L')
    {
        //right bank, right button
        //lift stop
        rc_driver_door.ForwardM1(rc_lift, 0);
        rc_driver_door.ForwardM2(rc_lift, 0);
        rc_driver_corral.ForwardM1(rc_lift, 0);
        rc_driver_corral.ForwardM2(rc_lift, 0);
    }
    else if(incoming_command[0] == 'K')
    {
        //right bank, top button
        //door open
        rc_driver_door.ForwardM1(rc_lift, 30);
        rc_driver_door.ForwardM2(rc_lift, 30);

        door_moving = true;

        millis_door = millis();
    }
    else if(incoming_command[0] == 'J')
    {
        //right bank, bottom button
        //lift down
        rc_driver_door.BackwardM1(rc_lift, 30);
        rc_driver_door.BackwardM2(rc_lift, 30);

        door_moving = true;
        
        millis_door = millis();
    }
    else if(incoming_command[0] == 'E')
    {
        //left bank, right button
        //corral stop
        rc_driver_door.ForwardM1(rc_lift, 0);
        rc_driver_door.ForwardM2(rc_lift, 0);
        rc_driver_corral.ForwardM1(rc_lift, 0);
        rc_driver_corral.ForwardM2(rc_lift, 0);
    }
    else if(incoming_command[0] == 'F')
    {
        //left bank, top button
        //corral up
        rc_driver_corral.ForwardM1(rc_lift, 80);
        rc_driver_corral.ForwardM2(rc_lift, 80);
        fix_led_color(0,0,255);
        
        corral_moving = true;
        corral_moving = true;

        millis_corral = millis();
    }
    else if(incoming_command[0] == 'G')
    {
        //left bank, bottom button
        //corral down
        rc_driver_corral.BackwardM1(rc_lift, 127);
        rc_driver_corral.BackwardM2(rc_lift, 127);
        fix_led_color(255,0,0);

        corral_moving = true;

        millis_corral = millis();
        
    } else {
      Serial.print("Other CMD: ");
      Serial.println(incoming_command[0]);
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

void rainbow_led()
{
  if(!rainbow) return;
  if(millis_current - millis_led > 5)
    {
        led_r += r_rate*led_change_rate;
        led_g += g_rate*led_change_rate;
        led_b += b_rate*led_change_rate;

        if(led_r >= 255 && r_rate > 0) {
          led_r = 255;
          r_rate = 0;
          g_rate = 0;
          b_rate = -1;
        } else if(led_r <= 0 && r_rate < 0) {
          led_r = 0;
          r_rate = 0;
          g_rate = 0;
          b_rate = 1;
        }

        if(led_g >= 255 && g_rate > 0) {
          led_g = 255;
          r_rate = -1;
          g_rate = 0;
          b_rate = 0;
        } else if(led_g <= 0 && g_rate < 0) {
          led_g = 0;
          r_rate = 1;
          g_rate = 0;
          b_rate = 0;
        }

        if(led_b >= 255 && b_rate > 0) {
          led_b = 255;
          r_rate = 0;
          g_rate = -1;
          b_rate = 0;
        } else if(led_b <= 0 && b_rate < 0) {
          led_b = 0;
          r_rate = 0;
          g_rate = 1;
          b_rate = 0;
        }

        ledStrip.setColorRGB(led_r, led_g, led_b);
        millis_led = millis();
    }
}

void readFromXBee()
{
  while (Serial3.available() > 0)
    {
        incomingByte = Serial3.read();
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
        }
       Serial.println(incomingByte);
    }
}

void loop()
{
    readFromXBee();

    /*verL = SpeedControl(current_vals[1]); //Runs function to create speed components
    //Serial.println(verL);
    horL = SpeedControl(current_vals[2]);
    //Serial.println(horL);
    rotL = -(SpeedControl(current_vals[3])/7);
    //Serial.println(rotL);*/

    //Serial.print("VH speed: ");
    verL = current_vals[1]; //Runs function to create speed components
    //Serial.print(verL);
    //Serial.print(" - ");
    horL = current_vals[2];
    //Serial.println(horL);
    rotL = -(current_vals[3]/4);

    FRs = -(-verL - horL - rotL); //Creates the speeds for each motor.
    BRs = -(-verL + horL - rotL);
    FLs = -(verL - horL - rotL);
    BLs = -(verL + horL - rotL); 

    if (FLs > 0)
    {
        FLs += CALIBRAITON_FL_FOR;
    }
    if (FLs < 0)
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

    /*FRs = constrain(FRs, -MAX_SPD, MAX_SPD); //Restricts The output to motor
    BRs = constrain(BRs, -MAX_SPD, MAX_SPD);
    FLs = constrain(FLs, -MAX_SPD, MAX_SPD);
    BLs = constrain(BLs, -MAX_SPD, MAX_SPD);*/

    //Outputs the speeds to each motor.
    byte_encoder(dataFL, FLs);
    CAN.sendMsgBuf(CONTROL | controller_one | 0x01040000, 1, 8, dataFL);

    byte_encoder(dataFR, FRs);
    CAN.sendMsgBuf(CONTROL | controller_two | 0x01040000, 1, 8, dataFR);

    byte_encoder(dataBR, BRs);
    CAN.sendMsgBuf(CONTROL | controller_three | 0x01040000, 1, 8, dataBL);

    byte_encoder(dataBL, BLs);
    CAN.sendMsgBuf(CONTROL | controller_four | 0x01040000, 1, 8, dataBR);

    //send enable frame
    CAN.sendMsgBuf(ENABLE, 1, 8, enable);

    millis_current = millis();

    if(door_moving && millis_current - millis_door > delay_door)
    {
        door_moving = false;

        rc_driver_door.ForwardM1(rc_lift, 0);
        rc_driver_door.ForwardM2(rc_lift, 0);
    }

    if(corral_moving && millis_current - millis_corral > delay_corral)
    {
        corral_moving = false;

        rc_driver_corral.ForwardM1(rc_lift, 0);
        rc_driver_corral.ForwardM2(rc_lift, 0);
    }
    // Reduce fixed led time, and if done, clear fixed led
    if(millis_fixed_led>0)
    {
      if(millis_current - millis_fixed_led > delay_fixed_led) 
      {
        millis_fixed_led = 0;
      }
    } else {
      rainbow_led();
    }
    // kill switch
    if((current_vals[0]!=0 || current_vals[1]!=0 || current_vals[2]!=0 || current_vals[3]!=0) && (millis_current - millis_lastmove) > delay_killswitch) {
      Serial.print("Killing all movement, it's been more than ");
      Serial.print(delay_killswitch);
      Serial.println("ms since last controller movement");
      for(int i=0;i<4;i++) {
        if(current_vals[i] != 0) {
          current_vals[i] = 0;
        }
      }
    }
/*  }
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
*/
    //delay(20);
}
