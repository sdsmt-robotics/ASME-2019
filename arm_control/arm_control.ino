/*****************************
 * Arduino Nano
 * Sensor input and motor 
 * control for robot arm
*****************************/
#include <SoftwareSerial.h>
#include <SPI.h>
#include <mcp_can.h>

//control pins
#define control_enable 2  //pin D2
#define control_in 3       //pin D3
#define control_out 4      //pin D4

//limit switches
#define limit_closed 5     //pin D5
#define limit_open 6       //pin D6

//spi pin for CS
#define spiCSPin 10
#define motor_controller 5

int control_status = LOW;
int motor_speed;

unsigned char can_data[8] = {0,0,0,0,0,0,0,0}; //stores the motor control data

const uint32_t CONTROL = 0x040080; //frame which is used for percent output.

const uint32_t ENABLE = 0x0401BF; //ID used for sending the enable frame

unsigned char enable[8] = {1, 0, 0, 0, 0, 0, 0, 0}; //data for the ENABLE frame

//create CAN object
MCP_CAN CAN(spiCSPin);


void set_speed(unsigned char data[8], int SPD) //Allows Can Bus to Read speeds.
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

void setup()
{
    pinMode(control_enable, INPUT);
    pinMode(control_in, INPUT);
    pinMode(control_out, OUTPUT);

    pinMode(limit_closed, INPUT);
    pinMode(limit_open, INPUT);
    
    Serial.begin(115200);

    Serial.println("initializing...");

    //set the baudrate and let it know we are using the 8MHz oscilator on the CAN module
    while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_8MHz))
    {
        Serial.println("CAN BUS init Failed");
        delay(100);
    }
    Serial.println("CAN BUS Shield Init OK!");
    }

void loop()
{
    control_status = digitalRead(control_enable);
    //not sure if should be hold high for move
    if(control_status == HIGH) //if we recieve a pulse 
    {
        Serial.println("Recieved Enable Command");
        
        motor_speed = 100;
        set_speed(can_data, motor_speed);
        
        while(digitalRead(limit_closed) == LOW && digitalRead(limit_open) == LOW)
        {
          Serial.println("Moving Forward");
            //motor forward
            CAN.sendMsgBuf(CONTROL | motor_controller | 0x01040000, 1, 8, can_data);
            CAN.sendMsgBuf(ENABLE, 1, 8, enable);
            delay(20);
        }

        Serial.println("Stopping");
        
        motor_speed = 0;
        
        set_speed(can_data, motor_speed);
        CAN.sendMsgBuf(CONTROL | motor_controller | 0x01040000, 1, 8, can_data);
        CAN.sendMsgBuf(ENABLE, 1, 8, enable);
        delay(20);
        //reverse speed
        motor_speed = -100;
        set_speed(can_data, motor_speed);

        while(digitalRead(limit_closed) == LOW && digitalRead(limit_open) == LOW)
        {
          Serial.println("Moving Backward");
            //motor backward
            CAN.sendMsgBuf(CONTROL | motor_controller | 0x01040000, 1, 8, can_data);
            CAN.sendMsgBuf(ENABLE, 1, 8, enable);
            delay(20);
        }
        Serial.println("Ending Loop");
        motor_speed = 0;
        
        set_speed(can_data, motor_speed);
        CAN.sendMsgBuf(CONTROL | motor_controller | 0x01040000, 1, 8, can_data);
        CAN.sendMsgBuf(ENABLE, 1, 8, enable);
        delay(20);
    }
    delay(20);
}
