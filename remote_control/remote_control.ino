/*****************************
 * Arduino Micro
 * Sends button values to the
 * robot.
*****************************/
#include <SoftwareSerial.h>

#define left_int 19     //Pin d2
#define right_int 18    //Pin d3
#define trig_int 1      //Pin d7

#define right_input A5  //Pin a5
#define left_input A4   //Pin a4
#define trig_input A10   //Pin d6

#define joy_ry A3       //Pin a3
#define joy_rx A2       //Pin a2
#define joy_ly A1       //Pin a1
#define joy_lx A0       //Pin a0

//variables for controls storage

//joystick
int joy_ry_raw = 0;
int joy_rx_raw = 0;
int joy_ly_raw = 0;
int joy_lx_raw = 0;

char joy_ry_map = 0;
char joy_rx_map = 0;
char joy_ly_map = 0;
char joy_lx_map = 0;

int joy_ry_old = 0;
int joy_rx_old = 0;
int joy_ly_old = 0;
int joy_lx_old = 0;

int right_val = 0;
int left_val = 0;
int trig_val = 0;

void read_right()
{
    Serial.println("Right button set pressed");
    right_val = analogRead(right_input);
    if (right_val < 378)
    {
        //joystick button
        Serial.write('A');
        Serial.write('X');
    }
    else if(right_val < 768)
    {
        //right
        Serial.write('B');
        Serial.write('X');
    }
    else if(right_val < 799)
    {
        //left
        Serial.write('C');
        Serial.write('X');
    }
    else if(right_val < 855)
    {
        //top
        Serial.write('D');
        Serial.write('X');
    }
    else
    {
        //bottom
        Serial.write('E');
        Serial.write('X');
    }
}
void read_left()
{
    Serial.println("Left button set pressed");
    left_val = analogRead(left_input);
    if (left_val < 450)
    {
        //joystick button
        Serial.write('F');
        Serial.write('X');
    }
    else if(left_val < 890)
    {
        //bottom
        Serial.write('G');
        Serial.write('X');
    }
    else if(left_val < 933)
    {
        //left
        Serial.write('H');
        Serial.write('X');
    }
    else if(left_val < 978)
    {
        //top
        Serial.write('I');
        Serial.write('X');
    }
    else
    {
        //right
        Serial.write('J');
        Serial.write('X');
    }
}
void read_trig()
{
    Serial.println("Trigger button set pressed");
    trig_val = analogRead(trig_input);

    if(trig_val < 790)
    {
        //button one
        Serial.write('K');
        Serial.write('X');
    }
    else if(trig_val < 870)
    {
        //button two
        Serial.write('L');
        Serial.write('X');
    }
    else if(trig_val < 953)
    {
        //button three
        Serial.write('M');
        Serial.write('X');
    }
    else
    {
        //button four
        Serial.write('N');
        Serial.write('X');
    }
}
void setup() 
{
    pinMode(joy_ry, INPUT); //joystick right y
    pinMode(joy_rx, INPUT); //joystick right x
    pinMode(joy_ly, INPUT); //joystick left y
    pinMode(joy_lx, INPUT); //joystick left x

    pinMode(right_input, INPUT);
    pinMode(left_input, INPUT);

    attachInterrupt(1, read_right, RISING);
    attachInterrupt(0, read_left, RISING);
    attachInterrupt(6, read_trig, RISING);

    Serial.begin(9600); //open serial port
    Serial.println("Starting Send Code");
}

void loop() 
{
    //read joystick values
    joy_ry_raw = analogRead(joy_ry); 
    joy_rx_raw = analogRead(joy_rx);
    joy_ly_raw = analogRead(joy_ly);
    joy_lx_raw = analogRead(joy_lx);

    if (abs(joy_ry_raw - joy_ry_old) > 3) //value has changed by +3/-3
    {
        joy_ry_map = map(joy_ry_raw, 0, 1024, 0, 254);
        Serial.write('a');
        Serial.write(joy_ry_map);
        Serial.write('X');
        joy_ry_old = joy_ry_raw;
        
        Serial.print("Right y axis: ");
        Serial.print(joy_ry_raw);
        Serial.print(" ");
        Serial.println(joy_ry_old);
    }
    
    if (abs(joy_rx_raw - joy_rx_old) > 3) //value has changed by +3/-3
    {
        joy_rx_map = map(joy_rx_raw, 0, 1024, 0, 254);
        Serial.write('b');
        Serial.write(joy_rx_map);
        Serial.write('X');
        joy_rx_old = joy_rx_raw;

        Serial.print("Right x axis: ");
        Serial.print(joy_rx_raw);
        Serial.print(" ");
        Serial.println(joy_rx_old);
    }
    
    if (abs(joy_ly_raw - joy_ly_old) > 3) //value has changed by +3/-3
    {
        joy_ly_map = map(joy_ly_raw, 0, 1024, 0, 254);
        Serial.write('c');
        Serial.write(joy_ly_map);
        Serial.write('X');
        joy_ly_old = joy_ly_raw;

        Serial.print("Left y axis: ");
        Serial.print(joy_ly_raw);
        Serial.print(" ");
        Serial.println(joy_ly_old);
    }

    if (abs(joy_lx_raw - joy_lx_old) > 3) //value has changed by +3/-3
    {
        joy_lx_map = map(joy_lx_raw, 0, 1024, 0, 254);
        Serial.write('d');
        Serial.write(joy_lx_map);
        Serial.write('X');
        joy_lx_old = joy_lx_raw;

        Serial.print("Left x axis: ");
        Serial.print(joy_lx_raw);
        Serial.print(" ");
        Serial.println(joy_lx_old);
    }

    delay(20);
}

