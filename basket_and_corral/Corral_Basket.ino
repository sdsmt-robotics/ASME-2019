#include <SoftwareSerial.h>
#include "RoboClaw.h"

#define address 0x80
#define BasketPin1 10
#define BasketPin2 11
#define CorralPin1 12
#define CorralPin2 13

SoftwareSerial serial(CorralPin1,CorralPin2);
RoboClaw Corral(&Serial2, 10000);

SoftwareSerial serial(BasketPin1,BasketPin2);  
RoboClaw Basket(&serial,10000);

bool CorralDown = false;
bool CorralUp = false;
bool CorralUse = false;

bool BasketMove = false;
char Move = 'u';

void setup() 
{
  Corral.begin(38400);
  Basket.begin(38400)
}

void loop() 
{ 
  if(/*Button Press for down*/ && !CorralDown)
  {
    CorralDown = true;
    if (CorralUP)
    {
        Corral.ForwardMixed(address, 0);
    }
    CorralUse = true;
  }
  if(/*Button Press for up*/ && !CorralUp)
  {
    CorralUp = true;
    if (CorralUP)
    {
        Corral.BackwardMixed(address, 0);
    }
    CorralUse = true;
  }
  if(CorralDown && CorralUse)
  {
    //Start motors at half speed
    Corral.ForwardM1(address,64); 
    CorralUse = false;
  }
  if(CorralUp && CorralUse)
  {
    Corral.BackwardMixed(address,64);
    CorralUse = false; 
  }
  if(/*Bottom Switch*/)
  {
    Corral.BackwardMixed(address, 0);
  }
  if(/*Top Switch*/)
  {
    Corral.ForwardMixed(address, 0);
  }

  //Basket Controls
  if (/*Button to active basket*/)
  {
    BasketMove = true;
  }
  
  if (BasketMove && Move == 'u')
  {
    Basket.ForwardM1(address,64); //start Motor1 forward at half speed
    Basket.BackwardM2(address,64); //start Motor2 backward at half speed
  }
  if (BasketMove && Move == down/* Trigger switch Top*/)
  {
    Basket.BackwardM1(address,64);
    Basket.ForwardM2(address,64);
    Move = 'd';
  }
  if (/*Trigger switch bottom*/)
  {
    //Moves Basket off switch
    Basket.ForwardM1(address,20); //start Motor1 forward at half speed
    Basket.BackwardM2(address,20); //start Motor2 backward at half speed
    delay(100);
    //Stops Switch and fixes directions
    Basket.ForwardM1(address,0); 
    Basket.BackwardM2(address,0);
    Move = 'u';
    BasketMove = false;
  }
}